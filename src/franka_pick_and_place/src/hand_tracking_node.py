import rospy
import tf2_ros
import os
import time
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image
import mediapipe as mp
import numpy as np
import networkx as nx
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from matplotlib.figure import Figure
from franka_pick_and_place.msg import HandStatus
from scipy.optimize import linear_sum_assignment

hand_objects = ["Hand1", "Hand2", "Hand3"]
robot = "panda_link8"

hand_status_pub = rospy.Publisher("/hand_status", HandStatus, queue_size=10)

# Create a directory for graph snapshots
snapshot_dir = "graph_snapshots"
os.makedirs(snapshot_dir, exist_ok=True)

snapshot_counter = 0

# Store latest pose messages from topics
hand_poses = {}

# Map MediaPipe hand index to world hand name
mediapipe_to_world_map = {}

# Hand detection bounds
hand_bounds = {"x": (-3.4, -2.5), "y": (2.5, 5.5), "z": (0.8, 1.6)}

hand_graph = nx.Graph()

fixed_positions = {
    "robot": [0, 1],  # Top center
    "left": [2, -1],  # Bottom left
    "center": [0, -1],  # Bottom center
    "right": [-2, -1],  # Bottom right
}

fixed_nodes = list(fixed_positions.keys())

node_thresholds = {"left": 0.2, "right": 0.2, "center": 0.6}

for node, threshold in node_thresholds.items():
    hand_graph.add_node(node)
    hand_graph.add_edge("robot", node, distance=threshold)

# OpenCV Bridge
bridge = CvBridge()

# TF Buffer and Listener (to be initialized after init_node)
tf_buffer = None
listener = None

# MediaPipe setup
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    model_complexity=1,
    static_image_mode=False,
    max_num_hands=5,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5,
)
mp_draw = mp.solutions.drawing_utils

"""
Category 1: Controlled stop with power maintained. 0.735s

Category 2: Controlled stop with power removed after stop. 0.743s

dmin=v⋅Tstop≤dreach=0.855m

Using the worst-case stop time (Category 2, Axis 1):
Tstop=0.743s⇒vsafe=0.8550.743≈1.15m/s

Typical "safe collaboration" speed:
250-500mm/s(as per ISO/TS 15066:2016)

ISO 10218-2 suggests:
Vmin=250mm/s=0.25m/s"""

d_reach = 0.855
alpha = 1.5
time_to_stop = 0.743  # s
velocity = 0.5  # m/s <= 1.15 m/s


def get_wrist_pose_from_landmarks(hand_landmarks):
    """Extract the wrist pose from MediaPipe hand landmarks.
    Args:
        hand_landmarks: MediaPipe hand landmarks object.
    Returns:
        Pose: A Pose object containing the wrist position.
    """
    wrist = hand_landmarks.landmark[mp_hands.HandLandmark.WRIST]
    pose = Pose()
    pose.position.x = wrist.x
    pose.position.y = wrist.y
    pose.position.z = wrist.z
    return pose


def hand_pose_callback(msg, hand_name):
    """
    Callback function for hand pose messages.
    Updates the global hand_poses dictionary with the latest pose for the specified hand.
    Args:
        msg (Pose): The Pose message containing the hand's position and orientation.
        hand_name (str): The name of the hand (e.g., "Hand1", "Hand2", "Hand3").
    Returns:
        None
    """
    hand_poses[hand_name] = msg


def get_object_pose_tf(object_name):
    """
    Get the pose of an object in the "world" frame using TF.
    Args:
        object_name (str): The name of the object to look up in TF.
    Returns:
        Pose: The pose of the object in the "world" frame, or None if not found.
    """
    try:
        transform = tf_buffer.lookup_transform(
            "world", object_name, rospy.Time(0), rospy.Duration(3.0)
        )
        pose = Pose()
        pose.position.x = transform.transform.translation.x
        pose.position.y = transform.transform.translation.y
        pose.position.z = transform.transform.translation.z
        pose.orientation = transform.transform.rotation
        return pose
    except (
        tf2_ros.LookupException,
        tf2_ros.ConnectivityException,
        tf2_ros.ExtrapolationException,
    ):
        rospy.logerr(f"Could not get TF transform for {object_name}")
        return None


def calculate_euclidean_distance(pose1, pose2):
    """
    Calculate the Euclidean distance between two Pose objects.
    Args:
        pose1 (Pose): The first Pose object.
        pose2 (Pose): The second Pose object.
    Returns:
        float: The Euclidean distance between the two poses, rounded to 2 decimal places.
    """
    dx = pose1.position.x - pose2.position.x
    dy = pose1.position.y - pose2.position.y
    dz = pose1.position.z - pose2.position.z
    distance = np.sqrt(dx**2 + dy**2 + dz**2)
    return round(distance, 2)


def compute_risk_score(distance, velocity, time_to_stop, d_reach, alpha):
    """
    Compute distance-based hazard (risk) score based on ISO/TS 15066.
    Args:
        distance (float): Current distance between human and robot (d_H)
        velocity (float): Robot's velocity (m/s)
        time_to_stop (float): Time required to stop robot (s)
        d_reach (float): Robot's maximum physical reach (m)
        alpha (float): Sensitivity coefficient for exponential decay
    Returns:
        float: Risk score (0 to 1)
    """
    d_min = velocity * time_to_stop  # Minimum safety distance in meters
    print(f"Distance: {distance:.2f} | d_min: {d_min:.2f} | d_reach: {d_reach:.2f}")
    d_H = distance

    if d_H >= d_reach:
        risk = 0.0
    elif d_H <= d_min:
        risk = 1.0
    else:  # d_min < d_H < d_reach
        risk = np.exp(-alpha * (d_H - d_min))

    return round(risk, 2)


def get_hand_region(hand_pose, robot_pos, threshold=0.35):
    """
    Determine the region of the hand relative to the robot's position.
    Args:
        hand_pose (Pose): Pose of the hand in the "world" frame.
        robot_pos (Pose): Pose of the robot in the "world" frame.
        threshold (float): Threshold distance to determine left/right/center.
    Returns:
        str: The relative position of the hand ("left", "right", or "center").
    """
    y_hand = hand_pose.position.y
    y_robot = robot_pos.position.y

    if y_hand > y_robot + threshold:
        return "left"
    elif y_hand < y_robot - threshold:
        return "right"
    else:
        return "center"


def assess_hand_risk(robot_pos, hand_pose, velocity, time_to_stop, d_reach, alpha):
    """
    Assess the risk of collision between a hand and the robot.
    Args:
        hand_name (str): Name of the hand (e.g., "Hand1").
        robot_pos (Pose): Pose of the robot in the "world" frame.
        hand_pose (Pose): Pose of the hand in the "world" frame.
        velocity (float): Robot's velocity (m/s).
        time_to_stop (float): Time required to stop robot (s).
        d_reach (float): Robot's maximum physical reach (m).
        alpha (float): Sensitivity coefficient for exponential decay.
    Returns:
        dict: A dictionary containing relative position, y-difference, distance,
              risk score, and collision status.
    """
    y_diff = abs(hand_pose.position.y - robot_pos.position.y)
    threshold = 0.35

    rel_pos = get_hand_region(hand_pose, robot_pos, threshold)

    dist_to_robot = calculate_euclidean_distance(hand_pose, robot_pos)
    risk_score = compute_risk_score(
        dist_to_robot, velocity, time_to_stop, d_reach, alpha
    )

    if risk_score <= 0.3:
        collision_status = "LOW"
    elif 0.3 < risk_score <= 0.7:
        collision_status = "MEDIUM"
    else:
        collision_status = "HIGH"

    return {
        "rel_pos": rel_pos,
        "y_diff": y_diff,
        "dist": dist_to_robot,
        "risk": risk_score,
        "status": collision_status,
    }


def probabilistic_or(risks):
    """
    Compute probabilistic OR of a list of risk values.
    R = 1 - product(1 - Ri)
    where Ri is the risk value of each hand.
    Args:
        risks (list): List of risk values (0 to 1).
    Returns:
        float: Combined risk value using probabilistic OR.
        The result is a float between 0 and 1.
    """
    product = 1.0
    for r in risks:
        product *= 1 - r
    return 1 - product


def compute_region_risks(graph):
    """
    For each region node (left, center, right), combine the risks of all hands connected
    to that region using probabilistic OR.
    Args:
        graph (nx.Graph): The hand graph containing region nodes and hand edges.
    Returns:
        dict: A dictionary with region names as keys and their combined risk values as values.
        If no hands are connected to a region, the risk is set to 0.0.
    """
    region_risks = {}
    for region in ["left", "center", "right"]:
        # Collect all risk values of hands connected to this region
        hand_risks = []
        for neighbor in graph.neighbors(region):
            # Skip if neighbor is robot or another region node
            if neighbor in ["robot", "left", "center", "right"]:
                continue
            edge_data = graph.get_edge_data(region, neighbor)
            if edge_data and "risk_value" in edge_data:
                hand_risks.append(edge_data["risk_value"])

        if hand_risks:
            region_risks[region] = probabilistic_or(hand_risks)
        else:
            region_risks[region] = 0.0

    return region_risks


def compute_robot_risk(region_risks, priorities, graph):
    """
    Compute overall robot risk:
    - Determine the critical region using weighted region risk
    - In that region, find the hand with the highest raw risk
    Args:
        region_risks (dict): Dictionary of region risks (left, center, right).
        priorities (dict): Dictionary of region priorities (weights).
        graph (nx.Graph): The hand graph containing region nodes and hand edges.
    Returns:
        tuple: (robot_risk, critical_region, critical_hand, max_risk)
        - robot_risk: Overall risk value for the robot.
        - critical_region: The region with the highest weighted risk.
        - critical_hand: The hand with the highest raw risk in that region.
        - max_risk: The raw risk value of the critical hand.
    """
    # Step 1: Compute total weight
    total_weight = sum(priorities.values())
    if total_weight == 0:
        return 0.0, None, None, 0.0

    # Step 2: Compute weighted risks
    weighted_risks = {r: priorities[r] * region_risks.get(r, 0.0) for r in priorities}
    total_weighted_risk = sum(weighted_risks.values())

    if total_weighted_risk == 0:
        return 0.0, None, None, 0.0

    # Step 3: Identify critical region (region with highest weighted risk)
    critical_region = max(weighted_risks, key=weighted_risks.get)
    robot_risk = weighted_risks[critical_region] / total_weight

    # Step 4: Find critical hand in that region (with max raw risk)
    max_risk = 0.0
    critical_hand = None
    for neighbor in graph.neighbors(critical_region):
        if neighbor in ["robot", "left", "center", "right"]:
            continue
        edge_data = graph.get_edge_data(critical_region, neighbor)
        if edge_data and "risk_value" in edge_data:
            if edge_data["risk_value"] > max_risk:
                max_risk = edge_data["risk_value"]
                critical_hand = neighbor

    return robot_risk, critical_region, critical_hand, max_risk


def remove_hand_from_graph(hand_name):
    """
    Remove a hand from the graph, including all edges connected to it.
    Args:
        hand_name (str): The name of the hand to remove.
    Returns:
        None
    """
    if hand_name in hand_graph.nodes:
        print(f"Removing {hand_name} from graph")

        # Remove edges connected to the hand node
        neighbors = list(hand_graph.neighbors(hand_name))
        for neighbor in neighbors:
            hand_graph.remove_edge(neighbor, hand_name)
        # Remove the node itself
        hand_graph.remove_node(hand_name)

        print(f"Removed {hand_name} from graph")
    else:
        print(f"{hand_name} not found in graph")


def add_hand_to_graph(
    hand_name, hand_pose, rel_pos, dist_to_robot, risk_score, collision_status
):
    """
    Add a hand to the graph with its relative position and risk score.
    Args:
        hand_name (str): The name of the hand to add.
        hand_pose (Pose): The Pose of the hand in the "world" frame.
        rel_pos (str): Relative position of the hand ("left", "right", "center").
        dist_to_robot (float): Distance from the hand to the robot.
        risk_score (float): Risk score for this hand.
        collision_status (str): Collision status ("LOW", "MEDIUM", "HIGH").
    Returns:
        None
    """
    robot_pos = get_object_pose_tf(robot)
    if robot_pos:
        # Remove the hand if it already exists
        if hand_name in hand_graph.nodes:
            hand_graph.remove_node(hand_name)
            print(f"Removed existing {hand_name} before adding new one.")

        # Add new hand node
        hand_graph.add_node(hand_name, pos=(hand_pose.position.x, hand_pose.position.y))
        hand_graph.add_edge(
            rel_pos,
            hand_name,
            distance=dist_to_robot,
            risk_value=risk_score,
            risk_level=collision_status,
        )
        print(f"Added {hand_name} under {rel_pos} with distance {dist_to_robot:.2f}")
    else:
        print(f"Robot not found for {hand_name}")


# Graph display function
def display_graph_opencv(graph):
    """Display the hand graph using OpenCV and Matplotlib.
    Args:
        graph (nx.Graph): The hand graph to display.
    Returns:
        None
    """
    global snapshot_counter

    fig = Figure(figsize=(8, 6))
    canvas = FigureCanvas(fig)
    ax = fig.add_subplot(111)

    # Spring layout with fixed base node positions
    pos = nx.spring_layout(graph, pos=fixed_positions, fixed=fixed_nodes, k=1)

    # Draw graph
    nx.draw(
        graph,
        pos,
        with_labels=True,
        node_size=1000,
        node_color="lightblue",
        font_size=10,
        font_weight="bold",
        ax=ax,
    )

    # Draw edge labels
    edge_labels = {}
    for u, v, data in graph.edges(data=True):
        if {u, v} <= {"robot", "left", "right", "center"}:
            dist = data.get("distance", 0.0)

            region_node = v if u == "robot" else u
            or_risk = graph.nodes.get(region_node, {}).get("or_risk", None)
            if or_risk is not None:
                edge_labels[(u, v)] = f"{dist:.2f} | RISK: {or_risk:.2f}"
            else:
                edge_labels[(u, v)] = f"{dist:.2f} | RISK: N/A"
        else:
            dist = data.get("distance", 0.0)
            risk_val = data.get("risk_value", 0.0)
            risk_lvl = data.get("risk_level", "N/A")
            edge_labels[(u, v)] = f"{dist:.2f} | {risk_val:.2f} | {risk_lvl.upper()}"

    nx.draw_networkx_edge_labels(
        graph,
        pos,
        edge_labels=edge_labels,
        ax=ax,
        font_size=8,
        font_color="black",
        label_pos=0.5,
        rotate=False,
        bbox=dict(facecolor="white", edgecolor="none", boxstyle="round,pad=0.2"),
    )

    # Annotate robot node with full risk summary
    for node, (x, y) in pos.items():
        if node == "robot":
            critical_region = graph.nodes[node].get("critical_region", "N/A")
            critical_hand = graph.nodes[node].get("critical_hand", "N/A")
            critical_hand_risk = graph.nodes[node].get("critical_hand_risk", None)

            label_lines = []
            if critical_region and critical_region in graph.nodes:
                region_to_robot_risk = graph.nodes[critical_region].get("or_risk", None)
                if region_to_robot_risk is not None:
                    label_lines.append(f"Robot Risk: {region_to_robot_risk:.2f}")

            label_lines.append(f"Critical Region: {critical_region}")
            label_lines.append(f"Critical Hand: {critical_hand}")
            if critical_hand_risk is not None:
                label_lines.append(f"Hand Risk: {critical_hand_risk:.2f}")

            for i, line in enumerate(label_lines):
                ax.text(
                    x,
                    y + 0.15 + i * 0.1,
                    line,
                    fontsize=8,
                    ha="center",
                    color="darkred",
                )

    # Convert to OpenCV image
    canvas.draw()
    buf = np.asarray(canvas.buffer_rgba())
    image_bgr = cv2.cvtColor(buf, cv2.COLOR_RGBA2BGR)

    # === Save image to file ===
    timestamp = time.strftime("%Y%m%d-%H%M%S")
    filename = f"{snapshot_dir}/graph_{snapshot_counter:04d}_{timestamp}.png"
    cv2.imwrite(filename, image_bgr)
    snapshot_counter += 1

    # Show image
    cv2.imshow("Hand Graph", image_bgr)
    cv2.waitKey(1)

# To match MediaPipe hand poses to world hand poses, we need to define a class for normalized positions and poses.
class NormalizedPosition:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


class NormalizedPose:
    def __init__(self, x, y, z):
        self.position = NormalizedPosition(x, y, z)


def normalize_pose(pose, bounds):
    norm_x = (pose.position.x - bounds["x"][0]) / (bounds["x"][1] - bounds["x"][0])
    norm_y = (pose.position.y - bounds["y"][0]) / (bounds["y"][1] - bounds["y"][0])
    norm_z = (pose.position.z - bounds["z"][0]) / (bounds["z"][1] - bounds["z"][0])
    return NormalizedPose(norm_x, norm_y, norm_z)


def match_hands_hungarian(
    media_poses,
    available_hands,
    cost_threshold=None,
    verbose=True,
    prev_mapping=None,
    switch_threshold=0.15,
):
    """
    Match MediaPipe hand poses to available world hand poses using the Hungarian algorithm.

    Args:
        media_poses (dict): Dictionary of MediaPipe hand poses indexed by their indices.
        available_hands (dict): Dictionary of available world hand poses indexed by their names.
        cost_threshold (float, optional): Maximum allowed cost for a match. Defaults to None.
        verbose (bool, optional): Whether to print detailed matching information. Defaults to True.
        prev_mapping (dict, optional): Previous mapping to reuse if possible. Defaults to None.
        switch_threshold (float, optional): Threshold for switching from previous mapping. Defaults to 0.15.

    Returns:
        dict: Mapping of MediaPipe indices to world hand names.
        If no valid matches are found, returns an empty dictionary.
    """
    if not media_poses or not available_hands:
        return {}

    media_indices = list(media_poses.keys())
    world_hand_names = list(available_hands.keys())
    num_m = len(media_indices)
    num_w = len(world_hand_names)

    # Cost matrix
    cost_matrix = np.zeros((num_m, num_w))
    for i, media_idx in enumerate(media_indices):
        for j, hand_name in enumerate(world_hand_names):
            media_pose = media_poses[media_idx]
            world_pose = available_hands[hand_name]
            norm_world_pose = normalize_pose(world_pose, hand_bounds)
            dist = calculate_euclidean_distance(media_pose, norm_world_pose)
            cost_matrix[i, j] = dist
            if verbose:
                print(f"Cost for MP {media_idx} ↔ {hand_name}: {dist:.3f}")

    # Solve assignment problem
    row_idx, col_idx = linear_sum_assignment(cost_matrix)
    mapping = {}

    for r, c in zip(row_idx, col_idx):
        mp_idx = media_indices[r]
        hand_name = world_hand_names[c]
        dist = cost_matrix[r, c]

        if prev_mapping and mp_idx in prev_mapping:
            prev_hand = prev_mapping[mp_idx]
            if prev_hand in available_hands:
                prev_dist = calculate_euclidean_distance(
                    media_poses[mp_idx],
                    normalize_pose(available_hands[prev_hand], hand_bounds),
                )
                # Only switch match if new match is significantly better
                if dist > prev_dist - switch_threshold:
                    hand_name = prev_hand
                    dist = prev_dist
                    if verbose:
                        print(
                            f"[REUSED] MP {mp_idx} → {hand_name} | Prev dist: {prev_dist:.3f}"
                        )

        # Cost threshold (None in this case)
        if cost_threshold is None or dist <= cost_threshold:
            mapping[mp_idx] = hand_name
            if verbose:
                print(f"[MATCH] MP {mp_idx} → {hand_name} | Distance: {dist:.3f}")
        elif verbose:
            print(
                f"[SKIPPED] MP {mp_idx} → {hand_name} | Distance {dist:.3f} exceeds threshold {cost_threshold:.3f}"
            )

    return mapping


def camera_callback(msg):
    """
    Callback function for camera messages.
    Processes the camera frame, detects hands using MediaPipe, and updates the hand graph.

    Args:
        msg (Image): The Image message containing the camera frame.

    Returns:
        None
    """
    global mediapipe_to_world_map
    current_frame_map = {}
    used_world_hands = set()

    try:
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
        image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Process with MediaPipe
        results = hands.process(image_rgb)

        print("\n Current world hand poses:")
        for hand in hand_objects:
            if hand in hand_poses:
                p = hand_poses[hand]
                print(
                    f"{hand} (world): x={p.position.x:.2f}, y={p.position.y:.2f}, z={p.position.z:.2f}"
                )
            else:
                print(f"{hand} pose not received yet")

        robot_pos = get_object_pose_tf(robot)
        if robot_pos:
            print(
                f"Robot position: x={robot_pos.position.x:.2f}, y={robot_pos.position.y:.2f}, z={robot_pos.position.z:.2f}"
            )
        else:
            print("Robot position not found")

        if results.multi_hand_landmarks:

            media_poses = {}
            h, w, _ = frame.shape

            for i, hand_landmarks in enumerate(results.multi_hand_landmarks):
                mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

                for lm in hand_landmarks.landmark:
                    cx, cy = int(lm.x * w), int(lm.y * h)
                    cv2.circle(frame, (cx, cy), 3, (0, 255, 0), -1)

                mp_pose = get_wrist_pose_from_landmarks(hand_landmarks)
                media_poses[i] = mp_pose  # store by index

                # Filter available hands (not used and inside bounds)
                available_hands = {
                    name: pose
                    for name, pose in hand_poses.items()
                    if name not in used_world_hands
                    and hand_bounds["x"][0] <= pose.position.x <= hand_bounds["x"][1]
                    and hand_bounds["y"][0] <= pose.position.y <= hand_bounds["y"][1]
                    and hand_bounds["z"][0] <= pose.position.z <= hand_bounds["z"][1]
                }

            print(f"Available hands: {list(available_hands.keys())}")

            if media_poses and available_hands:
                matched = match_hands_hungarian(media_poses, available_hands)

                for media_idx, hand_name in matched.items():
                    if robot_pos and hand_name in hand_poses:
                        used_world_hands.add(hand_name)
                        current_frame_map[media_idx] = hand_name

                        result = assess_hand_risk(
                            robot_pos,
                            hand_poses[hand_name],
                            velocity,
                            time_to_stop,
                            d_reach,
                            alpha,
                        )

                        print(
                            f"[INFO] Hand '{hand_name}' | MP idx {media_idx} | Y diff: {result['y_diff']:.3f} | Rel pos: {result['rel_pos']} | Dist: {result['dist']:.3f} | Risk: {result['risk']:.3f} | Status: {result['status']}"
                        )

                        add_hand_to_graph(
                            hand_name,
                            hand_poses[hand_name],
                            result["rel_pos"],
                            result["dist"],
                            result["risk"],
                            result["status"],
                        )

                        # Publish status
                        hand_status = HandStatus()
                        hand_status.hand_name = hand_name
                        hand_status.detected = True
                        hand_status.distance_to_robot = result["dist"]
                        hand_status.relative_position = result["rel_pos"]
                        hand_status.y_diff = result["y_diff"]
                        hand_status.risk_level = result["status"]
                        hand_status_pub.publish(hand_status)

            # Update global mapping
            mediapipe_to_world_map = current_frame_map
        else:
            mediapipe_to_world_map.clear()
            cv2.putText(
                frame,
                "No hands detected",
                (50, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 0, 255),
                2,
            )

        # Remove hands not detected this frame from graph
        for hand_name in list(hand_graph.nodes):
            if hand_name not in used_world_hands and hand_name not in [
                "robot",
                "left",
                "right",
                "center",
            ]:
                remove_hand_from_graph(hand_name)

        # Compute region risks and update graph nodes
        region_risks = compute_region_risks(hand_graph)
        for region, risk in region_risks.items():
            if region in hand_graph.nodes:
                hand_graph.nodes[region]["or_risk"] = round(risk, 3)

        robot_risk, critical_region, critical_hand, critical_hand_risk = (
            compute_robot_risk(region_risks, node_thresholds, hand_graph)
        )

        # Update robot node
        if "robot" in hand_graph.nodes:
            hand_graph.nodes["robot"]["weighted_risk"] = round(robot_risk, 3)
            hand_graph.nodes["robot"]["critical_region"] = critical_region
            hand_graph.nodes["robot"]["critical_hand"] = critical_hand
            hand_graph.nodes["robot"]["critical_hand_risk"] = round(
                critical_hand_risk, 3
            )

        # Print final report
        print(f"\nRobot overall risk: {robot_risk:.2f}")
        print(f"Critical region: {critical_region}")
        print(f"Critical hand: {critical_hand} | Raw risk: {critical_hand_risk:.2f}\n")

        # Display graph and camera feed
        display_graph_opencv(hand_graph)

        if critical_region and critical_hand:
            cv2.putText(
                frame,
                f"Critical Region: {critical_region}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (102, 51, 0),
                2,
            )
            cv2.putText(
                frame,
                f"Critical Hand: {critical_hand} | Risk: {critical_hand_risk:.2f}",
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (102, 51, 0),
                2,
            )

        cv2.imshow("Camera Feed", frame)

        # Exit on 'q' press
        if cv2.waitKey(1) & 0xFF == ord("q"):
            rospy.signal_shutdown("User requested shutdown")

    except Exception as e:
        rospy.logwarn(f"Error in camera callback: {e}")


def main():
    """
    Main function to initialize the ROS node and subscribers.
    Sets up the TF listener and subscribes to hand pose and camera topics.
    """
    global tf_buffer, listener
    rospy.init_node("hand_pose_listener", anonymous=True)

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    rospy.Subscriber("/rgb", Image, camera_callback)

    for hand in hand_objects:
        rospy.Subscriber(
            f"/{hand.lower()}/pose",
            Pose,
            lambda msg, h=hand: hand_pose_callback(msg, h),
        )

    rospy.loginfo("Hand tracking node running...")
    rospy.spin()


if __name__ == "__main__":
    main()