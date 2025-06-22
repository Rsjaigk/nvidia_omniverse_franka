import rospy
import tf2_ros
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

hand_objects = ["Hand1", "Hand2", "Hand3"]
robot = "panda_link8"

hand_status_pub = rospy.Publisher("/hand_status", HandStatus, queue_size=10)

# Store latest pose messages from topics
hand_poses = {}

# Map MediaPipe hand index to world hand name
mediapipe_to_world_map = {}

# Hand detection bounds
hand_bounds = {
    "x": (-3.4, -2.2),
    "y": (2.5, 5.5),
    "z": (0.8, 1.6)
}

hand_graph = nx.Graph()

fixed_positions = {
    "robot": [0, 1],        # Top center
    "left": [2, -1],       # Bottom left
    "center": [0, -1],      # Bottom center
    "right": [-2, -1],       # Bottom right
}

fixed_nodes = list(fixed_positions.keys())

node_thresholds = {
    "left": 0.2,
    "right": 0.2,
    "center": 0.6
}

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
    min_tracking_confidence=0.5
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
time_to_stop = 0.743 #s
velocity = 0.5 #m/s <= 1.15 m/s 

def match_to_closest_hand(detected_pose, known_poses):
    # Match detected pose to closest known pose (world hands). 
    closest_hand = None
    min_dist = float('inf')
    for name, pose in known_poses.items():
        dist = np.linalg.norm([
            pose.position.x - detected_pose.position.x,
            pose.position.y - detected_pose.position.y,
            pose.position.z - detected_pose.position.z
        ])
        if dist < min_dist:
            min_dist = dist
            closest_hand = name
    return closest_hand

def get_wrist_pose_from_landmarks(hand_landmarks):
    # Approximate wrist 3D pose from normalized image coordinates. 
    wrist = hand_landmarks.landmark[mp_hands.HandLandmark.WRIST]
    pose = Pose()
    pose.position.x = wrist.x
    pose.position.y = wrist.y
    pose.position.z = wrist.z
    return pose

def hand_pose_callback(msg, hand_name):
    hand_poses[hand_name] = msg

def get_object_pose_tf(object_name):
    try:
        transform = tf_buffer.lookup_transform("world", object_name, rospy.Time(0), rospy.Duration(3.0))
        pose = Pose()
        pose.position.x = transform.transform.translation.x
        pose.position.y = transform.transform.translation.y
        pose.position.z = transform.transform.translation.z
        pose.orientation = transform.transform.rotation
        return pose
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logerr(f"Could not get TF transform for {object_name}")
        return None
    
def calculate_euclidean_distance(pose1, pose2):
    # Calculate Euclidean distance between two poses, rounded to 5 decimals. 
    distance = np.sqrt(
        (abs(pose1.position.x) - abs(pose2.position.x)) ** 2 +
        (abs(pose1.position.y) - abs(pose2.position.y)) ** 2 +
        (abs(pose1.position.z) - abs(pose2.position.z)) ** 2
    )
    return round(distance, 2)

def compute_risk_score(distance, velocity, time_to_stop, d_reach, alpha):
    """
    Compute distance-based hazard (risk) score based on ISO/TS 15066.
    
    Parameters:
        distance (float): Current distance between human and robot (d_H)
        velocity (float): Robot's velocity (m/s)
        time_to_stop (float): Time required to stop robot (s)
        d_reach (float): Robot's maximum physical reach (m)
        alpha (float): Sensitivity coefficient for exponential decay

    Returns:
        float: Risk score (0 to 1)
    """
    d_min = velocity * time_to_stop # Minimum safety distance in meters
    print(f"Distance: {distance:.2f} | d_min: {d_min:.2f} | d_reach: {d_reach:.2f}")
    d_H = distance
    
    if d_H >= d_reach:
        risk = 0.0
    elif d_H <= d_min:
        risk = 1.0
    else:  # d_min < d_H < d_reach
        risk = np.exp(-alpha * (d_H - d_min))
    
    return round(risk, 2)

def assess_hand_risk(hand_name, robot_pos, hand_pose, velocity, time_to_stop, d_reach, alpha):
    y_diff = abs(hand_pose.position.y - robot_pos.position.y)
    threshold = 0.35

    if hand_pose.position.y > robot_pos.position.y + threshold:
        rel_pos = "left"
    elif hand_pose.position.y < robot_pos.position.y - threshold:
        rel_pos = "right"
    else:
        rel_pos = "center"

    dist_to_robot = calculate_euclidean_distance(hand_pose, robot_pos)
    risk_score = compute_risk_score(dist_to_robot, velocity, time_to_stop, d_reach, alpha)

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
        "status": collision_status
    }

def probabilistic_or(risks):
    """
    Compute probabilistic OR of a list of risk values.
    R = 1 - product(1 - Ri)
    """
    product = 1.0
    for r in risks:
        product *= (1 - r)
    return 1 - product

def compute_region_risks(graph):
    """
    For each region node (left, center, right), combine the risks of all hands connected
    to that region using probabilistic OR.
    
    Returns a dict: {region: risk_value}
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
    - Return: robot_risk (float), critical_region (str), critical_hand (str), critical_hand_risk (float)
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
    robot_risk = weighted_risks[critical_region] / total_weight  # Or just max, depending on semantics

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
    #Remove a hand from the graph and ensure its full removal.
    if hand_name in hand_graph.nodes:
        # Log before removal
        print(f"Removing {hand_name} from graph")
        
        # Remove edges connected to the hand node
        neighbors = list(hand_graph.neighbors(hand_name))
        for neighbor in neighbors:
            hand_graph.remove_edge(neighbor, hand_name)
        # Remove the node itself
        hand_graph.remove_node(hand_name)
        
        # Log after removal
        print(f"Removed {hand_name} from graph")
    else:
        print(f"{hand_name} not found in graph")

def add_hand_to_graph(hand_name, hand_pose, rel_pos, dist_to_robot, risk_score, collision_status):
    # Add or update a hand in the graph. 
    robot_pos = get_object_pose_tf(robot)
    if robot_pos:
        # Remove the hand if it already exists
        if hand_name in hand_graph.nodes:
            neighbors = list(hand_graph.neighbors(hand_name))
            for neighbor in neighbors:
                hand_graph.remove_edge(neighbor, hand_name)
            hand_graph.remove_node(hand_name)
            print(f"Removed existing {hand_name} before adding new one.")
            #time.sleep(0.5)  # Ensure removal is complete
        
        # Add new hand node
        hand_graph.add_node(hand_name, pos=(hand_pose.position.x, hand_pose.position.y))
        hand_graph.add_edge(
            rel_pos,
            hand_name,
            distance=dist_to_robot,
            risk_value=risk_score,
            risk_level=collision_status
        )

        #hand_graph.add_edge(rel_pos, hand_name, distance=dist_to_robot)
        print(f"Added {hand_name} under {rel_pos} with distance {dist_to_robot:.2f}")
    else:
        print(f"Robot not found for {hand_name}")


# Graph display function
def display_graph_opencv(graph):
    """ Display the hand graph using OpenCV and Matplotlib.
    This function uses a fixed layout for the robot and hand nodes.
    """
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
        ax=ax
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
        font_color='black',
        label_pos=0.5,
        rotate=False,
        bbox=dict(facecolor='white', edgecolor='none', boxstyle='round,pad=0.2')
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
                ax.text(x, y + 0.15 + i * 0.1, line, fontsize=8, ha='center', color='darkred')


    # Convert to OpenCV image
    canvas.draw()
    buf = np.asarray(canvas.buffer_rgba())
    image_bgr = cv2.cvtColor(buf, cv2.COLOR_RGBA2BGR)

    # Show image
    cv2.imshow("Hand Graph", image_bgr)
    cv2.waitKey(1)

def camera_callback(msg):

    global mediapipe_to_world_map
    current_frame_map = {}
    used_world_hands = set()

    try:
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
        image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Process with MediaPipe
        results = hands.process(image_rgb)

        for hand in hand_objects:
            if hand in hand_poses:
                p = hand_poses[hand]
                print(f"{hand} (world): x={p.position.x:.2f}, y={p.position.y:.2f}, z={p.position.z:.2f}")
            else:
                print(f"{hand} pose not received yet")

        robot_pos = get_object_pose_tf(robot)
        if robot_pos:
            print(f"Robot position: x={robot_pos.position.x:.2f}, y={robot_pos.position.y:.2f}, z={robot_pos.position.z:.2f}")
        else:
            print("Robot not found")

        h, w, _ = frame.shape

        if results.multi_hand_landmarks:

            for i, hand_landmarks in enumerate(results.multi_hand_landmarks):
                    # Draw landmarks on frame
                    mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                    for lm in hand_landmarks.landmark:
                        cx, cy = int(lm.x * w), int(lm.y * h)
                        cv2.circle(frame, (cx, cy), 3, (0, 255, 0), -1)

                    # Estimate 3D wrist pose approx
                    mp_pose = get_wrist_pose_from_landmarks(hand_landmarks)

                    # Filter available hands (not used and inside bounds)
                    available_hands = {
                        name: pose for name, pose in hand_poses.items()
                        if name not in used_world_hands and
                        hand_bounds["x"][0] <= pose.position.x <= hand_bounds["x"][1] and
                        hand_bounds["y"][0] <= pose.position.y <= hand_bounds["y"][1] and
                        hand_bounds["z"][0] <= pose.position.z <= hand_bounds["z"][1]
                    }

                    matched_hand = match_to_closest_hand(mp_pose, available_hands)
                    if matched_hand:
                        current_frame_map[i] = matched_hand
                        used_world_hands.add(matched_hand)

                        if robot_pos:
                      
                            result = assess_hand_risk(matched_hand, robot_pos, hand_poses[matched_hand], velocity, time_to_stop, d_reach, alpha)

                            print(f"[{matched_hand}] matched to MediaPipe hand {i} | Y diff: {result['y_diff']:.2f} | Rel pos: {result['rel_pos']}")
                            print(f"[{matched_hand}] matched to MediaPipe hand {i} | Dist to robot: {result['dist']:.2f}")
                            print(f"[{matched_hand}] Risk score: {result['risk']:.3f}")
                            print(f"[{matched_hand}] ➤ RISK LEVEL = {result['status'].upper()}")

                            add_hand_to_graph(matched_hand, hand_poses[matched_hand], result['rel_pos'], result['dist'], result['risk'], result['status'])

                            # Publish hand status
                            hand_status = HandStatus()
                            hand_status.hand_name = matched_hand
                            hand_status.detected = True
                            hand_status.distance_to_robot = result['dist']
                            hand_status.relative_position = result['rel_pos']
                            hand_status.y_diff = result['y_diff']
                            hand_status.risk_level = result['status']
                            hand_status_pub.publish(hand_status)
                    else:
                        print(f"MediaPipe hand {i} could not be matched (out of bounds or no pose).")

            # Update global mapping
            mediapipe_to_world_map = current_frame_map
        else:
             # No hands detected
            mediapipe_to_world_map = {}
            cv2.putText(frame, "No hands detected", (50, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
        # Remove hands not detected this frame from graph
        for hand_name in list(hand_graph.nodes):
            if hand_name not in used_world_hands and hand_name not in ["robot", "left", "right", "center"]:
                remove_hand_from_graph(hand_name)

        # Compute region risks and update graph nodes
        region_risks = compute_region_risks(hand_graph)
        for region, risk in region_risks.items():
            if region in hand_graph.nodes:
                hand_graph.nodes[region]['or_risk'] = round(risk, 3)

        robot_risk, critical_region, critical_hand, critical_hand_risk = compute_robot_risk(region_risks, node_thresholds, hand_graph)

        # Update robot node
        if "robot" in hand_graph.nodes:
            hand_graph.nodes["robot"]["weighted_risk"] = round(robot_risk, 3)
            hand_graph.nodes["robot"]["critical_region"] = critical_region
            hand_graph.nodes["robot"]["critical_hand"] = critical_hand
            hand_graph.nodes["robot"]["critical_hand_risk"] = round(critical_hand_risk, 3)

        # Print final report
        print(f"\nRobot overall risk: {robot_risk:.2f}")
        print(f"Critical region: {critical_region}")
        print(f"Critical hand: {critical_hand} | Raw risk: {critical_hand_risk:.2f}\n")

        # Display graph and camera feed
        display_graph_opencv(hand_graph)

        if critical_region and critical_hand:
            cv2.putText(frame, f"Critical Region: {critical_region}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (102, 51, 0), 2)
            cv2.putText(frame, f"Critical Hand: {critical_hand} | Risk: {critical_hand_risk:.2f}", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (102, 51, 0), 2)
        
        cv2.imshow("Camera Feed", frame)

        # Exit on 'q' press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rospy.signal_shutdown("User requested shutdown")

    except Exception as e:
        rospy.logwarn(f"Error in camera callback: {e}")

def main():
    global tf_buffer, listener
    rospy.init_node("hand_pose_listener", anonymous=True)

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    rospy.Subscriber("/rgb", Image, camera_callback)

    for hand in hand_objects:
        rospy.Subscriber(f"/{hand.lower()}/pose", Pose, lambda msg, h=hand: hand_pose_callback(msg, h))

    rospy.loginfo("Hand tracking node running...")
    rospy.spin()

if __name__ == "__main__":
    main()
