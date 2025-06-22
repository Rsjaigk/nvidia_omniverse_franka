import rospy
import tf2_ros
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image
import mediapipe as mp
import numpy as np
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
