import rospy
import tf2_ros
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image

hand_objects = ["hand1", "hand2", "hand3"]
robot = "panda_link8"

# Store latest pose messages from topics
hand_poses = {}

# OpenCV Bridge
bridge = CvBridge()

# TF Buffer and Listener (to be initialized after init_node)
tf_buffer = None
listener = None

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

def camera_callback(msg):
    try:
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
        image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

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

    except Exception as e:
        rospy.logwarn(f"Error in camera callback: {e}")

def main():
    global tf_buffer, listener
    rospy.init_node("hand_pose_listener", anonymous=True)

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    rospy.Subscriber("/rgb", Image, camera_callback)

    for hand in hand_objects:
        rospy.Subscriber(f"/{hand}/pose", Pose, lambda msg, h=hand: hand_pose_callback(msg, h))

    rospy.loginfo("Hand tracking node running...")
    rospy.spin()

if __name__ == "__main__":
    main()
