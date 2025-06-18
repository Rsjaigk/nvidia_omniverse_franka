import rospy
from geometry_msgs.msg import Pose

hand_objects = ["hand1", "hand2", "hand3"]

def hand_pose_callback(msg, hand_name):
    rospy.loginfo(f"Received pose for {hand_name}: {msg}")

def main():
    rospy.init_node("hand_pose_listener", anonymous=True)
    for hand in hand_objects:
        rospy.Subscriber(f"/{hand}/pose", Pose, lambda msg, h=hand: hand_pose_callback(msg, h))
    rospy.spin()

if __name__ == "__main__":
    main()
