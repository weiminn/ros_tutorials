#!/usr/bin/env python3
import rospy
from turtlesim.msg import Pose

# callback function to process the message promise
def pose_cb(msg: Pose):
    rospy.loginfo(
        str(msg.x) + ', ' +
        str(msg.y)
        )

if __name__ == "__main__":
    rospy.init_node("turtle_pose_subscriber")

    pub = rospy.Subscriber(
        '/turtle1/pose', 
        Pose,
        callback=pose_cb
        )
    
    rospy.loginfo("Subscriber has been initiated")

    rospy.spin() # infinite loop to block till the node has exit