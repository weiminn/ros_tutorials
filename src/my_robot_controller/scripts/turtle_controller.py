#!/usr/bin/env python3
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

# callback function to process the message promise
def pose_cb(msg: Pose):
    cmd = Twist()
    if msg.x > 9.0 or msg.x < 2.0 or msg.y > 9 or msg.y < 2:
        cmd.linear.x = 1.0
        cmd.angular.z = 1.4
    else:
        cmd.linear.x = 5.0
        cmd.angular.z = 0
    pub.publish(cmd)

if __name__ == "__main__":
    rospy.init_node("turtle_controller")

    pub = rospy.Publisher(
        '/turtle1/cmd_vel', 
        Twist,
        queue_size=10)
    
    sub = rospy.Subscriber(
        '/turtle1/pose', 
        Pose,
        callback=pose_cb
        )
    
    rospy.loginfo("Subscriber has been initiated")
    rospy.spin() # infinite loop to block till the node has exit

