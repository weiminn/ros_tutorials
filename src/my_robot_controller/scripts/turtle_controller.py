#!/usr/bin/env python3
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen

previous_x = 0
previous_y = 0

# service client
def call_set_pen_service(r, g, b, width, off):
    try:
        set_pen = rospy.ServiceProxy(
            "/turtle1/set_pen",
            SetPen
            )
        response = set_pen(r, g, b, width, off)
    except rospy.ServiceException as e:
        rospy.logwarn(e)

# callback function to process the message promise
def pose_cb(msg: Pose):
    cmd = Twist()
    if msg.x > 9.0 or msg.x < 2.0 or msg.y > 9 or msg.y < 2:
        cmd.linear.x = 1.0
        cmd.angular.z = 1.4
    else:
        cmd.linear.x = 5.0
        cmd.angular.z = 0

    global previous_x
    global previous_y
    if (msg.x >= 5.5 and previous_x < 5.5):
        previous_x = msg.x
        previous_y = msg.y
        call_set_pen_service(255, 0, 0, 3, 0)

    if (msg.x < 5.5 and previous_x >= 5.5):
        previous_x = msg.x
        previous_y = msg.y        
        call_set_pen_service(0, 255, 0, 3, 0)

    pub.publish(cmd)

if __name__ == "__main__":
    rospy.init_node("turtle_controller")

    rospy.wait_for_service("/turtle1/set_pen")

    pub = rospy.Publisher(
        '/turtle1/cmd_vel', 
        Twist,
        queue_size=10
        )
    
    sub = rospy.Subscriber(
        '/turtle1/pose', 
        Pose,
        callback=pose_cb
        )
    
    rospy.loginfo("Subscriber has been initiated")
    rospy.spin() # infinite loop to block till the node has exit

