# ROS Tutorials

## Setup on Ubuntu 20.04 using following links

For setting up ROS Noetic:
http://wiki.ros.org/noetic/Installation/Ubuntu

For setting up Ardupilot:
https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/installing_ros_20_04.md


## Graph View

ROS Master:

```
$roscore
```

ROS Node:
```
$rosrun rospy_tutorials talker
```

ROS Another Node:

```
$rosrun rospy_tutorials listener
```

ROS Graph View:

```
$rqt_graph
```

## Turtle Sim with Manual Control Command

```
$roscore
$rosrun turtlesim turtlesim_node 
$rosrun turtlesim turtle_teleop_key
```

## Create ROS Package

Create Catkin Workspace
```
$mkdir -p catkin_ws/src
$ls catkin_ws
$catkin_make # create build develop and source spaces
$source catkin_ws/devel/setup.bash
$echo "source ~/Document/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

Create ROS Package
```
# important for package creator to link to the newly created package
$source ~/.bashrc

$cd ~/Documents/catkin_ws/src
$catkin_create_pkg my_robot_controller rospy roscpp turtle_sim
$code . # open in vs code
$cd .. && catkin clean && catkin_make
```

Create ROS Node
```
$cd ~/Documents/catkin_ws/src/my_robot_controller
$mkdir scripts && cd scripts
$sudo nano my_first_node.py # with #!/usr/bin/env python3 at the top!
$chmod +x my_first_node.py # important for ROS to run the script
$rosrun my_robot_controller my_first_node.py 

```

## ROS Node Admin commands

| Description | Command | Note |
|-------------------|------|------|
| List Nodes | `$rosnode list`   | `/rosout` will be a staple  |
| Kill node | `rosnode kill /node_to_kill`| 
| List topics| `rostopic list` | Nodes don't talk to each other; they just publish to and listen to topics|
|Get topic info | `rostopic info /chatter` |
|Inspect ROS Message format | `rosmsg show turtlesim/pose` |
|Livestream of ROS topic messages | `rostopic echo /turtle1/pose`|
| Frequencies of Topics | `rostopic hz /turtle1/pose`|

Get Message format to publish to turtle sim:
```
$rostopic list # to get topic that the listener is subscribed to
$rostopic info /cmd_vel # to get the data type of the topic
$rosmsg show geometry_msg/Twist # to inspect the message format
```

## Create ROS Publisher

Publisher Python Script:
```
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

if __name__ == "__main__":
    rospy.init_node("draw_circle")
    rospy.loginfo("Drawer has been initiated")

    pub = rospy.Publisher(
        '/turtle1/cmd_vel', 
        Twist, # need to import
        queue_size=10)
    
    rate = rospy.Rate(2)

    while not rospy.is_shutdown():
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 1.0
        pub.publish(msg)
        rate.sleep()

# don't for get to chmod +x this file

# Add dependency into `package.xml` for `geometry_msgs` for `<build_depend>`, `<build_export_depend>` and `<exec_depend>` tags.

```

## Create ROS Subscriber

Get Message format of the topic to subscribe to:
```
$rostopic list # to get topic that the listener is subscribed to
$rostopic info /turtlesim/pose # to get the data type of the topic
$rosmsg show turtlesim/Pose # case sensitive!!!
```

Create Subscriber node:
```
#!/usr/bin/env python3
import rospy
from turtlesim.msg import Pose

# callback function to process the message promise
def pose_cb(msg: Pose): # define data type for better pylance
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
```

## Closed Loop ROS Controller 
```
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
```

## Client Server Interaction on ROS

ROS Commands for service:

| Description | Command | Note |
|-------------------|------|------|
| Launch an ROS service|`$rosrun rospy_tutorials add_two_ints_server`|
| List ROS services | `$rosservice list` |
| Get arguments of a service | `$rosservice info /add_two_ints`|
| Make client's request a service |`$rosservice call /add_two_ints "a: 0 b: 0" ` | Press tab twice to autofill the arguments|
| Get Message Formats and Data Types of a service |`$rossrv show /add_two_ints` |

Demo with Turtle Sim's services:
```
$rosrun rospy_tutorials turtlesim
```

## Client Server Interaction on ROS

Put the following line right after the initialization of the node to block the script and wait for the service to come online:
```
rospy.wait_for_service("/turtle1/set_pen")
```

Service Client code:
```
def call_set_pen_service(r, g, b, width, off):
    try:
        set_pen = rospy.ServiceProxy(
            "/turtle1/set_pen",
            SetPen
            )
        response = set_pen(r, g, b, width, off)
    except rospy.ServiceException as e:
        rospy.logwarn(e)
```

Only call the service at specific condition:
```
    # just before publish call
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

    # publish code
```

