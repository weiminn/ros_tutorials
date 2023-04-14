# [ROS Tutorials in Python](https://www.youtube.com/watch?v=wfDJAYTMTdk)

## Setup on Ubuntu 20.04 using following links

* [For setting up ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)

* [For setting up Ardupilot](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/installing_ros_20_04.md)


## Graph View

ROS Master:

```
$ roscore
```

ROS Node:
```
$ rosrun rospy_tutorials talker
```

ROS Another Node:

```
$ rosrun rospy_tutorials listener
```

ROS Graph View:

```
$ rqt_graph
```

## Turtle Sim with Manual Control Command

```
$ roscore
$ rosrun turtlesim turtlesim_node 
$ rosrun turtlesim turtle_teleop_key
```

## Create ROS Package

Create Catkin Workspace
```
$ mkdir -p catkin_ws/src
$ cd catkin_ws
$ catkin init
$ catkin_make # create build develop and source spaces
$ source catkin_ws/devel/setup.bash
$ echo "source ~/Document/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

Create ROS Package
```
$ cd ~/Documents/catkin_ws/src
$ catkin_create_pkg my_robot_controller rospy roscpp turtle_sim
$ cd .. && catkin clean && catkin_make

# important for package creator to link to the newly created package
$ source ~/.bashrc
```

Create ROS Node
```
$ cd ~/Documents/catkin_ws/src/my_robot_controller
$ mkdir scripts && cd scripts
$ sudo nano my_first_node.py # with #!/usr/bin/env python3 at the top!
$ chmod +x my_first_node.py # important for ROS to run the script
$ rosrun my_robot_controller my_first_node.py 

```

## ROS Node Admin commands

| Description | Command | Note |
|-------------------|------|------|
| List Nodes | `$ rosnode list`   | `/rosout` will be a staple  |
| Kill node | `$ rosnode kill /node_to_kill`| 
| List topics| `$ rostopic list` | Nodes don't talk to each other; they just publish to and listen to topics|
|Get topic info | `$ rostopic info /chatter` |
|Inspect ROS Message format | `$ rosmsg show turtlesim/pose` |
|Livestream of ROS topic messages | `$ rostopic echo /turtle1/pose`|
| Frequencies of Topics | `$ rostopic hz /turtle1/pose`|

## Create ROS Publisher

Get Message format to publish to turtle sim:
```
$ rostopic list # to get topic that the listener is subscribed to
$ rostopic info /cmd_vel # to get the data type of the topic
$ rosmsg show geometry_msg/Twist # to inspect the message format
```
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
$ rostopic list # to get topic that the listener is subscribed to
$ rostopic info /turtlesim/pose # to get the data type of the topic
$ rosmsg show turtlesim/Pose # case sensitive!!!
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
| Launch an ROS service|`$ rosrun rospy_tutorials add_two_ints_server`|
| List ROS services | `$ rosservice list` |
| Get arguments of a service | `$ rosservice info /add_two_ints`|
| Make client's request a service |`$ rosservice call /add_two_ints "a: 0 b: 0" ` | Press tab twice to autofill the arguments|
| Get Message Formats and Data Types of a service |`$ rossrv show /add_two_ints` |

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

# [ROS Course (ETH Zurich)](https://www.youtube.com/playlist?list=PLE-BQwvVGf8HOvwXPgtDfWoxd4Cc6ghiP)

Clone [this repository](https://github.com/leggedrobotics/ros_best_practices) into your catkin workspace.

> Can clone this directly into `catkin/src` or just separate directory and symlink it.

Build the package:
```
$ catkin build ros_package_template
```
Resource the workspace setup:
```
$ source devel/setup.bash
```
Launch the node:
```
$ roslaunch ros_package_template ros_package_template.launch
```
> `launch` is a tool (in xml format) for launching multiple nodes (along with `roscore` if it is not already running) as well as setting parameters.

## ROS Packages
`package.xml` file defines the properties of the package
* Package name
* Version number
* Authors
* Dependencies on other packages
* Etc...

```
<?xml version="1.0"?>
<package format="2">
  <name>ros_package_template</name>
  <version>0.1.0</version>
  <description>A template for ROS packages.</description>
  <maintainer email="pfankhauser@anybotics.com">Peter Fankhauser</maintainer>
  <license>BSD</license>
  <url type="website">https://github.com/ethz-asl/ros_best_practices</url>
  <url type="bugtracker">https://github.com/ethz-asl/ros_best_practices/issues</url>
  <author email="pfankhauser@anybotics.com">Peter Fankhauser</author>

  <!-- buildtool_depend: dependencies of the build process -->
  <buildtool_depend>catkin</buildtool_depend>
  <!-- build_depend: dependencies only used in source files -->
  <build_depend>boost</build_depend>
  <!-- depend: build, export, and execution dependency -->
  <depend>eigen</depend>
  <depend>roscpp</depend>
  <depend>sensor_msgs</depend>

  <build_depend>roslint</build_depend>
</package>

```

## CMakeLists.xml

For configuring the CMake build system

|Description|Field|Notes|
|-----|-----|----|
| `cmake_minimum_required`| Required CMake version 
| `project` | Package Name | Usually the same value as `<name>` in `package.xml`
| `find_package`| Find other CMake/Catkin packages needed for build
| `add_message_files`, `add_service_files`, `add_action_files` | Message/Service/Action generators|
| `generate_messages` | Invoke message/service/action generation|
| `catkin_package` | Package build export information |`INCLUDE_DIRS`, `LIBRARIES`, `CATKIN_DEPENDS`, `DEPENDS`
| `add_library`, `add_executables`, `target_link_libraries` | Libraries/Executables to build|
| `include_directories` | Location of Header files|
| `catkin_add_gtest` | Tests to build|
| `install` | Install rules|

## Node Handles

4 main types of Node Handles

|Type|Code|Topic Lookup|Notes|
|-----|-----|----|----|
| Default| `nh_ = ros::NodeHandle();`|`/namespace/topic`| Recommended 
| Private| `nh_private_ = ros::NodeHandle("~");`|`/namespace/node/topic/`| Recommended 
| Namespaced| `nh_eth_ = ros::NodeHandle("eth")`|`/namespace/eth/topic`|  
| Global| `nh_global_ = ros::NodeHandle("/")`|`/topic`| **Not** Recommended

> In C++, you need `NodeHandle` to get instances of Publishers and Subscribers whereas in Python, you can call `rospy` library directly.

## Parameter Server

Nodes can use *parameter server* to store and retrieve configuration parameters at runtime. The parameters are defined in separate YAML files, and configured in `package.launch` file.

|Description|Field|C++ Code|
|-----|-----|-----|
| List all parameters| `$ rosparam list` 
| Get value of parameter | `$ rosparam get parameter_name` | `nodeHandle.getParam(param_name, variable)`|
| Set parameter| `$ rosparam set parameter_name value`

> For parameters, typically use private node handler `ros::NodeHandle("~")`.

## [Transformation System](http://wiki.ros.org/tf)

Keeps track of coordinate frames over time, and lets users transform points, vectors, etc. It is implemented as publisher-subscriber model on topics `/tf` and `tf/static`.

|Tool|Command|
|-----|-----|
| Create visual Graph of the transform tree| `$ rosrun tf view_frames` <br/> `$ evince frames.pdf`|
| Print information about the current transform tree| `$ rosrun tf tf_monitor` |
| Print information about the transform between 2 frames | `$ rosrun tf tf_echo source_frame target_frame` <br/> ``$ rosrun rviz rviz -d `rospack find turtle_tf`/rviz/turtle_rviz.rviz`` |

# ROS Exercises

## [Simulating Husky](http://wiki.ros.org/husky_gazebo/Tutorials/Simulating%20Husky)

Install Simulation Package:
```
$ sudo apt install ros-noetic-husky-simulator
```

Set up environmental variable HUSKY_GAZEBO_DESCRIPTION:
```
$ export HUSKY_GAZEBO_DESCRIPTION=$(rospack find husky_gazebo)/urdf/description.gazebo.xacro
```

Launch simulation:
```
# Empty world
$ roslaunch husky_gazebo empty_world.launch

# Clearpath design world
$ roslaunch husky_gazebo husky_playpen.launch
```

Inspect ROS Nodes and their topics:
```
$ rosnode list
$ rostopic list
$ rostopic echo [TOPIC]
$ rostopic hz [TOPIC]
$ rqt_graph
```

Download and run `teleop_twist_keyboard` to control the robot using your keyboard:
```
$ sudo apt-get install ros-noetic-teleop-twist-keyboard
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

Change the launch file to load a different world:
```
# Replace this 
<arg name="world_name" default="worlds/empty.world"/>

# With this new line
<arg name="world_name" default="worlds/robocup14_spl_field.world"/>

```

## Creating Package from Scratch

Create new package with dependencies, and then inspect the `package.xml` and `CMakelists.txt`:
```
$ cd ~/catkin_ws/src # important to create inside src!
$ catkin_create_pkg eth_exercise roscpp sensor_msgs nav_msgs geometry_msgs tf2_geometry_msgs tf std_srvs
$ catkin build
$ source ~/.bashrc # to link the command line to newly compiled packages
```

> You can clone and build `ros_best_practices` from [the first part of the ROS course](#ros-course-eth-zurich), and use it as a reference.

Tell CMake to find other packages necessary for build:
```
## Recomended
find_package(catkin REQUIRED COMPONENTS 
  ## import below packages as components of catkin
  roscpp
  sensor_msgs
)

## Alternative (but not recommended):
find_package(catkin REQUIRED) # minimum this one at least
find_package(roscpp REQUIRED)
find_package(sensor_msgs REQUIRED )

```
> For catkin packages, if you find_package them as components of catkin, this is advantageous as a single set of environment variables is created with the catkin_ prefix. For example, let us say you were using the package nodelet in your code.

Add/Uncomment the following chunk in `CMakelists.txt` to register into dependency tree:
```
catkin_package(
 INCLUDE_DIRS include # for header files of your custom libraries
 CATKIN_DEPENDS roscpp sensor_msgs
)
```
> `CATKIN_DEPENDS` tells the other pacakges that `find_package` our package on which dependencies are being passed along.

> This function must be called before declaring any targets with `add_library`, or `add_executable`.

Locations of the header files of your custom libraries:
```
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
```

Add/Uncomment the following chunk in `CMakelists.txt` to register your custom libraries for OOP and Modular Purposes:
```
## Declare a C++ library
add_library(
  ${PROJECT_NAME}_core # name of outputted target library folder
  src/${PROJECT_NAME}.cpp
)
```

Add/Uncomment the following chunk in `CMakelists.txt` to register the main executable:
```
## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(
  ${PROJECT_NAME} # name of outputted target executable folder
  src/${PROJECT_NAME}_node.cpp
)
```

Add the following chunk in `CMakelists.txt` to link the executables and libraries:
```
target_link_libraries(${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
)
```

## Transformations

### Static Transform Broadcaster

Create package:
```
$ cd ~/catkin_ws/src
$ catkin_create_pkg learning_tf2 tf2 tf2_ros roscpp rospy turtlesim
$ roscd learning_tf2
```

Broadcaster code:
```
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>


std::string static_turtle_name;

int main(int argc, char **argv)
{
  ros::init(argc,argv, "my_static_tf2_broadcaster");
  if(argc != 8)
  {
    ROS_ERROR("Invalid number of parameters\nusage: static_turtle_tf2_broadcaster child_frame_name x y z roll pitch yaw");
    return -1;
  }
  if(strcmp(argv[1],"world")==0)
  {
    ROS_ERROR("Your static turtle name cannot be 'world'");
    return -1;

  }
  static_turtle_name = argv[1];
  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped static_transformStamped;

  static_transformStamped.header.stamp = ros::Time::now();
  static_transformStamped.header.frame_id = "world";
  static_transformStamped.child_frame_id = static_turtle_name;
  static_transformStamped.transform.translation.x = atof(argv[2]);
  static_transformStamped.transform.translation.y = atof(argv[3]);
  static_transformStamped.transform.translation.z = atof(argv[4]);
  tf2::Quaternion quat;
  quat.setRPY(atof(argv[5]), atof(argv[6]), atof(argv[7]));
  static_transformStamped.transform.rotation.x = quat.x();
  static_transformStamped.transform.rotation.y = quat.y();
  static_transformStamped.transform.rotation.z = quat.z();
  static_transformStamped.transform.rotation.w = quat.w();
  static_broadcaster.sendTransform(static_transformStamped);
  ROS_INFO("Spinning until killed publishing %s to world", static_turtle_name.c_str());
  ros::spin();
  return 0;
};
```

Modify `CMakeLists.txt` to export and link executables:
```
add_executable(static_turtle_tf2_broadcaster src/static_turtle_tf2_broadcaster.cpp)
target_link_libraries(static_turtle_tf2_broadcaster  ${catkin_LIBRARIES} )
```

Compile and run the static broadcaster:
```
$ catkin build
$ source ~/.bashrc
$ roscore

# Seperate terminal tab
$ source ~/.bashrc
$ rosrun learning_tf2 static_turtle_tf2_broadcaster mystaticturtle 0 0 1 0 0 0
```