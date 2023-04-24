# [ROS Tutorials in Python](https://www.youtube.com/watch?v=wfDJAYTMTdk)

## Setup on Ubuntu 20.04 using following links

* [For setting up ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)


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
> Dependencies (`buildtool_depend`, `build_depend` and `depend`) are pretty much the only things that matter in `package.xml`.

## CMakeLists.xml

For configuring the CMake build system

|Description|Field|Notes|
|-----|-----|----|
| `cmake_minimum_required`| Required CMake version 
| `project` | Package Name | Usually the same value as `<name>` in `package.xml`
| `find_package`| Find other CMake/Catkin packages needed for build | The convention is to `find_package` all other packages as `COMPONENTS` of `catkin REQUIRED` package.
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

# [ROS Official Tutorials](http://wiki.ros.org/ROS/Tutorials)

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

## [Creating Package from Scratch](http://wiki.ros.org/ROS/Tutorials/CreatingPackage)

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
> For catkin packages, if you `find_package` them as components of catkin, this is advantageous as a single set of environment variables is created with the `catkin_` prefix. 

Add/Uncomment the following chunk in `CMakelists.txt` to register into dependency tree:
```
catkin_package(
 INCLUDE_DIRS include # for header files of your custom libraries
 CATKIN_DEPENDS roscpp sensor_msgs
)
```
> `CATKIN_DEPENDS` tells the other packages that `find_package` our package on which dependencies are being passed along.

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

## [Services and Parameters](http://wiki.ros.org/ROS/Tutorials/UnderstandingServicesParams)

### Services

|Command|Description|Notes|
|---|---|---|
|`$ rosservice list`| Print information about active services |
|`$ rosservice call [service] [args]`| Call the service with the provided args |
|`$ rosservice type [service]`| Print service type | `std_srvs/Empty` type services take no argument and also return nothing <br> Use `$ rosservice type [service] \| grep rossrv show` to view the structure of the message if available|
|`$ rosservice find`| Find services by service type |
|`$ rosservice uri`| Print service ROSRPC uri |

### Parameters

A parameter server is a shared, multi-variate dictionary that is accessible via network APIs. Nodes use this server to store and retrieve parameters at runtime. As it is not designed for high-performance, it is best used for static, non-binary data such as configuration parameters. It is meant to be globally viewable so that tools can easily inspect the configuration state of the system and modify if necessary.

|Command|Description|Notes|
|---|---|---|
|`$ rosparam list`| List parameter names | Parameters are named using the normal ROS naming convention. This means that ROS parameters have a hierarchy that matches the namespaces used for topics and nodes. This hierarchy is meant to protect parameter names from colliding.
|`$ rosparam get [param_name]`| Get parameter value | `$ rosparam get /` shows contents of the entire Parameter server. <br> Multiple parameter values are returned as a dictionary.
|`$ rosparam set [param_name] [value]`| Set parameter to value|
|`$ rosparam dump [file]`| Save paramters into file | The parameters are usually stored in `.yaml` files.|
|`$ rosparam load [file] [namespace]`| Load param files into namespace | 
|`$ rosparam delete`| Delete parameter |

## [Custom Message and Services formats](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv)

### Creating a `msg` message format

Navigate into a ROS package and create `msg` directory, and create `.msg` file in there:
```
$ cd ros_pacakge
$ mkdir msg
$ echo "int64 num" > msg/Num.msg
```
Insert tags for compiling message formats in `package.xml`:
```
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```

Insert dependencies for `msg` compilations in `CMakeLists.txt`, by modifying `find_package` and `catkin_package` respectively:
```
# inside find_package
std_msgs
message_generation

# inside catkin_package
catkin_package(
  ...
  CATKIN_DEPENDS message_runtime ...
  ...)
```

Insert the command to compile our custom `msg` files in `CMakeLists.txt`:
```
add_message_files(
  FILES
  Num.msg
)
```

Uncomment `generate_messages` macro in `CMakeLists.txt`:
```
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)
```

Build the package using `catkin build` and re-source the environments, and you should be able to see the newly created `beginner_tutorials/Num` message inside `roscore`.

### Creating custom `srv` message format

Create `srv` folder inside a catkin package, and copy a sample service over from `rospy_tutorials` package:
```
$ cd catkin_ws/ros_package
$ mkdir srv
$ roscp rospy_tutorials AddTwoInts.srv srv/AddTwoInts.srv
```

Add same dependencies as with `msg` into `package.xml` and `CMakeLists.txt`.

Uncomment following line inside `CMakeLists.txt` to compile the custom service message format files:
```
add_service_files(
  FILES
  AddTwoInts.srv
)
```

Build the package using `catkin build` and re-source the environments, and you should be able to see the newly created `beginner_tutorials/AddTwoInts` service message format inside `roscore`.
> You **cannot** embed another `srv` file instead of an `srv`.


## [Transformations](http://wiki.ros.org/tf/Tutorials)

### [Static Transform Broadcaster](http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20static%20broadcaster%20%28C%2B%2B%29)

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

  // Name of the turtle, arbitrary for now
  // Cos you're not tracking any running turtle instance yet
  static_turtle_name = argv[1];

  // Broadcaster object to send transformations over the wire
  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped static_transformStamped;

  // Content of messages to broadcast
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

### [Tranformation Broadcaster](http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20static%20broadcaster%20%28C%2B%2B%29)

Code for Transformation Broadcaster:

```
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <turtlesim/Pose.h>

std::string turtle_name;

void poseCallback(const turtlesim::PoseConstPtr& msg){

    // Broadcaster object to send transformations over the wire
    static tf2_ros::TransformBroadcaster br;

    // Metadata for message to broadcast
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = turtle_name;
    
    // Content of messages to broadcast
    transformStamped.transform.translation.x = msg->x;
    transformStamped.transform.translation.y = msg->y;
    transformStamped.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, msg->theta);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    br.sendTransform(transformStamped);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "my_tf2_broadcaster");
    ros::NodeHandle private_node("~");

    // get name of the turtle you wanna track
    if(!private_node.hasParam("turtle")){
        if(argc != 2) {
            ROS_ERROR("need turtle name as argument");
            return -1;
        }
        turtle_name = argv[1];
    } else {
        private_node.getParam("turtle", turtle_name);
    }

    // subscribe the pose of the turtle running
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe(turtle_name+"/pose", 10, &poseCallback);

    ros::spin();
    return 0;
}
```

Add executables path and link between compiled bin and libraries to compile in ``CMakelists.txt``, and then run `catkin build` and resource bash:
```
add_executable(turtle_tf2_broadcaster src/turtle_tf2_broadcaster.cpp)

target_link_libraries(turtle_tf2_broadcaster
 ${catkin_LIBRARIES}
)
```

Create `start_demo.launch` with following contents inside:
```
<launch>
    <!-- Turtlesim Node to observe pose -->
    <node pkg="turtlesim" type="turtlesim_node" name="sim"/>

    <!-- Teleop key to control the turtle -->
    <node pkg="turtlesim" type="turtle_teleop_key" name="teleop" output="screen"/>

    <!-- Axes -->
    <param name="scale_linear" value="2" type="double"/>
    <param name="scale_angular" value="2" type="double"/>

    <!-- Nodes that subscribe to the turtles' poses and broadcast their transforms relative to world concurrently-->
    <node pkg="learning_tf2" type="turtle_tf2_broadcaster"
          args="/turtle1" name="turtle1_tf2_broadcaster" />
    <node pkg="learning_tf2" type="turtle_tf2_broadcaster"
          args="/turtle2" name="turtle2_tf2_broadcaster" />

</launch>
```

Launch the environment and observe transformation broadcast of the turtle:
```
$ roslaunch learning_tf2 start_demo.launch
$ rosrun tf tf_echo /world /turtle1
```

### [Transformation Listener](http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20listener%20%28C%2B%2B%29)

Transformation listener for Turtle1 relative to Turtle2:
```
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "my_tf2_listener");

    ros::NodeHandle node;

    ros::service::waitForService("spawn");
    ros::ServiceClient spawner =
        node.serviceClient<turtlesim::Spawn>("spawn");
    turtlesim::Spawn turtle;
    turtle.request.x = 4;
    turtle.request.y = 2;
    turtle.request.theta = 0;
    turtle.request.name = "turtle2";
    spawner.call(turtle);

    ros::Publisher turtle_vel =
        node.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);

    // buffer temporarily stores incoming transforms
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Rate rate(10.0);
    while (node.ok()){
        geometry_msgs::TransformStamped transformStamped;
        try{
            // retrieve transform message from buffer
            transformStamped = tfBuffer.lookupTransform(
                                "turtle2", // source (relative to)
                                "turtle1", // target 
                                ros::Time(0) // 0 for latest transform
                                );
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        // Use the relative transformation of turtle1 to send command to turtle2
        geometry_msgs::Twist vel_msg;
        vel_msg.angular.z = 4.0 * atan2(transformStamped.transform.translation.y,
                                        transformStamped.transform.translation.x);
        vel_msg.linear.x = 0.5 * sqrt(pow(transformStamped.transform.translation.x, 2) +
                                    pow(transformStamped.transform.translation.y, 2));
        turtle_vel.publish(vel_msg);

        rate.sleep();
    }
    return 0;
};
```

Add executables path and link between compiled bin and libraries to compile in ``CMakelists.txt``, and then run `catkin build` and resource bash:
```
add_executable(turtle_tf2_listener src/turtle_tf2_listener.cpp)
target_link_libraries(turtle_tf2_listener
 ${catkin_LIBRARIES}
)
```

Add listener node to `start_demo.launch`:
```
<node pkg="learning_tf2" type="turtle_tf2_listener"
          name="listener" />
```

### [Adding a frame](http://wiki.ros.org/tf2/Tutorials/Adding%20a%20frame%20%28C%2B%2B%29)

Create a new broadcaster node:
```
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "my_tf2_broadcaster");
    ros::NodeHandle node;

    tf2_ros::TransformBroadcaster tfb;
    geometry_msgs::TransformStamped transformStamped;

    // broadcast transform for carrot to be offset of turtle1
    transformStamped.header.frame_id = "turtle1";
    transformStamped.child_frame_id = "carrot1";
    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 2.0;
    transformStamped.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    ros::Rate rate(10.0);
    while (node.ok()){
        transformStamped.header.stamp = ros::Time::now();
        tfb.sendTransform(transformStamped);
        rate.sleep();
        printf("sending\n");
    }

};
```

Add executables path and link between compiled bin and libraries to compile in ``CMakelists.txt``, and then run `catkin build` and resource bash:
```
add_executable(frame_tf2_broadcaster src/frame_tf2_broadcaster.cpp)
target_link_libraries(frame_tf2_broadcaster
 ${catkin_LIBRARIES}
)
```

Add listener node to `start_demo.launch`:
```
<node pkg="learning_tf2" type="frame_tf2_broadcaster"
          name="broadcaster_frame" />
```

Change the behavior of the `turtle2` to follow `carrot1` instead of `turtle1`:
```
transformStamped = tfBuffer.lookupTransform(
                    "turtle2", // source (relative to)
                    // "turtle1", // old target 
                    "carrot1", // new offset target 
                    ros::Time(0) // 0 for latest transform
                    );
```


### [Transfomations and Time Travel](http://wiki.ros.org/tf2/Tutorials/Time%20travel%20with%20tf2%20%28C%2B%2B%29)

Make the 2nd turtle go to the where the first turtle was 3 seconds ago:
```    
ros::Time now = ros::Time::now();
ros::Time past = ros::Time::now() - ros::Duration(3.0);

// turtle2 goes to where turtle1 was 3 seconds ago
transformStamped = tfBuffer.lookupTransform(
    "turtle2", now,
    "turtle1", past, 
    "world", ros::Duration(1.0));
```

The advanced API with overloaded `lookupTransform` with 6 arguments:

<ol>
<li>Get the transformation from this frame</li>
<li>at this time</li>
<li>to this frame</li>
<li>at this time</li>
<li>Specify the frame that does not change over time, usually "/world"</li>
<li>timeout for exception</li>
</ol>

<!-- ### [Sensor Messages with TF2](http://wiki.ros.org/tf2/Tutorials/Using%20stamped%20datatypes%20with%20tf2%3A%3AMessageFilter) -->

# ROS for Drones

## Set up Gazebo plugin and Resources for ArduPilot

Clone Gazebo Ardupilot Plugin (doesn't need to be into your Catkin source dir) and build it:
```
$ cd ~
$ git clone https://github.com/khancyr/ardupilot_gazebo.git
$ cd ardupilot_gazebo
$ mkdir build
$ cd build
$ cmake ..
$ make -j4
$ sudo make install
```

After building check if `libArduPilotPlugin.so` and `libArduCopterIRLockPlugin.so` has been copied from `build` directory to `/usr/lib/x86_64-linux-gnu/gazebo`. If not, copy them in manually via `sudo`.

Clone [Gazebo Models repository from OSRF](https://github.com/osrf/gazebo_models) and add `GAZEBO_MODEL_PATH=/home/weiminn/gazebo_models` to `~/.bashrc` before setting up Gazebo environment paths:
```
$ echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
$ source ~/.bashrc # run this for every open terminal
```
> This command will append new models from the repo to our default models.

## Installing MAVROS and MAVLink from source

Clone MAVROS and MAVLink into your Catkin source dir and build it:
```
$ cd ~/catkin_ws
$ wstool init ~/catkin_ws/src

$ rosinstall_generator --upstream mavros | tee /tmp/mavros.rosinstall
$ rosinstall_generator mavlink | tee -a /tmp/mavros.rosinstall
$ wstool merge -t src /tmp/mavros.rosinstall
$ wstool update -t src
$ rosdep install --from-paths src --ignore-src --rosdistro `echo $ROS_DISTRO` -y

$ catkin build

$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc # if it already doesn't have

$ sudo ~/catkin_ws/src/mavros/mavros/scripts/install_geographiclib_datasets.sh
```

## IQ Simulation ROS Package

```
cd ~/catkin_ws/src
git clone https://github.com/Intelligent-Quads/iq_sim.git
echo "GAZEBO_MODEL_PATH=$HOME/catkin_ws/src/iq_sim/models:${GAZEBO_MODEL_PATH}" >> ~/.bashrc
```
>Make sure that `GAZEBO_MODEL_PATH` variable in `.bashrc` will be appended to have custom simulation models from Intelligent Quads, on top of its default `/usr/share/gazebo/models` and ArduPilot-Gazebo's models

## Run ROS Simulation for Drone

Start Simulation environment and ArduPilot SITL:
```
$ roslaunch iq_sim runway.launch

# in seperate terminal
$ cp ~/catkin_ws/src/iq_sim/scripts/
$ ./startsitl.sh
```
In a separate window, you can use `rostopic list` to see the ROS nodes of Gazebo broadcasting topics related to model states.

Launch MAVROS communication with the Drone:
```
$ roslaunch iq_sim apm.launch
```
You will now be able to see broadcasted ROS topics regarding telemetry from the Flight Controller and inspect them using `rostopic echo` to livestream data, `rostopic info` to check message type and `rosmsg info` to check data structure of the message.

## [ROS Drone Guidance and Navigation](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/gnc_tutorial.md)

`iq_gnc` package contains a library for MAVROS communication. For example, below function initializes the [broadcaster, subscribers and service clients to communicate to their respective modules and the required data type/format](http://wiki.ros.org/mavros) inside FCU:
```
# Declaration
ros::Publisher local_pos_pub;
ros::Publisher global_lla_pos_pub;
ros::Publisher global_lla_pos_pub_raw;
ros::Subscriber currentPos;
ros::Subscriber state_sub;
ros::ServiceClient arming_client;
ros::ServiceClient land_client;
ros::ServiceClient set_mode_client;
ros::ServiceClient takeoff_client;
ros::ServiceClient command_client;
ros::ServiceClient auto_waypoint_pull_client;
ros::ServiceClient auto_waypoint_push_client;
ros::ServiceClient auto_waypoint_set_current_client;

# Initialization
int init_publisher_subscriber(ros::NodeHandle controlnode)
{
	std::string ros_namespace;
	if (!controlnode.hasParam("namespace"))
	{

		ROS_INFO("using default namespace");
	}else{
		controlnode.getParam("namespace", ros_namespace);
		ROS_INFO("using namespace %s", ros_namespace.c_str());
	}
	local_pos_pub = controlnode.advertise<geometry_msgs::PoseStamped>((ros_namespace + "/mavros/setpoint_position/local").c_str(), 10);
	global_lla_pos_pub = controlnode.advertise<geographic_msgs::GeoPoseStamped>((ros_namespace + "/mavros/setpoint_position/global").c_str(), 10);
	global_lla_pos_pub_raw = controlnode.advertise<mavros_msgs::GlobalPositionTarget>((ros_namespace + "/mavros/setpoint_raw/global").c_str(), 10);
	currentPos = controlnode.subscribe<nav_msgs::Odometry>((ros_namespace + "/mavros/global_position/local").c_str(), 10, pose_cb);
	state_sub = controlnode.subscribe<mavros_msgs::State>((ros_namespace + "/mavros/state").c_str(), 10, state_cb);
	arming_client = controlnode.serviceClient<mavros_msgs::CommandBool>((ros_namespace + "/mavros/cmd/arming").c_str());
	land_client = controlnode.serviceClient<mavros_msgs::CommandTOL>((ros_namespace + "/mavros/cmd/land").c_str());
	set_mode_client = controlnode.serviceClient<mavros_msgs::SetMode>((ros_namespace + "/mavros/set_mode").c_str());
	takeoff_client = controlnode.serviceClient<mavros_msgs::CommandTOL>((ros_namespace + "/mavros/cmd/takeoff").c_str());
	command_client = controlnode.serviceClient<mavros_msgs::CommandLong>((ros_namespace + "/mavros/cmd/command").c_str());
	auto_waypoint_pull_client = controlnode.serviceClient<mavros_msgs::WaypointPull>((ros_namespace + "/mavros/mission/pull").c_str());
	auto_waypoint_push_client = controlnode.serviceClient<mavros_msgs::WaypointPush>((ros_namespace + "/mavros/mission/push").c_str());
	auto_waypoint_set_current_client = controlnode.serviceClient<mavros_msgs::WaypointSetCurrent>((ros_namespace + "/mavros/mission/set_current").c_str());
	return 0;
}
```


# [Obstacle Avoidance with ROS PX4](https://github.com/PX4/PX4-Avoidance)

## Setup

### Download and Install PX4 and Avoidance Package

Clone PX4 Avoidance package into your catkin source directory and build it:
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/PX4/avoidance.git
$ cd ..
$ catkin build
```

Clone PX4 Firmware (not into your catkin but somewhere else):
```
$ cd ~
$ git clone https://github.com/PX4/Firmware.git --recursive
$ cd ~/Firmware

# Install PX4 "common" dependencies.
$ ./Tools/setup/ubuntu.sh --no-sim-tools --no-nuttx

# Gstreamer plugins (for Gazebo camera)
$ sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly libgstreamer-plugins-base1.0-dev

# Build and run simulation, and quit with ctrl+c after building succeeds
$ make px4_sitl_default gazebo 

# Setup some more Gazebo-related environment variables (modify this line based on the location of the Firmware folder on your machine)
$ . ~/Firmware/Tools/simulation/gazebo/setup_gazebo.bash ~/Firmware ~/Firmware/build/px4_sitl_default

```

### Setup Environment

Make sure you reset all the environment variables in `~/.bashrc` before the sourcing happens:
```
unset GAZEBO_MASTER_URI
unset GAZEBO_MODEL_DATABASE_URI
unset GAZEBO_RESOURCE_PATH
unset GAZEBO_PLUGIN_PATH
unset GAZEBO_MODEL_PATH
unset LD_LIBRARY_PATH
unset OGRE_RESOURCE_PATH
```

And add the code for sourcings after the resets:
```
# for ROS
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# for Gazebo Model
export GAZEBO_MODEL_PATH=/home/weiminn/gazebo_models
source /usr/share/gazebo/setup.sh
export GAZEBO_MODEL_PATH=/home/weiminn/catkin_ws/src/iq_sim/models:${GAZEBO_MODEL_PATH}

# for PX4
px4_dir=/home/weiminn/Firmware
source $px4_dir/Tools/simulation/gazebo-classic/setup_gazebo.bash $px4_dir $px4_dir/build/px4_sitl_default
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:$px4_dir
export GAZEBO_MODEL_PATH=/home/weiminn/catkin_ws/src/avoidance/avoidance/sim/models:/home/weiminn/catkin_ws/src/avoidance/avoidance/sim/worlds:${GAZEBO_MODEL_PATH}
```

## Run Local Planner

Go to `avoidance_sitl_mavros.launch` and set `<arg name="gui" default="false"/>` to `true`.

Afterwards, you can launch the simulation via:
```
$ roslaunch local_planner local_planner_sitl_3cam.launch
```