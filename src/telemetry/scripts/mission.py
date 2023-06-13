#!/home/weiminn/catkin_ws/venv python3

import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel, SetModelState, SetModelStateRequest
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3
from mavros_msgs.srv import *
from mavros_msgs.msg import *
from mavros import mission as M
import random_waypoint_generator as rwg
import random

def set_mode(srv_ins, mode):
    res = srv_ins(custom_mode=mode)
    print(res)

def check_waypoints(srv_ins):
    res = srv_ins()
    # success = res.success
    # wp_received = res.wp_recieved
    # print(success, wp_received)
    print(res)

def randomize_waypoints(n, takeoff_first = False):
    to_return = []
    waypoints = rwg.polygon_random_points(rwg.polygon_corners, n)

    for i in range(len(waypoints)):
        command = 16
        x_lat = waypoints[i].x
        y_long = waypoints[i].y
        z_alt = 5
        if i == 0:
            if takeoff_first:
                command = 22
            x_lat = 47.39772851465713
            y_long = 8.545588483558463
        to_return.append(Waypoint(
            is_current = False,
            frame = 3,
            command = command,
            param1 = 0,
            param2 = 0,
            param3 = 0,
            param4 = 0,
            x_lat = x_lat, 
            y_long = y_long,
            z_alt = random.randrange(10, 20),
            autocontinue = True
        ))

    return to_return

def upload_waypoint(srv_ins, n, takeoff_first = False):
    res = srv_ins(start_index = 0, waypoints = randomize_waypoints(n, takeoff_first))
    print(res)

def arm_disarm(srv_ins, b):
    srv_ins(b)

def clear_missions(srv_ins):
    res = srv_ins()
    print(res)
    
if __name__ == "__main__":
    rospy.init_node("model_spawner")

    rospy.wait_for_service("mavros/mission/pull")
    mission_pull_service = rospy.ServiceProxy('mavros/mission/pull', WaypointPull)

    rospy.wait_for_service("mavros/mission/push")
    mission_push_service = rospy.ServiceProxy('mavros/mission/push', WaypointPush)

    rospy.wait_for_service("mavros/mission/clear")
    mission_clear_service = rospy.ServiceProxy('mavros/mission/clear', WaypointClear)

    rospy.wait_for_service("mavros/set_mode")
    mode_service = rospy.ServiceProxy('mavros/set_mode', SetMode)

    rospy.wait_for_service('mavros/cmd/arming')
    arm_service = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)

    check_waypoints(mission_pull_service)
    clear_missions(mission_clear_service)
    upload_waypoint(mission_push_service, 100, True)
    set_mode(mode_service, "AUTO.MISSION")
    arm_disarm(arm_service, True)
    # set_mode(mode_service, "AUTO.LAND")
    # arm_disarm(arm_service, False)



    # check_waypoints(mission_pull_service)
    # clear_missions(mission_clear_service)
    # upload_waypoint(mission_push_service, 50, False)
    # set_mode(mode_service, "AUTO.MISSION")
    