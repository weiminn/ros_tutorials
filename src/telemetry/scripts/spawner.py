#!/home/weiminn/catkin_ws/venv python3

import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel, SetModelState, SetModelStateRequest
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3
from mavros_msgs.srv import *
from mavros_msgs.msg import *
from mavros import mission as M
import random_waypoint_generator as rwg
import random

def spawn(spawner):
    spawner(
        'test_spawn',  
        # str(open('/home/weiminn/gazebo_models/oak_tree/model.sdf', 'r').read()),
        str(open('/home/weiminn/catkin_ws/src/telemetry/unit_box/model.sdf', 'r').read()),
        '/gazebo',
        Pose(position=Point(2.5, 3.5, 0), orientation=Quaternion(0,0,0,0)),
        'world')
    
def move(m_srv, model, vx, vy, vz):
    
    req_obj = SetModelStateRequest()
    req_obj.model_state.model_name = model
    req_obj.model_state.pose.position.x = 3
    req_obj.model_state.pose.position.y = 3
    req_obj.model_state.pose.position.z = 0
    req_obj.model_state.pose.orientation.w = 0
    req_obj.model_state.pose.orientation.x = 0
    req_obj.model_state.pose.orientation.y = 0
    req_obj.model_state.pose.orientation.z = 0
    req_obj.model_state.twist.linear.x = vx
    req_obj.model_state.twist.linear.y = vy
    req_obj.model_state.twist.linear.z = vz
    req_obj.model_state.twist.angular.x = 3
    req_obj.model_state.twist.angular.y = 3
    req_obj.model_state.twist.angular.z = 3
    req_obj.model_state.reference_frame = 'world'
    result = m_srv(req_obj)

    return result
    # m_srv(
    #     model,
    #     Pose(position=Point(2, 2, -5), orientation=Quaternion(0,0,0,0)),
    #     Twist(linear=Vector3(vx, vy, vz), angular=Vector3(0, 0, 0)),
    #     'world'
    # )
    
if __name__ == "__main__":
    rospy.init_node("model_spawner")

    rospy.wait_for_service("gazebo/spawn_sdf_model")
    spawner = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    rospy.wait_for_service("gazebo/delete_model")
    deleter = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

    rospy.wait_for_service("gazebo/set_model_state")
    move_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    
    # spawn(spawner)
    # move(move_service, 'test_spawn', -1, -2, 5)
    # deleter('test_spawn')