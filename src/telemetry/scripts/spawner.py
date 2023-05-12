import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, Point, Quaternion

def spawn(spawner):
    spawner(
        'test_spawn',  
        str(open('/home/weiminn/catkin_ws/src/telemetry/unit_box/model.sdf', 'r').read()),
        '/gazebo',
        Pose(position=Point(1.5, .5, 1), orientation=Quaternion(0,0,0,0)),
        'world')
    
if __name__ == "__main__":
    rospy.init_node("model_spawner")

    rospy.wait_for_service("gazebo/spawn_sdf_model")
    rospy.wait_for_service("gazebo/delete_model")
    spawner = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    deleter = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    
    spawn(spawner)