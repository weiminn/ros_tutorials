import rospy
import time
import tf_conversions 
from tf2_ros import *
from geometry_msgs.msg import *
from gazebo_msgs.msg import *
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import euler_from_quaternion
import random

i = 0
rospy.wait_for_service("gazebo/spawn_sdf_model")
rospy.wait_for_service("gazebo/delete_model")
spawner = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
deleter = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

def spawn(name, x, y, z):
    global spawner
    spawner(
        name,  
        # str(open('/home/weiminn/gazebo_models/oak_tree/model.sdf', 'r').read()),
        str(open('/home/weiminn/catkin_ws/src/telemetry/unit_box/model.sdf', 'r').read()),
        '/gazebo',
        Pose(position=Point(x, y, z), orientation=Quaternion(0,0,0,0)),
        'world')

# callback function to process the message promise
def pose_cb(msg: ModelStates):
    names = msg.name
    positions = msg.pose
    twists = msg.twist

    for name in names:
        if name == 'iris_obs_avoid':
            index = names.index(name)
            pos = positions[index].position
            ori = positions[index].orientation
            # twt = twists[index]
            # lin = twt.linear
            # ang = twt.angular

            timestamp = rospy.Time.now()
            broadcast(name, timestamp, pos, ori)

def broadcast(name, timestamp, pos: Pose, ori: Quaternion):

    #broadast transformation of the poses relative to the world
    br = TransformBroadcaster()
    t = TransformStamped()

    t.header.stamp = timestamp
    t.header.frame_id = "world"
    t.child_frame_id = name
    t.transform.translation.x = pos.x
    t.transform.translation.y = pos.y
    t.transform.translation.z = pos.z
    t.transform.rotation.x = ori.x
    t.transform.rotation.y = ori.y
    t.transform.rotation.w = ori.w
    t.transform.rotation.z = ori.z

    br.sendTransform(t)

    global i
    if timestamp.secs >= i+4:
        i = timestamp.secs
        r,p,y = euler_from_quaternion([ori.x,ori.y,ori.z,ori.w])
        rospy.loginfo(
            name + ', ' +
            str(timestamp.secs) + ', ' +
            str(pos.x) + ', ' +
            str(pos.y) + ', ' +
            str(pos.z) + ', ' +
            str(y)
            )
        
        name = 'test_spawn'
        
        global deleter
        deleter(name)
        time.sleep(1)
        x_noise = random.randrange(-4, -1) if random.random() < 0.5 else random.randrange(2, 5)
        y_noise = random.randrange(-4, -1) if random.random() < 0.5 else random.randrange(2, 5)
        z_noise = random.randrange(-3, -1) if random.random() < 0.5 else random.randrange(2, 4)
        spawn(name, pos.x + x_noise, pos.y + y_noise, pos.z)



if __name__ == "__main__":
    rospy.init_node("gazebo_model_states_subscriber")

    sub = rospy.Subscriber(
        '/gazebo/model_states', 
        ModelStates,
        callback=pose_cb
        )
    
    rospy.loginfo("Subscriber has been initiated")

    rospy.spin() # infinite loop to block till the node has exit