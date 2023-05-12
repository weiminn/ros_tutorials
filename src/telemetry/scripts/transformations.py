import rospy
import tf_conversions 
from tf2_ros import *
from geometry_msgs.msg import *
from gazebo_msgs.msg import *
from tf.transformations import quaternion_from_euler

# callback function to process the message promise
def pose_cb(msg: ModelStates):
    names = msg.name
    positions = msg.pose
    twists = msg.twist

    for name in names:
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

    rospy.loginfo(
        name + ', ' +
        str(timestamp.secs) + ', ' +
        str(pos.x) + ', ' +
        str(pos.y) + ', ' +
        str(pos.z)
        )


if __name__ == "__main__":
    rospy.init_node("gazebo_model_states_subscriber")

    sub = rospy.Subscriber(
        '/gazebo/model_states', 
        ModelStates,
        callback=pose_cb
        )
    
    rospy.loginfo("Subscriber has been initiated")

    rospy.spin() # infinite loop to block till the node has exit