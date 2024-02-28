#!/usr/bin/python
import rospy
import tf
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import WrenchStamped
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


def publish_stl(file_name,pose,id):
    global stl_pub
    stl_marker = Marker ()
    stl_marker.type = Marker.MESH_RESOURCE
    stl_marker.mesh_resource = file_name
    stl_marker.action = Marker.ADD
    stl_marker.id = id
    stl_marker.lifetime = rospy.Duration(0)
    stl_marker.header.frame_id = 'world'
    stl_marker.header.stamp = rospy.Time.now()
    stl_marker.ns="stl"
    stl_marker.pose.position.x = pose[0]
    stl_marker.pose.position.y = pose[1]
    stl_marker.pose.position.z = pose[2]
    q = quaternion_from_euler(pose[3],pose[4],pose[5],'sxyz')
    stl_marker.pose.orientation.x = q[0]
    stl_marker.pose.orientation.y = q[1]
    stl_marker.pose.orientation.z = q[2]
    stl_marker.pose.orientation.w = q[3]
    stl_marker.scale.x = 1
    stl_marker.scale.y = 1
    stl_marker.scale.z = 1
    stl_marker.color.r = 255
    stl_marker.color.g = 0
    stl_marker.color.b = 0
    stl_marker.color.a = 0.7

    marker_array = MarkerArray()
    marker_array.markers.append(stl_marker)
    stl_pub.publish(marker_array)

def publish_tf(frame_name):
    global marker_array_pub
    if listener.canTransform(frame_name, "/world", rospy.Time().now()):
        (trans,rot) = listener.lookupTransform(frame_name,'/world',rospy.Time.now())
        print(trans)


def left_wrench_cb(wrench_msg):
    global marker_array_pub
    marker_msg = Marker()
    # wrench_msg.wrench.force.x,wrench_msg.wrench.force.y,wrench_msg.wrench.force.z
    marker_msg.header.frame_id = "left_contact"
    marker_msg.ns = "left_grf"
    marker_msg.id = 1
    marker_msg.type = Marker.ARROW
    marker_msg.action = Marker.ADD
    marker_msg.scale.x = 0.05
    marker_msg.scale.y = 0.05
    marker_msg.scale.z = 0.4
    marker_msg.color.a = 1.0
    marker_msg.color.r = 0.4940
    marker_msg.color.g = 0.1840
    marker_msg.color.b = 0.5560
    marker_msg.points = [Point(-0.01*wrench_msg.wrench.force.x,-0.01*wrench_msg.wrench.force.y,-0.01*wrench_msg.wrench.force.z),Point(0,0,0)]

    marker_array = MarkerArray()
    marker_array.markers.append(marker_msg)
    marker_array_pub.publish(marker_array)

def right_wrench_cb(wrench_msg):
    global marker_array_pub
    marker_msg = Marker()
    # wrench_msg.wrench.force.x,wrench_msg.wrench.force.y,wrench_msg.wrench.force.z
    marker_msg.header.frame_id = "right_contact"
    marker_msg.ns = "right_grf"
    marker_msg.id = 1
    marker_msg.type = Marker.ARROW
    marker_msg.action = Marker.ADD
    marker_msg.scale.x = 0.05
    marker_msg.scale.y = 0.05
    marker_msg.scale.z = 0.4
    marker_msg.color.a = 1.0
    marker_msg.color.r = 0.4660
    marker_msg.color.g = 0.6740
    marker_msg.color.b =  0.1880
    marker_msg.points = [Point(-0.01*wrench_msg.wrench.force.x,-0.01*wrench_msg.wrench.force.y,-0.01*wrench_msg.wrench.force.z),Point(0,0,0)]

    marker_array = MarkerArray()
    marker_array.markers.append(marker_msg)
    marker_array_pub.publish(marker_array)


rospy.init_node("marker_grf")
marker_array_pub = rospy.Publisher('/od/marker_array', MarkerArray, queue_size=10)
stl_pub = rospy.Publisher('/stl', MarkerArray, queue_size=10)

rospy.Subscriber('/od/left_wrench',WrenchStamped,left_wrench_cb,queue_size=10)
rospy.Subscriber('/od/right_wrench',WrenchStamped,right_wrench_cb,queue_size=10)


rate = rospy.Rate(10)
listener = tf.TransformListener()
while not rospy.is_shutdown():
    publish_stl("package://od_model/meshes/10_1.STL",[1.59332, 0, 0, 1.57083, -0, 0],0)
    publish_stl("package://od_model/meshes/10_3.STL",[1.89281, 0.002712, -0.00047, 1.56691, -0, 0],1)
    publish_stl("package://od_model/meshes/10_3.STL",[1.59397, -0.499618, -0.000449, 1.57468, 0, -3.14],2)
    # publish_tf('/base_footprint')
    # if listener.canTransform('world','base_footprint', rospy.Time(0)):
    #     (trans,rot) = listener.lookupTransform('world','base_footprint',rospy.Time(0))
    rate.sleep()