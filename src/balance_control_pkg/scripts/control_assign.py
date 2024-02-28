#!/usr/bin/python
'''
right_cmd = base + diff
left_cmd = base - diff
'''
import rospy
from std_msgs.msg import Float64


velocity = 0.0
pusai = 0.0
def velocity_cb(msg):
    global velocity1 
    velocity1 = msg.data

def pusai_cb(msg):
    global pusai 
    pusai = msg.data

rospy.Subscriber('/balance/base_command',Float64,velocity_cb,queue_size=1)
rospy.Subscriber('/balance/diff_command',Float64,pusai_cb,queue_size=1)
right_wheel_command_pub = rospy.Publisher('/balance/cmd/right_wheel',Float64,queue_size=10)
left_wheel_command_pub = rospy.Publisher('/balance/cmd/left_wheel',Float64,queue_size=10)

rospy.init_node('control_assign')
r = rospy.Rate(300)

while not rospy.is_shutdown():
    right_cmd = Float64(velocity + pusai)
    left_cmd = Float64(velocity - pusai)
    right_wheel_command_pub.publish(right_cmd)
    left_wheel_command_pub.publish(left_cmd)
    r.sleep()