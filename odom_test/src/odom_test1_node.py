#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import pi

roll = pitch = yaw = 0.0
roll_raw = pitch_raw = yaw_raw = 0.0


def get_rotation(msg):
    global roll, pitch, yaw
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    
def get_rotation_raw(msg):
    global roll_raw, pitch_raw, yaw_raw
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll_raw, pitch_raw, yaw_raw) = euler_from_quaternion(orientation_list)    
    

rospy.init_node('odom_test_node')

sub = rospy.Subscriber('/odom', Odometry, get_rotation)
subraw = rospy.Subscriber('/raw_odom', Odometry, get_rotation_raw)


r= rospy.Rate(1)
while not rospy.is_shutdown():
    print('odom {0}'.format((yaw/(2*pi))*360))
    print('raw {0}'.format((yaw_raw/(2*pi))*360))
    print("----------")
    r.sleep()
    
