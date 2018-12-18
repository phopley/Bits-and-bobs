#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
rospy.init_node('odom_test2_node')

stop_twist = Twist()
go_twist = Twist()
go_twist.linear.x = 0.5
go_twist.angular.z = 0.25

driving = True
started = False

rate = rospy.Rate(10)

change_time = rospy.Time.now()

while not rospy.is_shutdown():
    rate.sleep()

    if started == False:
        change_time = rospy.Time.now() + rospy.Duration(1)
        started = True
        
    if driving:
        cmd_vel_pub.publish(go_twist)
    else:
        cmd_vel_pub.publish(stop_twist)
        
    if rospy.Time.now() > change_time:
        driving = False

    
    
