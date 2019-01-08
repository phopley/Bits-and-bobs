#!/usr/bin/env python

import rospy
import actionlib
from face_recognition_msgs.msg import scan_for_facesAction, scan_for_facesGoal, scan_for_facesResult

rospy.init_node('face_recognition_client')
client = actionlib.SimpleActionClient('face_recognition', scan_for_facesAction)
client.wait_for_server()
goal = scan_for_facesGoal()
client.send_goal(goal)
client.wait_for_result()
result = client.get_result()
print(result.ids_detected)
print(result.names_detected)
