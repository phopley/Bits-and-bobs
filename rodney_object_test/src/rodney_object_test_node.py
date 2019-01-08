#!/usr/bin/env python

import rospy
import actionlib
from tf_object_detection.msg import scan_for_objectsAction, scan_for_objectsGoal, scan_for_objectsResult

rospy.init_node('object_detection_client')
client = actionlib.SimpleActionClient('object_detection', scan_for_objectsAction)
client.wait_for_server()
goal = scan_for_objectsGoal()
client.send_goal(goal)
client.wait_for_result()
result = client.get_result()
print(result.names_detected)
