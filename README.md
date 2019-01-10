# Test code for robotic projects
## odom_test
odom_test1_node.py  
ROS node that prints out the yaw of a robot in degrees
## rodney_head_test
ROS node for testing the head_control node
* Key 1 - move to max pan and tilt
* Key 2 - move to min pan and tilt
* key 3 - move to pan 0, tilt 0
* key 4 - move to pan 0, tilt -45 degress
* key 5 - move tilt up by 10 degrees
* key 6 - move pan by 20 anti-clockwise
* key 7 - move pan by 20 clockwise and tilt by 10 down
* key c - Cancel action
## rodney_object_test
Simple ROS node which tests the tf_object_detection node
## rodney_recognition_test
Simple ROS node which tests the face_recognition node
## rodney_sim_control
ROS package that launches the rodney urdf model in a Gazebo empty world
## rodney_voice_test
ROS node for a Rodney robot intergration test. Test the following packages together:
* homer_robot_face
* speech


