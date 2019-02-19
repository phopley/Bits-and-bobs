#!/bin/bash
# Pan head anticlockwise
rostopic pub -1 /head_controller/command trajectory_msgs/JointTrajectory '{joint_names: ["head_pan", "head_tilt"], points: [{positions: [-1.57, 0.0], time_from_start: [1.0, 0.0]}]}'
