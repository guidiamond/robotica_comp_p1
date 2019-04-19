#!/bin/bash
rosrun p1_b_ros p1_mobilenet.py | egrep -v "^\[INFO\]|frame"
rostopic pub -1 /cmd_vel geometry_msgs/Twist -- '[0.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]'

