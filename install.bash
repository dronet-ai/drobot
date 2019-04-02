#!/bin/bash
cd catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin clean -b -y
catkin_make
. devel/setup.bash
