#!/bin/bash

source ~/anaconda3/etc/profile.d/conda.sh
conda activate pyslam


source /opt/ros/jazzy/setup.bash

# python src/chassis_semantic/item_identifition/yoloe_ros_runner.py --ros-args \
#   -p min_confidence:=0.3 \
#   -p max_objects_per_frame:=10 \
#   -p max_depth:=5.0 \
#   -p min_depth:=0.2 \
#   -p point_step:=2 \
#   -p interval_sec:=1.0

python lib/pyslam/main_slam.py --headless
