#!/bin/bash

source ~/anaconda3/etc/profile.d/conda.sh
conda activate itemid


source /opt/ros/jazzy/setup.bash
source install/setup.bash

# python src/chassis_semantic/item_identifition/yoloe_ros_runner.py --ros-args \
#   -p min_confidence:=0.3 \
#   -p max_objects_per_frame:=10 \
#   -p max_depth:=5.0 \
#   -p min_depth:=0.2 \
#   -p point_step:=2 \
#   -p interval_sec:=1.0

python src/chassis_semantic/item_identifition/yoloe_ros_runner.py --ros-args;

python src/chassis_semantic/item_identifition/yoloe_rpc_nav_client.py \
    --ros-args \
  -p target_pose_topic:=/yoloe/target_pose \
  -p rpc_host:=localhost \
  -p rpc_port:=6000 \
  -p map_frame:=map \
  -p base_frame:=base_link \
  -p goal_distance:=0.8