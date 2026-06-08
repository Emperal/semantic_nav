DATASET_DIR="/home/robot2/hzf/1asemantic_nav/lib/pyslam/dataset/1"

if [ -d "$DATASET_DIR" ]; then
    echo "Found old dataset folder: $DATASET_DIR"
    echo "Deleting..."
    rm -rf "$DATASET_DIR"
else
    echo "Dataset folder does not exist, skip deleting."
fi
source install/setup.bash
ros2 run orbbec_pyslam_exporter orbbec_tum_exporter \
  --ros-args \
  -p dataset_dir:=/home/robot2/hzf/1asemantic_nav/lib/pyslam/dataset/1 \
  -p groundtruth_source:=odom \
  -p odom_topic:=/dt/odom_info
  #   identity (fake groundtruth file)
  