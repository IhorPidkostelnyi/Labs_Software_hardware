set -eo pipefail

ARDU_WS="${ARDU_WS:-$HOME/lab2_variant15/ardu_ws}"
source /opt/ros/humble/setup.bash
source "$ARDU_WS/install/setup.bash"

export GZ_VERSION=harmonic
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

GAZEBO_SHARE="$(ros2 pkg prefix ardupilot_gazebo)/share"
DESCRIPTION_SHARE="$(ros2 pkg prefix ardupilot_gz_description)/share"
export GZ_SIM_RESOURCE_PATH="$GAZEBO_SHARE:$DESCRIPTION_SHARE${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export GZ_FILE_PATH="$GAZEBO_SHARE:$DESCRIPTION_SHARE${GZ_FILE_PATH:+:$GZ_FILE_PATH}"
export SDF_PATH="$GAZEBO_SHARE:$DESCRIPTION_SHARE${SDF_PATH:+:$SDF_PATH}"

if pgrep -f '[a]rducopter|[m]icro_ros_agent|[i]ris_maze.launch.py' >/dev/null; then
  echo 'A simulation is already running. Stop it before starting a fresh world.'
  exit 1
fi

ros2 launch ardupilot_gz_bringup iris_maze.launch.py rviz:=false
