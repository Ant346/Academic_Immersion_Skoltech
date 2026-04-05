#!/usr/bin/env bash
set -e

cd /work/Academic_Immersion_Skoltech/aida-camera
source /opt/ros/${ROS_DISTRO}/setup.bash

# Build on startup so code edits in bind-mounted workspace are picked up automatically.
colcon build --packages-select gige --symlink-install
source install/setup.bash

while true; do
	echo "[publisher] starting gige launch..."
	ros2 launch gige gige_launch.py || true
	echo "[publisher] launch exited; retrying in 2s"
	sleep 2
done
