#!/usr/bin/env bash
set -e

cd /work/Academic_Immersion_Skoltech/aida-camera
source /opt/ros/${ROS_DISTRO}/setup.bash
if [ -f install/setup.bash ]; then
  source install/setup.bash
fi

# Wait until camera image topic is available, then open viewer.
until ros2 topic list 2>/dev/null | grep -q "/Camera_1/color/image_raw"; do
  sleep 1
  echo "[viewer] waiting for /Camera_1/color/image_raw ..."
done

exec ros2 run rqt_image_view rqt_image_view
