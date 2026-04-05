#!/usr/bin/env bash
set -e

source /opt/ros/${ROS_DISTRO}/setup.bash

if [ -f /work/Academic_Immersion_Skoltech/aida-camera/install/gige/share/gige/local_setup.bash ]; then
  source /work/Academic_Immersion_Skoltech/aida-camera/install/setup.bash
fi

exec "$@"
