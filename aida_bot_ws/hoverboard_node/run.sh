#!/bin/bash
xhost +local:docker
docker run -it \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /home/ilya/projects/aida_bot_ws:/workspace \
  -v /dev:/dev \
  ros_humble_zsh zsh

  