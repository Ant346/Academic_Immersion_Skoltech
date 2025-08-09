#!/bin/bash

set -e

docker build -t ros_humble_zsh .
echo "Docker image 'ros_humble_zsh' built successfully."
