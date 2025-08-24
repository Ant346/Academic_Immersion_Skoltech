#!/bin/bash

# Copy udev rules
sudo cp 99-hoverboard.rules /etc/udev/rules.d/

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "Udev rules installed. You may need to unplug and replug your devices for the changes to take effect." 