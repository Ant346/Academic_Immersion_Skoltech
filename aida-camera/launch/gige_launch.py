#!/usr/bin/python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import sys
import yaml

dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(dir_path)
import mvsdk
from gige_launch_utils import apply_param_file_overrides, normalize_serial



cam_name = "gige"
package_name = "gige"

#### Get the Config Files ####
config_directory = os.path.join(get_package_share_directory(package_name), 'config')
config_list = os.listdir(config_directory)
config_files = [f for f in config_list if f.startswith(cam_name)]


try:
    dev_list = mvsdk.CameraEnumerateDevice()
except Exception:
    dev_list = []

dev_serial = {normalize_serial(dev.acSn.decode(errors="ignore")) for dev in dev_list}

node_list = []
matched_any_camera = False
for file_name in config_files:
    file_directory = os.path.join(get_package_share_directory(package_name), 'config', file_name)
    with open(file_directory, 'r') as file: data = yaml.safe_load(file)

    config_serial = normalize_serial(data.get("serial", ""))
    if config_serial in dev_serial:
        matched_any_camera = True
        data["serial"] = config_serial

        if data["load_from_file"] and len(data["GigE_param_file"]) > 0:
            data["GigE_param_file"] = os.path.join(get_package_share_directory(package_name), 'config', data["GigE_param_file"])
            data = apply_param_file_overrides(data)

        
        data.pop("load_from_file", None)  

        index = data["index"]
        name = f"GigE_Camera_{index}"      

        position = data["position"].split(",")
        orientation = data["orientation"].split(",")

        node_list.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = [position[0], position[1], position[2], orientation[0], orientation[1], orientation[2], name, name + "_robot"]
        ))
        
        param = [{key: value} for key, value in data.items()]

        node_list.append(Node(package="gige", executable="gige_camera", name="Camera", namespace = "GigE", parameters=param, output='screen'))

if (not matched_any_camera) and dev_list and config_files:
    # Fallback mode: bind the first available camera to the first config.
    fallback_config = os.path.join(get_package_share_directory(package_name), 'config', config_files[0])
    with open(fallback_config, 'r') as file:
        data = yaml.safe_load(file)

    data["serial"] = normalize_serial(dev_list[0].acSn.decode(errors="ignore"))

    if data["load_from_file"] and len(data["GigE_param_file"]) > 0:
        data["GigE_param_file"] = os.path.join(get_package_share_directory(package_name), 'config', data["GigE_param_file"])
        data = apply_param_file_overrides(data)

    data.pop("load_from_file", None)

    index = data["index"]
    name = f"GigE_Camera_{index}"

    position = data["position"].split(",")
    orientation = data["orientation"].split(",")

    node_list.append(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[position[0], position[1], position[2], orientation[0], orientation[1], orientation[2], name, name + "_robot"]
    ))

    param = [{key: value} for key, value in data.items()]
    node_list.append(Node(package="gige", executable="gige_camera", name="Camera", namespace="GigE", parameters=param, output='screen'))
        
def generate_launch_description():
    return LaunchDescription(node_list)
