#!/usr/bin/python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os, yaml, sys, json, re

dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(dir_path)
import mvsdk



cam_name = "gige"

#### Get the Config Files ####
package_name = os.path.realpath(__file__).split("/")[-3]
config_directory = os.path.join(get_package_share_directory(package_name), 'config')
config_list = os.listdir(config_directory)
config_files = [f for f in config_list if f.startswith(cam_name)]


DevList = mvsdk.CameraEnumerateDevice()
nDev = len(DevList)


dev_serial = []
for dev in DevList:
    dev_serial.append(dev.acSn.decode())

def add_quotes_to_keys(nested_str):
    return re.sub(r'(\w+)\s*:', r'"\1":', nested_str)

# Recursively process the nested string to add double quotes around keys
def process_nested_string(nested_str):
    processed_str = add_quotes_to_keys(nested_str)
    return re.sub(r'{([^{}]+)}', lambda x: "{" + process_nested_string(x.group(1)) + "}", processed_str)

def read_from_file(data):
    address = data["GigE_param_file"]
    if address:
        with open(address, "r") as f:
            lines = f.read().splitlines()[2:]
            lines = '\n'.join(lines).replace("=", ":")
            processed_string = "{" + process_nested_string(lines).replace(";", ",") + "}"
            processed_string = processed_string.replace("false", "False")
            processed_string = processed_string.replace("true", "True")
            dict_from_string = eval(processed_string)

            width = dict_from_string["resolution"]["image_size"]["iWidth"]
            height = dict_from_string["resolution"]["image_size"]["iHeight"]
            data["resolution"] = f"{width},{height}"

            data["invert_color"] = dict_from_string["isp_color"]["inverse"]
            data["saturation"] = dict_from_string["isp_color"]["saturation"]
            data["monochrome"] = dict_from_string["isp_color"]["mono"]

            data["sharpness"] = dict_from_string["isp_shape"]["sharpness"]
            
            data["contrast"] = dict_from_string["isp_lut"]["contrast"]
            data["gamma"] = dict_from_string["isp_lut"]["gamma"]

            data["auto_exposure"] = dict_from_string["exposure"]["ae_enable"]
            data["auto_exposure_target"] = dict_from_string["exposure"]["ae_target"]
            data["exposure_time"] = int(dict_from_string["exposure"]["user_exposure_time"])
            data["analog_gain"] = int(dict_from_string["exposure"]["analog_gain"])
            data["light_frequency"] = 60 if dict_from_string["exposure"]["anti_flick_freq"] else 50
            data["antiflick"] = dict_from_string["exposure"]["anti_flick"]

            frame_speed = dict_from_string["video_format"]["frame_speed_sel"]
            data["fps"] = (frame_speed + 1)*10

    return data

node_list = []
for file_name in config_files:
    file_directory = os.path.join(get_package_share_directory(package_name), 'config', file_name)
    with open(file_directory, 'r') as file: data = yaml.safe_load(file)
    
    if data["serial"] in dev_serial:
        data["serial"] += "," 
        
        if data["load_from_file"] and len(data["GigE_param_file"]) > 0:
            data["GigE_param_file"] = os.path.join(get_package_share_directory(package_name), 'config', data["GigE_param_file"])

            data = read_from_file(data)

        
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
        
        # arg = LaunchConfiguration(variable_name=name, default = name)
        param = [{key: value} for key, value in data.items()]

        # nd.append(Node(package=package_name, executable=executable, namespace = [arg], parameters=param, output='screen'))
        node_list.append(Node(package="gige", executable="gige_camera", name="Camera", namespace = "GigE", parameters=param, output='screen'))
        
def generate_launch_description():
    return LaunchDescription(node_list)
