#!/usr/bin/env python3
import cantools
import os
from pathlib import Path
import yaml


# Import paths from yaml file 
yaml_path = os.path.join(os.path.dirname(__file__), "..", "config", "params.yaml")
yaml_path = os.path.abspath(yaml_path)

with open(yaml_path, "r") as file:
    config = yaml.safe_load(file)

# Extract DBC file path from the YAML structure 
DIR_DBC = config.get("can_ros_driver", {}).get("dbc_file_path", "")

# Set DIR_MSG and DIR_CMAKELISTS based on your folder structure
DIR_MSG = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "msg"))
DIR_CMAKELISTS = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "CMakeLists.txt"))


### This script converts creates ROS messages in the /msg folder from the DBC file
def clear_old_msgs_and_cmakelists(DIR_MSG, DIR_CMAKELISTS):
    # Remove all .msg files in the msg directory
    for file in os.listdir(DIR_MSG):
        if file.endswith(".msg"):
            os.remove(os.path.join(DIR_MSG, file))
    print("Cleared old .msg files in", DIR_MSG)

    # Remove all .msg entries from add_message_files in CMakeLists.txt
    with open(DIR_CMAKELISTS, 'r') as file:
        lines = file.readlines()

    new_lines = []
    inside_add_message_files = False
    for line in lines:
        if 'add_message_files(' in line:
            inside_add_message_files = True
            new_lines.append(line)
            continue
        if inside_add_message_files:
            if ')' in line:
                inside_add_message_files = False
                new_lines.append(line)
            # skip .msg lines
            continue
        new_lines.append(line)

    with open(DIR_CMAKELISTS, 'w') as file:
        file.writelines(new_lines)
    print("Cleared old .msg entries in CMakeLists.txt")

def convert_dbc_to_ros_msgs():
    # Ensure the DBC file exists
    if not os.path.isfile(DIR_DBC):
        print(f"DBC file not found: {DIR_DBC}")
        return

    # Ensure output directory exists
    Path(DIR_MSG).mkdir(parents=True, exist_ok=True)

    # Clear old msg files and CMakeLists.txt entries
    clear_old_msgs_and_cmakelists(DIR_MSG, DIR_CMAKELISTS)

    # Load the DBC file
    try:
        db = cantools.database.load_file(DIR_DBC)
    except Exception as e:
        print(f"Failed to load DBC file: {e}")
        return

    message_names = []
    for message in db.messages:
        message_name = message.name
        signals = message.signals
        
        # Create ROS message file content
        ros_msg_content = "std_msgs/Header header\n"
        for signal in signals:
            signal_name = signal.name
            signal_type = get_ros_type(signal)
            ros_msg_content += f"{signal_type} {signal_name}\n"
        
        # Write the ROS message file
        ros_msg_filename = os.path.join(DIR_MSG, f"{message_name}.msg")
        with open(ros_msg_filename, 'w') as file:
            file.write(ros_msg_content)

        message_names.append(f"{message_name}.msg")
        
    print("Updated ROS msg files in", DIR_MSG)
    update_cmakelists(DIR_CMAKELISTS, message_names)

def get_ros_type(signal):
    # Check if the signal is a float based on its properties
    if hasattr(signal, 'is_float') and signal.is_float:
        if signal.length == 32:
            return "float32"
        elif signal.length == 64:
            return "float64"
        else:
            raise ValueError("Unsupported float signal length")
    
    # Map DBC signal types to ROS basic types for integers
    if signal.length <= 8:
        return "int8" if signal.is_signed else "uint8"
    elif signal.length <= 16:
        return "int16" if signal.is_signed else "uint16"
    elif signal.length <= 32:
        return "int32" if signal.is_signed else "uint32"
    elif signal.length <= 64:
        return "int64" if signal.is_signed else "uint64"
    else:
        raise ValueError("Unsupported signal length")

def update_cmakelists(DIR_CMAKELISTS, message_names):
    # Read the existing CMakeLists.txt content
    with open(DIR_CMAKELISTS, 'r') as file:
        cmakelists_content = file.readlines()

    # Find the add_message_files section
    add_message_files_index = None
    for i, line in enumerate(cmakelists_content):
        if 'add_message_files(' in line:
            add_message_files_index = i
            break

    if add_message_files_index is not None:
        # Find the end of the add_message_files section
        end_index = add_message_files_index
        while ')' not in cmakelists_content[end_index]:
            end_index += 1

        # Insert new message names before the closing parenthesis
        for message_name in message_names:
            msg_line = f"  {message_name}\n"
            if msg_line not in cmakelists_content:
                cmakelists_content.insert(end_index, msg_line)
                end_index += 1

        # Write the updated content back to the CMakeLists.txt
        with open(DIR_CMAKELISTS, 'w') as file:
            file.writelines(cmakelists_content)
    
        print("Updated ROS msg in CMakeLists.txt")
    else:
        print("add_message_files section not found in CMakeLists.txt")

if __name__ == "__main__":
    convert_dbc_to_ros_msgs()
