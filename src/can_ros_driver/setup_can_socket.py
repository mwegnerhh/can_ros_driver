import os
import yaml
from can_ros_driver.can_socket import CANSocket
import rospy

def load_can_bus_names():
    yaml_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "config", "params.yaml"))
    with open(yaml_path, "r") as file:
        config = yaml.safe_load(file)
    return config.get("can_ros_driver", {}).get("can_bus_names", [])

def instantiate_can_sockets():
    can_bus_names = load_can_bus_names()
    for name in can_bus_names:
        CANSocket(name)
        rospy.loginfo(f"Started CANSocket for bus: {name}")