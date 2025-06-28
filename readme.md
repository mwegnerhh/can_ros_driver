# can_ros_driver

## Overview

`can_ros_driver` is a flexible ROS driver for interfacing with one or more complete CAN bus networks. Using a DBC file, it automatically encodes and decodes all CAN messages on the bus, and dynamically generates the corresponding ROS message types and CMakeLists.txt entries. For each CAN message, the driver automatically sets up ROS publishers and subscribers that follow the naming and structure defined in the DBC file, enabling seamless and automated communication between your CAN network and ROS ecosystem. 


---

## Features

- **Automatic ROS message generation** from DBC files
- **Encoding/decoding** of all CAN messages in DBC file
- **Support for multiple CAN interfaces** (e.g., `can0`, `can1`)
- **Configuration** via YAML
- **ROS publishers and subscribers** for each CAN message

---

## Requirements

- Python 3.7+
- [ROS Noetic](http://wiki.ros.org/noetic) (or compatible ROS 1 distro)
- [python-can](https://python-can.readthedocs.io/en/master/)
- [cantools](https://cantools.readthedocs.io/en/latest/)
- [PyYAML](https://pyyaml.org/)
- Linux with SocketCAN support (e.g., Ubuntu)
- A valid DBC file describing your CAN messages

Install dependencies with:

```bash
pip install python-can cantools pyyaml
sudo apt-get install ros-noetic-rosmsg ros-noetic-std-msgs
```

---

## Setup

1. **Clone the repository into your ROS workspace:**

    ```bash
    cd ~/catkin_ws/src
    git clone https://github.com/mwegnerhh/can_ros_driver.git
    ```

2. **Configure path of DBC file and change names of CAN interfaces:**

    Edit `config/params.yaml`:

    ```yaml
    can_ros_driver:
      dbc_file_path: "../dbc/your_can_db.dbc"  
      can_bus_names:
        - can1
        - can2
        - ...
    ```

3. **Place your DBC file** in the `dbc/` directory.

4. **Generate ROS messages from the DBC file:**

    ```bash
    cd src/can_ros_driver/scripts
    ./update_can_msgs.py
    ```

5. **Build your workspace:**

    ```bash
    cd ~/catkin_ws
    catkin build
    source devel/setup.bash
    ```

---

## Usage

1. **Bring up your CAN interface(s):**

    ```bash
    sudo ip link set can1 up type can
    sudo ip link set can2 up type can 
    ```

    For development and testing, you can use virtual CAN interfaces.  
    Scripts for setting up virtual CAN interfaces (e.g., `vcan0`, `vcan1`) are provided in the `scripts/` directory.  

2. **Run the CAN driver node:**

    ```bash
    roslaunch can_ros_driver can_driver_node.py
    ```

3. **Topics:**

    - Publishes decoded CAN messages to:  
      `/can_bus/<interface>/from_can_bus/<MessageName>`
    - Subscribes for outgoing CAN messages on:  
      `/can_bus/<interface>/to_can_bus/<MessageName>`

---


## License

MIT License

---

## Authors
- Michel Wegner, contact@michelwegner.de