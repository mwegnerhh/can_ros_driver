#!/usr/bin/env python3

import rospy
import can
import cantools
from functools import partial
import sys

from can_ros_driver.msg import *

# Create a mapping of message names to ROS message classes
ROS_MSG_CLASS_MAP = {name: cls for name, cls in vars(sys.modules[__name__]).items() if isinstance(cls, type)}

class CANSocket:
    
    def __init__(self, interface_name):

        self.interface_name = interface_name
                
        # Instantiate can bus object
        self.can_bus = can.interface.Bus(channel=self.interface_name, bustype='socketcan')

        # Load the DBC file
        self.path_dbc = rospy.get_param('~dbc_file_path')

        try:
            self.db = cantools.database.load_file(self.path_dbc)
        except Exception as e:
            rospy.loginfo(f"Failed to load DBC file: {e}")
            return
        
        # Use the interface name directly in the topic paths for arbitrary CAN bus names
        self.message_names = [msg.name for msg in self.db.messages]
        self.can_interface = self.interface_name.replace(' ', '_')
        self.pub_topics = [f"{self.can_interface}/from_can_bus/{name.replace(' ', '_')}" for name in self.message_names]
        self.sub_topics = [f"{self.can_interface}/to_can_bus/{name.replace(' ', '_')}" for name in self.message_names]

        # Setup Publishers and Subscribers
        self.publishers = {}
        self.subscriber = {}

        # Create publishers for each topic using the same index for pub_topics and message names
        for name, pub_topic in zip(self.message_names, self.pub_topics):
            self.publishers[name] = rospy.Publisher(pub_topic, self.get_object_by_name(name), queue_size=10)

        # Create subscriber for each topic using the same index for topics and message names
        for name, sub_topic in zip(self.message_names, self.sub_topics):
            self.subscriber[name] = rospy.Subscriber(sub_topic, self.get_object_by_name(name), partial(self.on_data_for_can_bus, name=name))

        # Create a CAN notifier and attach the callback
        self.notifier = can.Notifier(self.can_bus, [self.on_data_from_can_bus])

    
    def on_data_for_can_bus(self, msg, name):
        try:
            # Get the message type from the DBC file
            msg_type = self.db.get_message_by_name(name)
            
            # Convert the ROS message to a dictionary for encoding
            data_dict = {}
            for signal in msg_type.signals:
                # Get the actual data from the ROS message field
                value = getattr(msg, signal.name)
                data_dict[signal.name] = value
            
            # Encode the dictionary using the DBC message type
            payload = msg_type.encode(data_dict, scaling=True)
            
            # Create a CAN frame with the encoded payload
            can_frame = can.Message(
                arbitration_id=msg_type.frame_id,
                dlc=msg_type.length,
                data=payload,
                is_extended_id=False  # Ensure the CAN ID is not extended
            )
            
            # Send the CAN frame
            self.can_bus.send(can_frame)

        except Exception as e:
            rospy.logerr(f"Failed to send CAN message: {e}")


    def on_data_from_can_bus(self, msg):
        try:
            # Ensure msg is of the expected type
            if not isinstance(msg, can.Message):
                raise ValueError(f"Unexpected message type: {type(msg)}")

            # Decode the CAN message using the DBC file
            decoded_msg = self.db.decode_message(msg.arbitration_id, msg.data, scaling=True)
            message_name = self.db.get_message_by_frame_id(msg.arbitration_id).name            

            # Create a ROS message based on the decoded data
            ros_msg_class = self.get_object_by_name(message_name)
            if not ros_msg_class:
                raise ValueError(f"ROS message class for {message_name} not found")

            # Instantiate the ROS message object
            ros_msg = ros_msg_class()

            # Loop through the signals in the message and add them to the ROS message
            for signal in decoded_msg:
                setattr(ros_msg, signal, decoded_msg[signal])

            # Ensure the ROS message is of the correct type
            if not isinstance(ros_msg, rospy.Message):
                raise ValueError(f"ros_msg is not a rospy.Message: {type(ros_msg)}")

            # Publish to CAN bus ROS topic
            self.publishers[message_name].publish(ros_msg)  
            
        except Exception as e:
            rospy.logerr(f"Failed to decode CAN message: {e} | Message: {msg}")
            
            
    # This method retrieves the ROS message class from the mapping based on the name
    def get_object_by_name(self, name):
        try:
            if name in ROS_MSG_CLASS_MAP:
                return ROS_MSG_CLASS_MAP[name]
            else:
                raise NameError(f"No ROS message class named '{name}' found")
        except Exception as e:
            rospy.logerr(f"Error in get_object_by_name: {e}")
            return None

