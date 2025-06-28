#!/usr/bin/env python3

import rospy
from can_ros_driver.setup_can_socket import instantiate_can_sockets

if __name__ == "__main__":

    # Initialize Node
    rospy.init_node("can_ros_driver", anonymous=False)

    instantiate_can_sockets() 

    rospy.spin()
