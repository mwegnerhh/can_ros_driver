cmake_minimum_required(VERSION 3.0.2)
project(can_ros_driver)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files( FILES
  VEHICLE_CONTROL.msg
  VEHICLE_STATUS.msg
  OBSTACLE_DETECTION.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(CATKIN_DEPENDS
message_runtime 
rospy 
std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)




