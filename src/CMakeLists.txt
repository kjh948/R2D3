cmake_minimum_required(VERSION 3.0.2)
project(r2d3_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  std_srvs
  message_generation
)

# Generate custom service messages
add_service_files(
  FILES
  PlayAudio.srv
)

# Generate added messages/services with dependencies
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

# Install python node scripts
catkin_install_python(PROGRAMS
  r2d3_control/dome_node.py
  r2d3_control/led_node.py
  r2d3_control/motor_node.py
  r2d3_control/audio_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install all python package files (wrappers, __init__, etc.) using standard install
install(FILES
  r2d3_control/__init__.py
  r2d3_control/dome_wrapper.py
  r2d3_control/led_wrapper.py
  r2d3_control/wheel_wrapper.py
  r2d3_control/audio_wrapper.py
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)
