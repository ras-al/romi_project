cmake_minimum_required(VERSION 3.8)
project(romi_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find packages
find_package(ament_cmake REQUIRED)

find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-transport11 REQUIRED)
find_package(ignition-msgs8 REQUIRED)
find_package(ignition-math6 REQUIRED)
find_package(ignition-plugin1 REQUIRED)

# Include directories
include_directories(
  include
)

# Source files for robot models
set(ROBOT_MODELS_SOURCES
  src/pid_control.cpp
)

# Plugin source files
set(PLUGIN_SOURCES
  src/romi_encoder_plugin.cpp
  ${ROBOT_MODELS_SOURCES}
)

# Create shared library for the plugin
add_library(romi_encoder_plugin SHARED ${PLUGIN_SOURCES})
target_include_directories(romi_encoder_plugin PUBLIC include)

target_link_libraries(romi_encoder_plugin
  ignition-gazebo6::core
  ignition-transport11::core
  ignition-msgs8::ignition-msgs8
  ignition-math6::ignition-math6
  ignition-plugin1::loader
)

# Install plugin
install(TARGETS romi_encoder_plugin
  DESTINATION lib
)

# Install directories
install(DIRECTORY
  launch
  models
  meshes
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/warehouse_explorer.py
  scripts/export_sensor_data.py
  scripts/odom_to_tf.py
  scripts/hold_teleop.py
  scripts/record_video.py
  DESTINATION lib/${PROJECT_NAME}
)

# Set up environment hook for GZ_SIM_SYSTEM_PLUGIN_PATH
ament_environment_hooks(
  "env_hook/romi_gazebo.dsv.in"
)

ament_package()
