cmake_minimum_required(VERSION 3.8)
project(dualarm_control)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python executables (make sure these files actually exist and are +x)
install(PROGRAMS
  scripts/robot_a_controller.py
  scripts/robot_b_controller.py
  scripts/vision_processor.py
  scripts/system_coordinator.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
