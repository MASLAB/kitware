cmake_minimum_required(VERSION 3.8)
project(kitware)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#==========================================================#
#                       Dependencies                       #
#==========================================================#
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

#==========================================================#
#                        Interfaces                        #
#==========================================================#

rosidl_generate_interfaces(${PROJECT_NAME}
    msg/DriveCmd.msg
    msg/EncoderRead.msg
)

#==========================================================#
#                           C++                            #
#==========================================================#
# include_directories(include)

# add_executable(cpp_executable src/cpp_node.cpp)
# ament_target_dependencies(cpp_executable rclcpp)

# install(TARGETS
#   cpp_executable
#   DESTINATION lib/${PROJECT_NAME}
# )

#==========================================================#
#                         Python                           #
#==========================================================#
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  kitware/kbd_driver.py
  kitware/kitbot.py
  kitware/drive_motor.py
  DESTINATION lib/${PROJECT_NAME}
)

#==========================================================#
#                         Launch                           #
#==========================================================#
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
