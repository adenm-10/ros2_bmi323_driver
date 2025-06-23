cmake_minimum_required(VERSION 3.5)
project(bmi323driver)

# C/C++ Standards
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Add the C library from lib/ (must contain CMakeLists.txt defining bmi323sensor target)
add_subdirectory(lib)

# ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Create the node
add_executable(${PROJECT_NAME} src/bmi323_node.cpp)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
)

# Link the C BMI323 library
target_link_libraries(${PROJECT_NAME} bmi323sensor linux_i2c_communicator)
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                       $<INSTALL_INTERFACE:include>)

# Install targets and launch files
install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
ament_package()
