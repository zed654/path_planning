cmake_minimum_required(VERSION 3.5)
project(hybrid_a_star_algorithm_pkg)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)
# find_package(ex_msg_srv REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
    include
)


add_executable(${PROJECT_NAME} src/main.cpp src/main.cpp src/monitor.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC "${Eigen3_INCLUDE_DIRS}")
ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs OpenCV eigen3_cmake_module Eigen3)

install(TARGETS
	${PROJECT_NAME}
	DESTINATION
	lib/${PROJECT_NAME})
install(DIRECTORY
	launch
	DESTINATION
	share/${PROJECT_NAME}) # To launch, this line shoudl be writen

ament_package()
