cmake_minimum_required(VERSION 3.5)
project(pso_planner_pkg)

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


add_executable(${PROJECT_NAME} src/main.cpp src/monitor.cpp)
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



# cmake_minimum_required(VERSION 2.8.3)
# project(pso_planner_pkg)

# # SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3 -pthread -Wall")

# #find_package(Torch REQUIRED)
# find_package(OpenCV)
# find_package(catkin REQUIRED COMPONENTS
#     roscpp
#     std_msgs
#     #sim_test
# )

# catkin_package(
#     #  INCLUDE_DIRS include
#     LIBRARIES pso_planner_pkg
#     CATKIN_DEPENDS roscpp std_msgs
#     DEPENDS system_lib
# )


# include_directories(
#     include
#     ${catkin_INCLUDE_DIRS}
#     /usr/local/include
#     /usr/include
#     src
#     $(CMAKE_CURRENT_SOURCE_DIR)/ThridParty
# )

# link_directories(
#     /usr/local/lib
#     ${CMAKE_CURRENT_SOURCE_DIR}/lib
# )

# #MESSAGE(${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty)

# # Add OpenCV Libraries
# link_directories(${OpenCV_INCLUDE_DIRS})
# link_libraries(${OpenCV_LIBS})

# add_executable(pso_planner_pkg_exe
# 	src/main.cpp
# 	src/monitor.cpp
#         include/astar.hpp
# 	include/pso_planner.hpp
# )

# target_link_libraries(pso_planner_pkg_exe ${catkin_LIBRARIES})

