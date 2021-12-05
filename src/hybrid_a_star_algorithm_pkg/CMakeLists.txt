cmake_minimum_required(VERSION 2.8.3)
project(hybrid_a_star_algorithm_pkg)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3 -pthread -Wall")

#find_package(Torch REQUIRED)
find_package(OpenCV)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    #sim_test
)

catkin_package(
    #  INCLUDE_DIRS include
    LIBRARIES hybrid_a_star_algorithm_pkg
    CATKIN_DEPENDS roscpp std_msgs
    DEPENDS system_lib
)


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    /usr/local/include
    /usr/include
    src
    $(CMAKE_CURRENT_SOURCE_DIR)/ThridParty
)

link_directories(
    /usr/local/lib
    ${CMAKE_CURRENT_SOURCE_DIR}/lib
)

#MESSAGE(${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty)

# Add OpenCV Libraries
link_directories(${OpenCV_INCLUDE_DIRS})
link_libraries(${OpenCV_LIBS})

add_executable(hybrid_a_star_algorithm_pkg_exe
	src/main.cpp
	src/monitor.cpp
        include/hybrid_astar.hpp
)

target_link_libraries(hybrid_a_star_algorithm_pkg_exe ${catkin_LIBRARIES})

