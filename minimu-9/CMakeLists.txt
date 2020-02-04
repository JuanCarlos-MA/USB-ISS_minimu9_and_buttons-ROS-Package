cmake_minimum_required(VERSION 2.8.3)
project(minimu-9)


find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(CATKIN_DEPENDS roscpp)


include_directories(${catkin_INCLUDE_DIRS})

add_executable(minimu9 src/minimu9.cpp)
target_link_libraries(minimu9 ${catkin_LIBRARIES})
