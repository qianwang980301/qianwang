cmake_minimum_required(VERSION 2.8.3)
project(grabsth)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin_simple REQUIRED)
catkin_simple()
cs_add_library(action_client src/action_client.cpp)
cs_add_executable(grabsth_usage src/grabsth_usage.cpp)

target_link_libraries(grabsth_usage action_client ${catkin_LIBRARIES})

cs_install()
cs_export()