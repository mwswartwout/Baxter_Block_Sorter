cmake_minimum_required(VERSION 2.8.3)
project(color_move)

find_package(catkin_simple REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})
catkin_simple()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries
cs_add_library(color_move_lib src/color_move.cpp)

# Executables
# cs_add_executable(my_interesting_moves src/my_interesting_moves.cpp)

# target_link_libraries(my_interesting_moves my_interesting_moves_lib ${catkin_LIBRARIES})

cs_install()
cs_export()
    
