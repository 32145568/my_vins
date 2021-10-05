cmake_minimum_required(VERSION 2.8.3)
project(synthetic)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    cv_bridge
)

catkin_package()

find_package(OpenCV REQUIRED)
find_package(Eigen3)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/src
)

set(CMAKE_MODULE_PATH cmake)

add_executable(synthetic 
    src/generate_node.cpp
    src/generate_synthetic_data.cpp
)

target_link_libraries(synthetic ${catkin_LIBRARIES} ${OpenCV_LIBS})
