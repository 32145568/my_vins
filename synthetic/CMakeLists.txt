cmake_minimum_required(VERSION 2.8.3)
project(synthetic)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}
    ${CERES_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/src
)

add_executable(synthetic 
    src/test_main.cpp
    src/generate_synthetic_data.cpp
)

target_link_libraries(synthetic ${OpenCV_LIBS} ${CERES_LIBRARIES})
