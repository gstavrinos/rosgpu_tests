cmake_minimum_required(VERSION 2.8.3)
project(rosgpu_tests)

add_compile_options(-std=c++11)

find_package(OpenCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
)


include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OpenCL_INCLUDE_DIRS})

catkin_package()
add_executable(hello_gpu src/hello_gpu.cpp)
target_link_libraries(hello_gpu ${OpenCL_LIBRARIES} ${catkin_LIBRARIES})
