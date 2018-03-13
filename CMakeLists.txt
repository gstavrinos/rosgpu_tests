cmake_minimum_required(VERSION 2.8.3)
project(rosgpu_tests)

add_compile_options(-std=c++11)

find_package(OpenCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OPENCL_INCLUDE_DIR}
)

#add_executable(hello_gpu_world src/hello_gpu_world.cpp)
#target_link_libraries(hello_gpu_world ${catkin_LIBRARIES} ${OPENCL_INCLUDE_DIR})
