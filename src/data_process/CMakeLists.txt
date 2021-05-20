cmake_minimum_required(VERSION 3.0.2)
project(data_process)


find_package(catkin REQUIRED COMPONENTS
  pcl_conversions

  pcl_ros
  roscpp
  rospy
  common_func
  std_msgs
)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)

SET(CUDA_NVCC_FLAGS "-arch=aarch64" CACHE STRING "nvcc flags" FORCE)
SET (CUDA_VERBOSE_BUILD ON CACHE BOOL "nvcc verbose" FORCE)



catkin_package(
   INCLUDE_DIRS include
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
)

link_directories(${catkin_LIB_DIRS} lib)  #博客中作者编写的第三方库放在lib文件夹中，即路径agritural_car/src/data_process/lib下


add_executable(data_process src/data_process.cpp src/image_process.cpp)
target_link_libraries(
data_process
${catkin_LIBRARIES}
${PCL_LIBRARIES}
${OpenCV_LIBRARIES}
${CUDA_LIBRARIES}
cudafilter
cudasegmentation)

add_dependencies(data_process data_process_generate_messages_cpp)
