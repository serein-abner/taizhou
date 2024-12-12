cmake_minimum_required(VERSION 2.8.3)
project(taizou_path_pro)

# set(CMAKE_BUILD_TYPE_DEBUG)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  cv_bridge
  image_transport
  # geometry_msgs
)
find_package(OpenCV REQUIRED)
find_package(PCL 1.10 REQUIRED)
find_package(OpenMP)

catkin_package(
  CATKIN_DEPENDS 
  roscpp 
  # geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_definitions(
  ${PCL_DEFINITIONS}
)

# 添加库文件目录
add_library(path_library STATIC src/dijkstra.cpp)



# add_executable(path src/open_lslidar.cpp)
add_executable(taizou_path_pro src/taizou_path.cpp)

# target_link_libraries(taizou_path_pro ${catkin_LIBRARIES} /usr/local/lib/libyaml-cpp.a)
target_link_libraries(taizou_path_pro ${catkin_LIBRARIES} /usr/lib/x86_64-linux-gnu/libyaml-cpp.a)




# target_link_libraries(path
#   PUBLIC
#   CSF
#   OpenMP::OpenMP_CXX
#   path_library
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
#   ${OpenCV_LIBRARIES}
# )

target_link_libraries(taizou_path_pro
  PUBLIC
  CSF
  OpenMP::OpenMP_CXX
  path_library
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
