cmake_minimum_required(VERSION 3.5)
project(rgbd-pub)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
add_compile_options(-g)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs image_transport cv_bridge)
find_package(OpenCV 4 QUIET COMPONENTS imgproc highgui)
find_package(depthai CONFIG REQUIRED)


catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs image_transport cv_bridge
  DEPENDS OpenCV
)


include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(frame_publisher src/nodes/frame_publisher.cpp)
target_link_libraries(frame_publisher ${catkin_LIBRARIES} depthai::core depthai::opencv opencv_imgproc)


install(TARGETS frame_publisher
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)