cmake_minimum_required(VERSION 2.8.3)
project(image_imu_file_processor)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -fPIC")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -Wall -g -ggdb -fPIC")

# for pcm
add_definitions(-DPCM_PRINT_LEVEL=5)

# Find catkin macros and libraries if COMPONENTS list like find_package(catkin
# REQUIRED COMPONENTS xyz) is used, also find other catkin packages
find_package(
  catkin
  REQUIRED
  roscpp
  rosbag
  std_msgs
  cv_bridge
  image_algorithm
  sensor_config
  utility_tool)

# System dependencies are found with CMake's conventions
find_package(OpenCV 4 REQUIRED)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(yaml-cpp REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS
  include
  CATKIN_DEPENDS
  roscpp
  std_msgs
  cv_bridge
  rosbag
  image_algorithm
  sensor_config
  utility_tool)

add_executable(test_img_decoder src/test_img_decoder.cc)
target_link_libraries(test_img_decoder ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(test_imu_reader src/test_imu_reader.cc)
target_link_libraries(test_imu_reader ${catkin_LIBRARIES})

add_executable(test_img_imu_align src/test_img_imu_align.cc)
target_link_libraries(test_img_imu_align ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(test_img_imu_align_self src/test_img_imu_align_self.cc)
target_link_libraries(test_img_imu_align_self ${catkin_LIBRARIES}
                      ${OpenCV_LIBS})

add_executable(test_img_align src/test_img_align.cc)
target_link_libraries(test_img_align ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(offline_stereo_rectify src/offline_stereo_rectify.cc)
target_link_libraries(offline_stereo_rectify ${catkin_LIBRARIES} ${OpenCV_LIBS}
                      yaml-cpp)

add_executable(offline_stereo_rectify_dir src/offline_stereo_rectify_dir.cc)
target_link_libraries(offline_stereo_rectify_dir ${catkin_LIBRARIES}
                      ${OpenCV_LIBS} yaml-cpp)

add_executable(offline_stereo_remap src/offline_stereo_remap.cc)
target_link_libraries(offline_stereo_remap ${catkin_LIBRARIES} ${OpenCV_LIBS}
                      yaml-cpp)

add_executable(ros_bag_creater src/ros_bag_creater.cc)
target_link_libraries(ros_bag_creater ${catkin_LIBRARIES} ${OpenCV_LIBS}
                      yaml-cpp)

add_executable(ros_bag_from_left_right_cam src/ros_bag_from_left_right_cam.cc)
target_link_libraries(ros_bag_from_left_right_cam ${catkin_LIBRARIES}
                      ${OpenCV_LIBS} yaml-cpp)

add_executable(test_rosbag_reader src/test_rosbag_reader.cc)
target_link_libraries(test_rosbag_reader ${catkin_LIBRARIES} ${OpenCV_LIBS}
                      yaml-cpp)

add_executable(stereo_optical_match src/stereo_optical_match.cc)
target_link_libraries(stereo_optical_match ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(test_stereo_match src/test_stereo_match.cc)
target_link_libraries(test_stereo_match ${catkin_LIBRARIES} ${OpenCV_LIBS}
                      yaml-cpp)

add_executable(rect_cal_imgs_disp src/rect_cal_imgs_disp.cc)
target_link_libraries(rect_cal_imgs_disp ${catkin_LIBRARIES} ${OpenCV_LIBS}
                      yaml-cpp)

add_executable(resize_image src/resize_image.cc)
target_link_libraries(resize_image ${catkin_LIBRARIES} ${OpenCV_LIBS} yaml-cpp)

add_executable(rosbag_img_interval_check src/tool/rosbag_img_interval_check.cc)
target_link_libraries(rosbag_img_interval_check ${catkin_LIBRARIES})

add_executable(rosbag_imu_modifer src/tool/rosbag_imu_modifer.cc)
target_link_libraries(rosbag_imu_modifer ${catkin_LIBRARIES})
