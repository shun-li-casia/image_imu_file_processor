/*******************************************************************************
 *   Copyright (C) 2023 CASIA. All rights reserved.
 *
 *   @Filename: test_rosbag_reader.cc
 *
 *   @Author: shun li
 *
 *   @Email: shun.li.at.casia@outlook.com
 *
 *   @Date: 11/10/2023
 *
 *   @Description:
 *
 *******************************************************************************/

#include "image_imu_file_processor/rosbag_reader.h"
#include "ros/ros.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_rosbag_reader_node");
  ros::NodeHandle nh("~");

  std::string imu_topic = "/imu0";
  std::vector<std::string> img_topics;
  img_topics.push_back("/raw_cam_ch_00");
  img_topics.push_back("/raw_cam_ch_01");
  img_topics.push_back("/rect_cam_ch_00");
  img_topics.push_back("/rect_cam_ch_01");

  std::string rosbag_name = "rect_stereo_imu.bag";

  image_imu_file_processor::RosbagReader rosbag_reader(rosbag_name, imu_topic,
                                                  img_topics);

  std::vector<sensor_msgs::Imu> imus;
  std::vector<sensor_msgs::Image> imgs;
  while (ros::ok()) {
    rosbag_reader.ReadTillNextImage(&imus, &imgs);
    PCM_PRINT_DEBUG("imus.size = %zu\n", imus.size());
    for (auto& i : imus) {
      PCM_PRINT_DEBUG("imus: %u-%u, %lf, %lf, %lf, %lf, %lf, %lf\n",
                      i.header.stamp.sec, i.header.stamp.nsec,
                      i.angular_velocity.x, i.angular_velocity.y,
                      i.angular_velocity.z, i.linear_acceleration.x,
                      i.linear_acceleration.y, i.linear_acceleration.z);
    }

    PCM_PRINT_DEBUG("imgs.size = %zu\n", imgs.size());
    for (auto& i : imgs) {
      PCM_PRINT_DEBUG("img: %u-%u\n", i.header.stamp.sec,
                      i.header.stamp.nsec);
    }
    getchar();
  }
  return 0;
}
