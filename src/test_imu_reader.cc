/*******************************************************************************
 *   Copyright (C) 2023 CASIA. All rights reserved.
 *
 *   @Filename: test_imu_reader.cc
 *
 *   @Author: shun li
 *
 *   @Email: shun.li.at.casia@outlook.com
 *
 *   @Date: 14/08/2023
 *
 *   @Description:
 *
 *******************************************************************************/

#include "utility_tool/cmdline.h"
#include "image_imu_file_processor/imu_reader.h"
#include "sensor_msgs/Imu.h"
#include "ros/ros.h"

#include <iostream>

ros::Time Tp2RosT(uint64_t tp) {
  std::string buf = std::to_string(tp);
  std::string s_sec = buf.substr(0, 10), s_mui_sec = buf.substr(10, 6);
  uint32_t sec = std::atoll(s_sec.c_str());
  uint32_t mui_sec = std::atoll(s_mui_sec.c_str());
  return ros::Time(sec, mui_sec * 1e3);
}

int main(int argc, char **argv) {
  cmdline::parser par;
  par.add<std::string>("imu_path", 0, "");
  par.add<int>("pub_rate", 0, "", false, 500);

  par.parse_check(argc, argv);

  std::string path = par.get<std::string>("imu_path");

  // get the file name
  std::string::size_type iPos = path.find_last_of('/') + 1;
  std::string filename = path.substr(iPos, path.length() - iPos);

  std::string pure_filename = filename.substr(0, filename.rfind("."));
  std::string tail = filename.substr(filename.rfind(".") + 1);

  if (tail != "txt") {
    PCM_PRINT_ERROR("only the txt file is acceptable!\n");
    return -1;
  }

  ros::init(argc, argv, "imu_reader_node");
  ros::NodeHandle nh("~");
  ros::Publisher imu_puber = nh.advertise<sensor_msgs::Imu>("/fdi_imu", 1000);
  sensor_msgs::Imu imu_msg;

  ImuReader::ImuData data;
  ImuReader imu_reader(path);
  if (!imu_reader.OpenFile()) {
    PCM_PRINT_ERROR("can not open %s\n!", path.c_str());
  }

  ros::Rate rate(par.get<int>("pub_rate"));

  while (ros::ok() && imu_reader.ReadOneLine(&data)) {
    PCM_STREAM_INFO("------------- reading "
                    << imu_reader.get_cur_line() << "th "
                    << "line -------------" << std::endl);
    PCM_STREAM_DEBUG("timestamp: " << data.timestamp_ << std::endl);
    PCM_STREAM_DEBUG("gyr_x: " << data.gyr_x_ << std::endl);
    PCM_STREAM_DEBUG("gyr_y: " << data.gyr_y_ << std::endl);
    PCM_STREAM_DEBUG("gyr_z: " << data.gyr_z_ << std::endl);
    PCM_STREAM_DEBUG("acc_x: " << data.acc_x_ << std::endl);
    PCM_STREAM_DEBUG("acc_y: " << data.acc_y_ << std::endl);
    PCM_STREAM_DEBUG("acc_z: " << data.acc_z_ << std::endl);

    imu_msg.header.stamp = Tp2RosT(data.timestamp_);
    imu_msg.header.frame_id = "body";
    imu_msg.angular_velocity.x = data.gyr_x_;
    imu_msg.angular_velocity.y = data.gyr_y_;
    imu_msg.angular_velocity.z = data.gyr_z_;
    imu_msg.linear_acceleration.x = data.acc_x_;
    imu_msg.linear_acceleration.y = data.acc_y_;
    imu_msg.linear_acceleration.z = data.gyr_z_;
    imu_puber.publish(imu_msg);

    if (!imu_reader.JumpLine(1)) break;
    rate.sleep();
  }
  return 0;
}
