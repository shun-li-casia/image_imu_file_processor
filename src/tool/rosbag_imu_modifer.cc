/*******************************************************************************
 *   Copyright (C) 2023 CASIA. All rights reserved.
 *
 *   @Filename: rosbag_imu_modifer.cc
 *
 *   @Author: shun li
 *
 *   @Email: shun.li.at.casia@outlook.com
 *
 *   @Date: 18/06/2024
 *
 *   @Description:
 *
 *******************************************************************************/

#include "sensor_msgs/Image.h"
#include "utility_tool/print_ctrl_macro.h"
#include "utility_tool/pcm_debug_helper.h"
#include "utility_tool/cmdline.h"

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <boost/foreach.hpp>

void adjustImuAccelerationAndSave(const std::string& input_bag_path,
                                  const std::string& output_bag_path,
                                  const std::string& imu_topic,
                                  double adjustment_factor) {
  // 打开源bag文件
  rosbag::Bag in_bag(input_bag_path, rosbag::bagmode::Read);

  // 创建目标bag文件
  rosbag::Bag out_bag(output_bag_path, rosbag::bagmode::Write);

  rosbag::View view_all(in_bag);

  // 遍历bag中的每一条消息
  for (const rosbag::ConnectionInfo* ci : view_all.getConnections()) {
    std::string topic = ci->topic;
    if (topic == imu_topic) {
      rosbag::View view_imu(in_bag, rosbag::TopicQuery(topic));
      BOOST_FOREACH (rosbag::MessageInstance const m, view_imu) {
        sensor_msgs::Imu::ConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
        if (imu_msg != nullptr) {
          // 创建一个新的Imu消息实例，用于存储修改后的数据
          sensor_msgs::Imu modified_imu_msg(*imu_msg);

          // 修改加速度值（乘以一个因子作为示例）
          modified_imu_msg.linear_acceleration.x *= adjustment_factor;
          modified_imu_msg.linear_acceleration.y *= adjustment_factor;
          modified_imu_msg.linear_acceleration.z *= adjustment_factor;

          PCM_PRINT_DEBUG(
              "org imu_msg: %u-%u, %lf, %lf, %lf, %lf, %lf, %lf\n",
              imu_msg->header.stamp.sec, imu_msg->header.stamp.nsec,
              imu_msg->angular_velocity.x, imu_msg->angular_velocity.y,
              imu_msg->angular_velocity.z, imu_msg->linear_acceleration.x,
              imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z);
          PCM_PRINT_DEBUG("mod imu_msg: %u-%u, %lf, %lf, %lf, %lf, %lf, %lf\n",
                          modified_imu_msg.header.stamp.sec,
                          modified_imu_msg.header.stamp.nsec,
                          modified_imu_msg.angular_velocity.x,
                          modified_imu_msg.angular_velocity.y,
                          modified_imu_msg.angular_velocity.z,
                          modified_imu_msg.linear_acceleration.x,
                          modified_imu_msg.linear_acceleration.y,
                          modified_imu_msg.linear_acceleration.z);

          // 写入新bag，使用原始topic名称
          out_bag.write(imu_topic, m.getTime(), modified_imu_msg);
        }
      }
    } else {
      rosbag::View view_other(in_bag, rosbag::TopicQuery(topic));
      BOOST_FOREACH (rosbag::MessageInstance const m, view_other) {
        out_bag.write(topic, m.getTime(), m);
      }
    }
  }

  // 关闭bag文件
  in_bag.close();
  out_bag.close();

  PCM_STREAM_INFO("IMU acceleration adjustment completed and saved to: "
                  << output_bag_path << std::endl);
}

int main(int argc, char** argv) {
  cmdline::parser par;
  par.add<std::string>("input_bag_path", 'i', "input bag path", true);
  par.add<std::string>("output_bag_path", 'o', "output bag path", true);
  par.add<std::string>("imu_topic", 't', "imu topic", true);
  par.add<double>("adjustment_factor", 'f', "adjustment factor", true);
  par.parse_check(argc, argv);

  std::string input_bag_path = par.get<std::string>("input_bag_path");
  std::string output_bag_path = par.get<std::string>("output_bag_path");
  std::string imu_topic = par.get<std::string>("imu_topic");
  double adjustment_factor = par.get<double>("adjustment_factor");

  adjustImuAccelerationAndSave(input_bag_path, output_bag_path, imu_topic,
                               adjustment_factor);

  return 0;
}
