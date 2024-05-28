/*******************************************************************************
 *   Copyright (C) 2023 CASIA. All rights reserved.
 *
 *   @Filename: rosbag_img_interval_check.cc
 *
 *   @Author: shun li
 *
 *   @Email: shun.li.at.casia@outlook.com
 *
 *   @Date: 28/05/2024
 *
 *   @Description:
 *
 *******************************************************************************/

#include "utility_tool/print_ctrl_macro.h"
#include "utility_tool/pcm_debug_helper.h"
#include "utility_tool/cmdline.h"
#include "utility_tool/file_writter.h"

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>

int main(int argc, char** argv) {
  cmdline::parser par;
  par.add<std::string>("bag_name", 'b', "rosbag name", true);
  par.add<std::string>("img_topic", 't', "image topic", true);
  par.parse_check(argc, argv);

  ros::init(argc, argv, "rosbag_img_interval_check_node");
  ros::NodeHandle nh("~");

  const std::string bag_name = par.get<std::string>("bag_name");
  const std::string img_topic = par.get<std::string>("img_topic");

  rosbag::Bag bag;
  bag.open(bag_name, rosbag::bagmode::Read);

  std::shared_ptr<rosbag::View> bag_view =
      std::make_shared<rosbag::View>(bag, rosbag::TopicQuery(img_topic));

  rosbag::View::iterator view_iter = bag_view->begin();

  int img_num = 0;
  ros::Time last_time;
  utility_tool::FileWritter fw("rosbag_img_interval_check.log", 8);
  fw.EraseOpen();

  while (view_iter != bag_view->end() && ros::ok()) {
    std::string topic = view_iter->getTopic();
    if (topic == img_topic) {
      sensor_msgs::Image::ConstPtr img =
          view_iter->instantiate<sensor_msgs::Image>();
      if (img != NULL) {
        PCM_PRINT_DEBUG("img %d, tp: %u-%u\n", img_num++, img->header.stamp.sec,
                        img->header.stamp.nsec);
        if (img_num > 1) {
          float interval = (img->header.stamp - last_time).toNSec() / 1e6f;
          PCM_PRINT_DEBUG("interval: %lf ms\n", interval);
          fw.Write(interval);
        }
        last_time = img->header.stamp;
      }
    }
    view_iter++;
  }

  fw.Close();
  return 0;
}
