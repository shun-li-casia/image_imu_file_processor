/*******************************************************************************
 *   Copyright (C) 2023 CASIA. All rights reserved.
 *
 *   @Filename: rosbag_reader.h
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

#include "utility_tool/print_ctrl_macro.h"
#include "utility_tool/pcm_debug_helper.h"

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>

#include <string>
#include <vector>
#include <memory>

namespace image_imu_file_processor {
class RosbagReader {
 public:
  RosbagReader(const std::string bag_name, const std::string imu_topic,
               const std::vector<std::string>& img_topics)
      : bag_name_(bag_name), imu_topic_(imu_topic), img_topics_(img_topics) {
    bag_.open(bag_name_, rosbag::bagmode::Read);
    std::vector<std::string> all_topics;
    all_topics.push_back(imu_topic);
    all_topics.insert(all_topics.end(), img_topics.begin(), img_topics.end());
    for (auto& it : all_topics) {
      PCM_STREAM_INFO("read topic: " << it << std::endl);
    }
    bag_view_ =
        std::make_shared<rosbag::View>(bag_, rosbag::TopicQuery(all_topics));
    clear_view_iter();
  }

  void clear_view_iter() { view_iter_ = bag_view_->begin(); }

  // NOTE(shun li): the tp of diffrient channel images shoule be exactly same!
  void ReadTillNextImage(std::vector<sensor_msgs::Imu>* imus,
                         std::vector<sensor_msgs::Image>* imgs) {
    imus->clear();
    imgs->clear();
    imgs->resize(img_topics_.size());

    // 0: firstly read imu
    // 1: read img
    // 2: secondly read imu
    int cur_state = 0;
    while (view_iter_ != bag_view_->end() && ros::ok()) {
      std::string topic = view_iter_->getTopic();

      // imu
      if (topic == imu_topic_) {
        sensor_msgs::Imu::ConstPtr imu =
            view_iter_->instantiate<sensor_msgs::Imu>();

        if (imu != NULL) {
          if (cur_state == 1) break;
          imus->push_back(*imu);
          PCM_PRINT_DEBUG("imu %u-%u\n", imu->header.stamp.sec,
                          imu->header.stamp.nsec);
        }
      } else {  // imgs
        for (size_t i = 0; i < img_topics_.size(); ++i) {
          if (topic == img_topics_[i]) {
            sensor_msgs::Image::ConstPtr img =
                view_iter_->instantiate<sensor_msgs::Image>();

            if (img != NULL) {
              if (cur_state == 0) cur_state = 1;
              (*imgs)[i] = *img;
              PCM_PRINT_DEBUG("img %u-%u\n", img->header.stamp.sec,
                              img->header.stamp.nsec);
            }
          }
        }
      }

      view_iter_++;
    }

    // if (!imus->empty())
    //   imus->pop_back();  // we only need the imu measure before

    for (size_t i = 0; i < imgs->size(); ++i) {
      if ((*imgs)[i].data.empty()) {
        PCM_PRINT_WARN("miss image in topic: %s\n", img_topics_[i].c_str());
      }
    }
  }

 private:
  const std::string bag_name_{""};
  const std::string imu_topic_;
  const std::vector<std::string> img_topics_;
  rosbag::Bag bag_;
  std::shared_ptr<rosbag::View> bag_view_{nullptr};
  rosbag::View::iterator view_iter_;
};
}  // namespace image_imu_file_processor
