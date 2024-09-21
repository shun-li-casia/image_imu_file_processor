/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: save_rosimg_to_file.cc
 *
 *   @Author: ShunLi
 *
 *   @Email: 2015097272@qq.com
 *
 *   @Date: 26/04/2024
 *
 *   @Description:
 *
 *******************************************************************************/

#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "utility_tool/pcm_debug_helper.h"
#include "utility_tool/system_lib.h"
#include "utility_tool/cmdline.h"
#include "utility_tool/cmdline_multi.h"

#include <ros/init.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

class ImgSubscriber {
 public:
  ImgSubscriber(const std::string& img_topic, const std::string& path,
                bool show_image = true)
      : img_topic_(img_topic), img_path_(path), show_image_(show_image) {
    img_sub_ = nh_.subscribe(img_topic, 10, &ImgSubscriber::img_callback, this);
  }

 private:
  ros::NodeHandle nh_;
  ros::Subscriber img_sub_;

  std::string img_topic_;
  std::string img_path_;
  bool show_image_;

  void img_callback(const sensor_msgs::Image::ConstPtr& img_msg) {
    cv_bridge::CvImageConstPtr ptr =
        cv_bridge::toCvCopy(img_msg, img_msg->encoding);
    const cv::Mat& rgb = ptr->image;
    if (show_image_) {
      cv::imshow("image: " + img_topic_, rgb);
      cv::waitKey(1);
    }

    std::string cur_t = std::to_string(img_msg->header.stamp.toNSec());
    cv::imwrite(img_path_ + "/" + cur_t + ".png", rgb);
    PCM_PRINT_INFO("image from topic %s received! %s\n", img_topic_.c_str(),
                   (cur_t + ".png").c_str());
  }
};

int main(int argc, char* argv[]) {
  cmdline::parser par;

  par.add<std::string>("image_topics", 'i', "image topic", true);
  par.add("show_image", 's', "show image");
  par.parse_check(argc, argv);
  std::string img_topics = par.get<std::string>("image_topics");
  bool if_show_image = par.exist("show_image");

  std::vector<std::string> topics =
      utility_tool::cmdline_multi::ParseMultiArgs<std::string>(img_topics, ',');
  PCM_STREAM_CONTAINER_DEBUG(topics);

  std::string t = utility_tool::GetCurLocalTimeStr("%Y%m%d%H%M%S");
  ros::init(argc, argv, "save_rosimg_to_file_node_" + t);
  ros::NodeHandle nh;

  std::vector<std::string> paths;
  for (auto& topic : topics) {
    auto path = t + "_" + utility_tool::ReplaceEleInStr(topic, "/", "_");
    if (!utility_tool::ShellMkdir(path)) {
      PCM_PRINT_ERROR("mkdir %s failed!\n", t.c_str());
      exit(-1);
    }
    paths.push_back(path);
  }

  std::vector<ImgSubscriber> sub_list;
  for (size_t i = 0; i < topics.size(); ++i) {
    sub_list.push_back(ImgSubscriber(topics[i], paths[i], if_show_image));
  }

  ros::MultiThreadedSpinner spinner(sub_list.size());
  spinner.spin();

  return 0;
}
