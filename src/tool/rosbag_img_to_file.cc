/*******************************************************************************
 *   Copyright (C) 2023 CASIA. All rights reserved.
 *
 *   @Filename: rosbag_img_to_file.cc
 *
 *   @Author: shun li
 *
 *   @Email: shun.li.at.casia@outlook.com
 *
 *   @Date: 18/09/2024
 *
 *   @Description:
 *
 *******************************************************************************/

#include <rosbag/bag.h>
#include <ros/ros.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"

#include <utility_tool/cmdline.h>
#include <utility_tool/cmdline_multi.h>
#include <utility_tool/print_ctrl_macro.h>
#include <utility_tool/pcm_debug_helper.h>
#include <utility_tool/file_writter.h>
#include <utility_tool/system_lib.h>

int main(int argc, char** argv) {
  cmdline::parser par;
  par.add<std::string>("type", 'i', "save as image or video", true);
  par.add<int>("fps", 'f', "the fps of video", true);
  par.add<int>("height", 'h', "the height of image", true);
  par.add<int>("width", 'w', "the width of image", true);
  par.add<std::string>("ex", 'e', "the extension name, png, jpg or mp4~", true);
  par.add<std::string>("topics", 't', "topic name, use ',' to seperate", true);
  par.add<std::string>("bag", 'b', "ros bag name", true);
  par.parse_check(argc, argv);

  std::string bag_name = par.get<std::string>("bag");
  std::string topics = par.get<std::string>("topics");
  std::string type = par.get<std::string>("type");
  std::string ex = par.get<std::string>("ex");
  int height = par.get<int>("height");
  int width = par.get<int>("width");
  int fps = par.get<int>("fps");

  std::vector<std::string> topic_vec =
      utility_tool::cmdline_multi::ParseMultiArgs<std::string>(topics, ',',
                                                               nullptr);
  PCM_PRINT_DEBUG("topics = \n");
  for (auto topic : topic_vec) {
    std::cout << topic << std::endl;
  }

  // create the output
  if (type != "image" && type != "video") {
    PCM_PRINT_ERROR("type should be 'image' or 'video'\n");
    return 0;
  }

  std::string tp = utility_tool::GetCurLocalTimeStr("%Y%m%d_%H%M%S");
  std::vector<std::string> out_paths;
  for (auto& t : topic_vec) {
    std::string out_path =
        tp + "_" + utility_tool::ReplaceEleInStr(t, "/", "_");
    utility_tool::ShellMkdir(out_path);
    out_paths.push_back(out_path);
  }

  rosbag::Bag bag;
  bag.open(bag_name, rosbag::bagmode::Read);
  rosbag::View view(bag, rosbag::TopicQuery(topic_vec));

  auto view_iter = view.begin();

  std::vector<std::shared_ptr<cv::VideoWriter>> video_writers;
  video_writers.resize(topic_vec.size());

  if (type == "video") {
    for (size_t i = 0; i < topic_vec.size(); i++) {
      std::string out_path = out_paths[i];
      video_writers[i] = std::make_shared<cv::VideoWriter>(
          out_path + "/" + tp + "_" +
              utility_tool::ReplaceEleInStr(topic_vec[i], "/", "_") + "." + ex,
          cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps,
          cv::Size(width, height));
    }

    // check if opened!
    for (auto w : video_writers) {
      if (!w->isOpened()) {
        PCM_PRINT_ERROR("open video writer failed!\n");
        return 0;
      }
    }
  }

  while (view_iter != view.end()) {
    std::string topic = view_iter->getTopic();

    for (size_t i = 0; i < topic_vec.size(); i++) {
      if (topic == topic_vec[i]) {
        if (type == "image") {
          sensor_msgs::Image::ConstPtr img =
              view_iter->instantiate<sensor_msgs::Image>();
          if (img != nullptr) {
            cv_bridge::CvImagePtr cv_ptr;
            try {
              cv_ptr =
                  cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
            } catch (cv_bridge::Exception& e) {
              ROS_ERROR("cv_bridge exception: %s", e.what());
              return 0;
            }
            cv::resize(cv_ptr->image, cv_ptr->image, cv::Size(width, height));
            std::string out_path = out_paths[i];
            cv::imwrite(out_path + "/" + std::to_string(img->header.stamp.sec) +
                            "_" + std::to_string(img->header.stamp.nsec) + "." +
                            ex,
                        cv_ptr->image);

            PCM_PRINT_DEBUG("image %u-%u\n", img->header.stamp.sec,
                            img->header.stamp.nsec);
          }
        } else if (type == "video") {
          sensor_msgs::Image::ConstPtr img =
              view_iter->instantiate<sensor_msgs::Image>();
          if (img != nullptr) {
            cv_bridge::CvImagePtr cv_ptr;
            try {
              cv_ptr =
                  cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
            } catch (cv_bridge::Exception& e) {
              ROS_ERROR("cv_bridge exception: %s", e.what());
              return 0;
            }
            cv::resize(cv_ptr->image, cv_ptr->image, cv::Size(width, height));
            // write to the video
            video_writers[i]->write(cv_ptr->image);
            PCM_PRINT_DEBUG("image %u-%u\n", img->header.stamp.sec,
                            img->header.stamp.nsec);
          }
        }
      }
    }
    view_iter++;
  }

  for (auto video_writer : video_writers) {
    if (video_writer != nullptr) {
      video_writer->release();
    }
  }

  return 0;
}
