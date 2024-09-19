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
  par.add<float>("fps", 'f',
                 "the fps of video, for image, only save the image as fps",
                 true);
  par.add<int>("height", 'h', "the height of image", true);
  par.add<int>("width", 'w', "the width of image", true);
  par.add<std::string>("ex", 'e', "the extension name, png, jpg or mp4~", true);
  par.add<std::string>("topics", 't', "topic name, use ',' to seperate", true);
  par.add<std::string>("bag", 'b', "ros bag name", true);
  par.add<int>("preview", 'p', "if preview th image", true);
  par.parse_check(argc, argv);

  std::string bag_name = par.get<std::string>("bag");
  std::string topics = par.get<std::string>("topics");
  std::string type = par.get<std::string>("type");
  std::string ex = par.get<std::string>("ex");
  int height = par.get<int>("height");
  int width = par.get<int>("width");
  float fps = par.get<float>("fps");
  bool if_preview = par.get<int>("preview") == 1;

  std::vector<std::string> topic_vec =
      utility_tool::cmdline_multi::ParseMultiArgs<std::string>(topics, ',');

  if (topic_vec.empty()) {
    PCM_PRINT_ERROR("topic_vec is empty!\n");
    return 0;
  } else {
    PCM_STREAM_CONTAINER_DEBUG(topic_vec);
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

  std::vector<ros::Time> last_tp_vec;
  last_tp_vec.resize(topic_vec.size(), ros::Time(0, 0));

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

            if (last_tp_vec[i] == ros::Time(0, 0)) {
              last_tp_vec[i] = img->header.stamp;
            } else {
              float tmp_fps =
                  1.0f / (img->header.stamp - last_tp_vec[i]).toSec();
              if (tmp_fps > fps) continue;
              last_tp_vec[i] = img->header.stamp;
            }

            cv::resize(cv_ptr->image, cv_ptr->image, cv::Size(width, height));
            std::string out_path = out_paths[i];
            std::stringstream ss;
            ss << std::setfill('0') << std::setw(6) << img->header.stamp.sec;
            ss << "_" << std::setfill('0') << std::setw(9)
               << img->header.stamp.nsec;

            cv::imwrite(out_path + "/" + ss.str() + "." + ex, cv_ptr->image);

            PCM_PRINT_DEBUG("image %s\n", ss.str().c_str());

            if (if_preview) {
              cv::imshow(topic, cv_ptr->image);
              cv::waitKey(1);
            }
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

            if (if_preview) {
              cv::imshow(topic, cv_ptr->image);
              cv::waitKey(0);
            }
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
