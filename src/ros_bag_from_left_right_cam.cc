/*******************************************************************************
 *   Copyright (C) 2023 CASIA. All rights reserved.
 *
 *   @Filename: ros_bag_from_left_right_cam.cc
 *
 *   @Author: shun li
 *
 *   @Email: shun.li.at.casia@outlook.com
 *
 *   @Date: 05/02/2024
 *
 *   @Description:
 *
 *******************************************************************************/

#include "utility_tool/cmdline.h"
#include "utility_tool/system_lib.h"
#include "utility_tool/print_ctrl_macro.h"
#include "utility_tool/pcm_debug_helper.h"

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <string>

#include <dirent.h>

int main(int argc, char **argv) {
  cmdline::parser par;
  par.add<std::string>("data_path", 0,
                       "the left and right images' path, which should contains "
                       "{left} & {right} sub-directory",
                       true);
  par.add<std::string>("rosbag_name", 0, "created ros bag name", true);
  par.add<int>("rotate_deg", 0, "rotate", false, 0);
  par.add<int>("down_factor", 0, "the down sample factor of the image", false,
               1);

  par.parse_check(argc, argv);

  // STEP: 0 check the amount of the left and right images.
  std::string data_path = par.get<std::string>("data_path");
  std::string left_dir = data_path + "/left";
  std::string right_dir = data_path + "/right";

  std::vector<std::string> l_imgs_name, r_imgs_name;
  utility_tool::GetFilesInDirectory(left_dir, &l_imgs_name);
  utility_tool::GetFilesInDirectory(right_dir, &r_imgs_name);

  assert(l_imgs_name.size() == r_imgs_name.size());
  const int down_f = par.get<int>("down_factor");
  PCM_PRINT_INFO("Downsample size is %d\n", down_f);

  ros::init(argc, argv,
            "rosbag_from_lr_cam_" + utility_tool::GetCurLocalTimeStr());
  ros::NodeHandle nh("~");

  // STEP: 1 read the image use opencv and wirte to the ros bag
  rosbag::Bag ros_bag;
  std::string ros_bag_name = "down_factor_" + std::to_string(down_f) + "_" +
                             par.get<std::string>("rosbag_name");
  ros_bag.open(ros_bag_name, rosbag::bagmode::Write);

  const int rotate = par.get<int>("rotate_deg");
  PCM_PRINT_INFO("rotate is %d\n", rotate);

  ros::Time beg_t = ros::Time::now();
  for (size_t i = 0; i < l_imgs_name.size() && ros::ok(); ++i) {
    beg_t = beg_t + ros::Duration(1);

    // left image
    PCM_PRINT_DEBUG("No. %zu left image name is %s\n", i,
                    l_imgs_name[i].c_str());
    cv::Mat l_tmp_img = cv::imread(l_imgs_name[i]);
    PCM_PRINT_DEBUG("original image shape [width, height] is [%d, %d]\n",
                    l_tmp_img.cols, l_tmp_img.rows);

    cv::resize(l_tmp_img, l_tmp_img,
               cv::Size(l_tmp_img.cols / down_f, l_tmp_img.rows / down_f));
    if (rotate == 90) {
      cv::rotate(l_tmp_img, l_tmp_img, cv::ROTATE_90_CLOCKWISE);
    }
    std_msgs::Header l_header = std_msgs::Header();
    l_header.stamp = beg_t;
    l_header.frame_id = "left_cam";
    sensor_msgs::ImagePtr left_img_msg =
        cv_bridge::CvImage(l_header, "bgr8", l_tmp_img).toImageMsg();
    ros_bag.write("/left", left_img_msg->header.stamp, left_img_msg);

    // right image
    PCM_PRINT_DEBUG("No. %zu right image name is %s\n", i,
                    r_imgs_name[i].c_str());
    cv::Mat r_tmp_img = cv::imread(r_imgs_name[i]);
    cv::resize(r_tmp_img, r_tmp_img,
               cv::Size(r_tmp_img.cols / down_f, r_tmp_img.rows / down_f));
    if (rotate == 90) {
      cv::rotate(r_tmp_img, r_tmp_img, cv::ROTATE_90_CLOCKWISE);
    }
    std_msgs::Header r_header = std_msgs::Header();
    r_header.stamp = beg_t;
    r_header.frame_id = "right_cam";
    sensor_msgs::ImagePtr right_img_msg =
        cv_bridge::CvImage(r_header, "bgr8", r_tmp_img).toImageMsg();
    ros_bag.write("/right", right_img_msg->header.stamp, right_img_msg);
  }

  return 0;
}
