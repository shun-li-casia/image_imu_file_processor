/*******************************************************************************
 *   Copyright (C) 2023 CASIA. All rights reserved.
 *
 *   @Filename: test_img_imu_align_self.cc
 *
 *   @Author: shun li
 *
 *   @Email: shun.li.at.casia@outlook.com
 *
 *   @Date: 17/08/2023
 *
 *   @Description:
 *
 *******************************************************************************/

#include "utility_tool/cmdline.h"
#include "utility_tool/print_ctrl_macro.h"
#include "image_imu_file_processor/img_imu_align.h"
#include "image_imu_file_processor/img_file_name_checker.h"
#include "utility_tool/system_lib.h"

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <sstream>
#include <unordered_set>
#include <thread>

int main(int argc, char** argv) {
  cmdline::parser par;
  par.add<std::string>("imu_path", 0,
                       "the imu txt file path, which should be txt", true);
  par.add<std::string>("img_paths", 0,
                       "the image bin file path, which should be "
                       "cam_ch_xx.bin,cam_ch_xx.bin..., the first channel will "
                       "be taken as timestamp reference.",
                       true);

  par.parse_check(argc, argv);

  const std::string imu_path = par.get<std::string>("imu_path");

  // check the img path
  std::stringstream img_ss(par.get<std::string>("img_paths"));
  std::string tmp_path;
  std::unordered_set<std::string> img_chs;
  std::vector<std::string> img_paths;
  std::vector<ImgNameChecker::BinNameEle> bin_eles;
  while (std::getline(img_ss, tmp_path, ',')) {
    img_paths.push_back(tmp_path);
  }

  ros::init(argc, argv, "img_imu_alian_node");
  ros::NodeHandle nh("~");

  ImgImuAlign align(imu_path, img_paths);

  ImgImuAlign::DataFrame data;

  while (ros::ok()) {
    PCM_PRINT_DEBUG("-------------------align----------------------\n");
    align.ReadOneFrame(&data);
    if (data.imus_.size() < 19 || data.imus_.size() > 21) getchar();
  }

  return 0;
}
