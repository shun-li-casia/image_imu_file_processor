/*******************************************************************************
 *   Copyright (C) 2023 CASIA. All rights reserved.
 *
 *   @Filename: offline_stereo_rectify_dir.cc
 *
 *   @Author: shun li
 *
 *   @Email: shun.li.at.casia@outlook.com
 *
 *   @Date: 06/02/2024
 *
 *   @Description:
 *
 *******************************************************************************/

#include "utility_tool/cmdline.h"
#include "utility_tool/system_lib.h"
#include "utility_tool/print_ctrl_macro.h"
#include "utility_tool/pcm_debug_helper.h"
#include "sensor_config/modules/stereo_rectifier.h"
#include "sensor_config/modules/stereo_cam_config_manager.h"

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char** argv) {
  cmdline::parser par;
  // read the image and remap them, then save to the rect_ dir
  par.add<std::string>("data_path", 0,
                       "the left and right images' path, which should contains "
                       "{left} & {right} sub-directory",
                       true);
  par.add<std::string>("stereo_calibra_files", 0,
                       "raw stereo calibration files, generated by kalibr.",
                       true);
  par.add<int>("down_factor", 0,
               "the downsample factor, which will be applied on the intrinsic",
               true);
  par.parse_check(argc, argv);

  // read the calibration files
  const std::string stereo_calibra_files =
      par.get<std::string>("stereo_calibra_files");

  sensor_config::StereoCamConfig config;
  PCM_PRINT_INFO("read raw stereo calibration from %s \n",
                 stereo_calibra_files.c_str());

  const int down_factor = par.get<int>("down_factor");

  // recover the intrinsic from downsample
  sensor_config::StereoCamConfigManager::ReadKalibr(stereo_calibra_files,
                                                    &config, down_factor);

  // apply the rect
  Eigen::Matrix3d rect_r_rl;
  Eigen::Vector3d rect_t_rl;
  sensor_config::PinholeCamera::Parameters& l_cam = config.cam_params_[0];
  sensor_config::PinholeCamera::Parameters& r_cam = config.cam_params_[1];
  std::pair<cv::Mat, cv::Mat> l_map, r_map;

  // apply the rect
  sensor_config::StereoRectifier::RectStereoParam(
      config.r_rl_, config.t_rl_, &rect_r_rl, &rect_t_rl, &l_cam, &r_cam,
      &l_map, &r_map);
  config.r_rl_ = rect_r_rl;
  config.t_rl_ = rect_t_rl;

  PCM_PRINT_DEBUG("the shape of the org map is [%d, %d]\n", l_map.first.cols,
                  l_map.first.rows);
  PCM_PRINT_DEBUG("the shape of the target map is [%d, %d]\n",
                  l_map.second.cols, l_map.second.rows);

  std::string data_path = par.get<std::string>("data_path");

  sensor_config::StereoCamConfigManager::WriteKalibr(
      config, data_path + "/rect_" + stereo_calibra_files);

  std::string left_dir = data_path + "/left";
  std::string right_dir = data_path + "/right";

  std::string rect_left_dir = data_path + "/rect_left";
  utility_tool::ShellMkdir(rect_left_dir);

  std::string rect_right_dir = data_path + "/rect_right";
  utility_tool::ShellMkdir(rect_right_dir);

  std::vector<std::string> l_imgs_name, r_imgs_name;
  utility_tool::GetFilesInDirectory(left_dir, &l_imgs_name);
  utility_tool::GetFilesInDirectory(right_dir, &r_imgs_name);

  assert(l_imgs_name.size() == r_imgs_name.size());

  for (size_t i = 0; i < l_imgs_name.size(); ++i) {
    std::vector<int> compress_param;
    compress_param.push_back(cv::IMWRITE_JPEG_QUALITY);
    compress_param.push_back(100);
    // left image
    PCM_PRINT_DEBUG("No. %zu left image name is %s\n", i,
                    l_imgs_name[i].c_str());
    cv::Mat l_tmp_img = cv::imread(l_imgs_name[i]);
    PCM_PRINT_DEBUG("original image shape [width, height] is [%d, %d]\n",
                    l_tmp_img.cols, l_tmp_img.rows);

    cv::Mat l_rect_img;
    cv::remap(l_tmp_img, l_rect_img, l_map.first, l_map.second,
              cv::INTER_LINEAR);
    char l_img_index[11];
    snprintf(l_img_index, sizeof(l_img_index), "%04zu", i);
    cv::imwrite(rect_left_dir + "/l_" + std::string(l_img_index) + ".jpg",
                l_rect_img, compress_param);

    // right image
    PCM_PRINT_DEBUG("No. %zu right image name is %s\n", i,
                    r_imgs_name[i].c_str());
    cv::Mat r_tmp_img = cv::imread(r_imgs_name[i]);
    cv::Mat r_rect_img;
    cv::remap(r_tmp_img, r_rect_img, r_map.first, r_map.second,
              cv::INTER_LINEAR);
    char r_img_index[11];
    snprintf(r_img_index, sizeof(r_img_index), "%04zu", i);
    cv::imwrite(rect_right_dir + "/r_" + std::string(r_img_index) + ".jpg",
                r_rect_img, compress_param);
  }

  return 0;
}
