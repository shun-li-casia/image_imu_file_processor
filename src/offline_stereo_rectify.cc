/*******************************************************************************
 *   Copyright (C) 2023 CASIA. All rights reserved.
 *
 *   @Filename: offline_stereo_rectify.cc
 *
 *   @Author: shun li
 *
 *   @Email: shun.li.at.casia@outlook.com
 *
 *   @Date: 09/10/2023
 *
 *   @Description:
 *
 *******************************************************************************/

#include "utility_tool/cmdline.h"
#include "utility_tool/print_ctrl_macro.h"
#include "utility_tool/pcm_debug_helper.h"
#include "image_imu_file_processor/img_imu_align.h"
#include "image_imu_file_processor/img_file_name_checker.h"
#include "sensor_config/modules/stereo_cam_config_manager.h"
#include "sensor_config/modules/stereo_rectifier.h"

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <Eigen/Core>

ros::Time Tp2RosT(uint64_t tp) {
  std::string buf = std::to_string(tp);
  std::string s_sec = buf.substr(0, 10), s_mui_sec = buf.substr(10, 6);
  uint32_t sec = std::atoll(s_sec.c_str());
  uint32_t mui_sec = std::atoll(s_mui_sec.c_str());
  return ros::Time(sec, mui_sec * 1e3);
}

int main(int argc, char** argv) {
  cmdline::parser par;
  par.add<std::string>("imu_path", 0,
                       "the imu file path, emmits it to avoid imu measurements",
                       false, "");

  par.add<std::string>(
      "img_paths", 0,
      "the image bin file path, which should be"
      "cam_ch_00.bin,cam_ch_01.bin,cam_ch_02.bin,cam_ch_03.bin... they will be "
      "treated as {00,01}, {02,03} ...",
      true);

  par.add<std::string>("stereo_calibra_files", 0,
                       "raw stereo calibration files, generated by kalibr. "
                       "They could be multiple files.",
                       true);

  par.add<int>("img_step", 0, "steps to jump over some imags", false, 1);

  par.add<int>("start_frame", 0, "the frame start to decode and rectify", false,
               0);

  par.add<int>("img_num", 0, "total number of images", false, INT_MAX);

  par.add<int>("save_rect_img", 0, "if save the rected images", false, 0);
  par.add<std::string>("rect_img_format", 0,
                       "the format when saving the rect images", false, "png");
  par.add<int>("rect_img_width", 0, "the save image width", true);
  par.add<int>("rect_img_height", 0, "the save image height", true);

  par.add<int>("save_rosbag", 0, "if save as the rect images as rosbag", false,
               0);
  par.add<int>("preview_img", 0, "", false, 0);

  par.parse_check(argc, argv);

  // imu path
  const std::string imu_path = par.get<std::string>("imu_path");
  // imu reader...
  ImuReader imu_reader(imu_path);
  if (!imu_path.empty()) {
    if (!imu_reader.OpenFile()) {
      PCM_PRINT_ERROR("can not open imu file %s!\n", imu_path.c_str());
      return -1;
    } else {
      PCM_PRINT_INFO("open imu file: %s\n", imu_path.c_str());
    }
  }

  // check the img paths
  std::stringstream img_ss(par.get<std::string>("img_paths"));
  std::string tmp_path;
  std::vector<std::string> img_paths;
  std::vector<ImgNameChecker::BinNameEle> bin_eles;
  while (std::getline(img_ss, tmp_path, ',')) {
    ImgNameChecker::BinNameEle ele;
    ImgNameChecker::GetImgBinNameEle(tmp_path, &ele);
    if (ele.tail != "bin") {
      PCM_STREAM_ERROR(tmp_path << ": wrong extension name!" << std::endl);
      return -1;
    }
    if (ele.pre != "cam_ch") {
      PCM_STREAM_ERROR(tmp_path << ": wrong file name!" << std::endl);
      return -1;
    }
    bin_eles.push_back(ele);
    img_paths.push_back(tmp_path);
  }

  // img decoders
  std::vector<ImgDecoder::Ptr> img_decoders;
  for (auto p : img_paths) {
    ImgDecoder::Ptr d = std::make_shared<ImgDecoder>(p);
    img_decoders.push_back(d);
    if (!d->OpenFile()) {
      PCM_PRINT_ERROR("can not open %s!\n", p.c_str());
      return -1;
    } else {
      PCM_PRINT_INFO("open img file: %s\n", p.c_str());
    }
  }

  // read the calibration files
  const std::string stereo_calibra_files =
      par.get<std::string>("stereo_calibra_files");
  std::stringstream calib_ss(stereo_calibra_files);
  std::string tmp_calib;
  std::vector<std::string> calib_files;
  while (std::getline(calib_ss, tmp_calib, ',')) {
    calib_files.push_back(tmp_calib);
  }
  assert(calib_files.size() * 2 == img_paths.size());
  const int num_of_stereos = calib_files.size();

  std::vector<sensor_config::StereoCamConfig> config;
  config.resize(num_of_stereos);
  for (int i = 0; i < num_of_stereos; ++i) {
    sensor_config::StereoCamConfig& c = config[i];
    PCM_PRINT_INFO("read raw stereo calibration from %s \n",
                   calib_files[i].c_str());
    sensor_config::StereoCamConfigManager::ReadKalibr(calib_files[i], &c);
  }

  std::vector<std::pair<cv::Mat, cv::Mat>> stereo_maps;
  for (int i = 0; i < num_of_stereos; ++i) {
    Eigen::Matrix3d rect_r_rl;
    Eigen::Vector3d rect_t_rl;
    sensor_config::PinholeCamera::Parameters& l_cam = config[i].cam_params_[0];
    sensor_config::PinholeCamera::Parameters& r_cam = config[i].cam_params_[1];
    std::pair<cv::Mat, cv::Mat> l_map, r_map;

    // apply the rect
    sensor_config::StereoRectifier::RectStereoParam(
        config[i].r_rl_, config[i].t_rl_, &rect_r_rl, &rect_t_rl, &l_cam,
        &r_cam, &l_map, &r_map);
    config[i].r_rl_ = rect_r_rl;
    config[i].t_rl_ = rect_t_rl;
    stereo_maps.push_back(l_map);
    stereo_maps.push_back(r_map);

    config[i].rostopic_[0] = "/rect_cam_ch_" + bin_eles[2 * i].ch;
    config[i].rostopic_[1] = "/rect_cam_ch_" + bin_eles[2 * i + 1].ch;

    sensor_config::StereoCamConfigManager::WriteKalibr(
        config[i], "rect_stereo_" + std::to_string(i) + ".yaml");
  }

  ros::init(argc, argv, "offline_stereo_rectify_node");
  ros::NodeHandle nh("~");

  bool save_rosbag = par.get<int>("save_rosbag");
  bool if_preview_img = par.get<int>("preview_img");
  rosbag::Bag ros_bag;
  if (save_rosbag) ros_bag.open("rect_stereo_imu.bag", rosbag::bagmode::Write);

  // read write imus
  if (!imu_path.empty() && save_rosbag) {
    while (ros::ok()) {
      ImuReader::ImuData imu;
      imu_reader.ReadOneLine(&imu);
      sensor_msgs::Imu imu_msg;
      imu_msg.header.stamp = Tp2RosT(imu.timestamp_);
      imu_msg.header.frame_id = "body";
      imu_msg.angular_velocity.x = imu.gyr_x_;
      imu_msg.angular_velocity.y = imu.gyr_y_;
      imu_msg.angular_velocity.z = imu.gyr_z_;
      imu_msg.linear_acceleration.x = imu.acc_x_;
      imu_msg.linear_acceleration.y = imu.acc_y_;
      imu_msg.linear_acceleration.z = imu.acc_z_;

      ros_bag.write("/imu0", imu_msg.header.stamp, imu_msg);
      if (!imu_reader.JumpLine(1)) {
        PCM_PRINT_INFO("end of write imu!\n");
        break;
      }
    }
  }

  const int img_step = par.get<int>("img_step");
  const int img_num = par.get<int>("img_num");
  const int start_frame = par.get<int>("start_frame");
  const bool save_rect_img = par.get<int>("save_rect_img");
  const std::string rect_img_format = par.get<std::string>("rect_img_format");
  const int rect_img_width = par.get<int>("rect_img_width");
  const int rect_img_height = par.get<int>("rect_img_height");

  for (auto d : img_decoders) {
    d->JumpFrame(start_frame);
  }

  for (size_t i = 0; i < img_decoders.size(); ++i) {
    std::string img_save_path = "rect_imgs_" + bin_eles[i].ch;
    if (save_rect_img) {
      utility_tool::ShellRm(img_save_path);
      utility_tool::ShellMkdir(img_save_path);
    }
    int img_cnt = 0;

    while (ros::ok() && img_cnt++ < img_num) {
      ImgDecoder::Frame img_frame;
      img_decoders[i]->ReadOneFrame(&img_frame);

      // apply rectify
      cv::Mat rect_img;
      cv::remap(img_frame.img_, rect_img, stereo_maps[i].first,
                stereo_maps[i].second, cv::INTER_LINEAR);

      // show
      if (if_preview_img || img_cnt < 50) {
        cv::imshow("raw img", img_frame.img_);
        cv::imshow("rect img", rect_img);
        cv::waitKey(1);
      }

      if (save_rect_img) {
        char img_index[11];
        snprintf(img_index, sizeof(img_index), "%010d", img_cnt);

        cv::Mat resized_rect_img;
        cv::resize(rect_img, resized_rect_img,
                   cv::Size(rect_img_width, rect_img_height));
        cv::imwrite(img_save_path + "/" + std::string(img_index) + "." +
                        rect_img_format,
                    resized_rect_img);
      }

      if (save_rosbag) {
        // raw image
        std_msgs::Header raw_header = std_msgs::Header();
        raw_header.stamp = Tp2RosT(img_frame.timestamp_);
        raw_header.frame_id = "raw_cam_ch_" + bin_eles[i].ch;
        sensor_msgs::ImagePtr raw_img_msg =
            cv_bridge::CvImage(raw_header, "bgr8", img_frame.img_).toImageMsg();
        ros_bag.write("/" + raw_img_msg->header.frame_id,
                      raw_img_msg->header.stamp, raw_img_msg);

        // rect image
        std_msgs::Header rect_header = std_msgs::Header();
        rect_header.stamp = Tp2RosT(img_frame.timestamp_);
        rect_header.frame_id = "rect_cam_ch_" + bin_eles[i].ch;
        sensor_msgs::ImagePtr rect_img_msg =
            cv_bridge::CvImage(rect_header, "bgr8", rect_img).toImageMsg();
        ros_bag.write("/" + rect_img_msg->header.frame_id,
                      rect_img_msg->header.stamp, rect_img_msg);
      }

      if (!img_decoders[i]->JumpFrame(img_step)) {
        PCM_PRINT_INFO("end of write ch %s!\n", bin_eles[i].ch.c_str());
        break;
      }
    }
  }

  ros_bag.close();

  return 0;
}
