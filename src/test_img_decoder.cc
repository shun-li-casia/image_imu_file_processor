/*******************************************************************************
 *   Copyright (C) 2023 CASIA. All rights reserved.
 *
 *   @Filename: test_img_decoder.cc
 *
 *   @Author: shun li
 *
 *   @Email: shun.li.at.casia@outlook.com
 *
 *   @Date: 04/08/2023
 *
 *   @Description:
 *
 *******************************************************************************/

#include "image_imu_file_processor/img_decoder.h"
#include "image_imu_file_processor/img_file_name_checker.h"
#include "utility_tool/system_lib.h"
#include "utility_tool/cmdline.h"

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <time.h>

int main(int argc, char **argv) {
  cmdline::parser par;

  par.add<bool>("save_img", 0, "save the decoded images", false, false);
  par.add<bool>("preview_img", 0, "preview decoded images", false, false);
  par.add<bool>("save_video", 0, "save the decoded video", false, false);
  par.add<bool>("save_interval", 0, "save time stamp interval", false, false);
  par.add<bool>("save_tp", 0, "save the frame time stamp", false, false);
  par.add<int>("start_frame", 0, "frames start with", false, 0);
  par.add<std::string>("output_path", 0, "the output path(without last /)",
                       true);
  par.add<std::string>("bin_path", 0, "bin file path", true);
  par.add<int>("step", 0,
               "for each n frames and save 1 frame. set to 1 to save each "
               "frames in the bin.",
               false, 1);
  par.parse_check(argc, argv);

  std::string bin_path = par.get<std::string>("bin_path");

  // get the file name
  ImgNameChecker::BinNameEle ele;
  ImgNameChecker::GetImgBinNameEle(bin_path, &ele);

  time_t cur_time = std::time(NULL);
  char ch_cur_time[64];
  std::strftime(ch_cur_time, sizeof(ch_cur_time), "%Y%m%d%H%M%S",
                std::localtime(&cur_time));
  std::string st_cur_t = ch_cur_time;

  std::string output_dir =
      par.get<std::string>("output_path") + "/ch_" + ele.ch + "_" + st_cur_t;

  bool if_preview_img = par.get<bool>("preview_img");
  bool if_save_img = par.get<bool>("save_img");
  bool if_save_video = par.get<bool>("save_video");
  bool if_save_tp = par.get<bool>("save_tp");
  bool if_save_interval = par.get<bool>("save_interval");
  if (if_save_img || if_save_video) {
    if (!utility_tool::ShellRm(output_dir)) return -1;
    if (!utility_tool::ShellMkdir(output_dir)) return -1;
  } else {
    output_dir = ".";
  }

  if (ele.pre != "cam_ch" || ele.tail != "bin") {
    std::cerr << "wrong cam_ch_xx.bin!" << std::endl;
    return -1;
  }

  int step = par.get<int>("step");

  std::cout << "file path: " << bin_path << std::endl
            << "output path: " << output_dir << std::endl
            << "step: " << step << std::endl
            << "file name: " << ele.fn << std::endl
            << "channel: " << ele.ch << std::endl;

  ros::init(argc, argv, "img_decoder_node_ch_" + ele.ch + "_" + st_cur_t);
  ros::NodeHandle nh("~");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                 ros::console::levels::Debug);

  ImgDecoder decoder(bin_path);
  if (!decoder.OpenFile()) {
    ROS_ERROR("open file failed!");
    return -1;
  } else {
    ROS_INFO("open file: %s", bin_path.c_str());
  }
  ImgDecoder::Frame f;

  std::fstream ts_file, interval_file, tp_err_file, no_tp_err_file;
  if (if_save_tp)
    ts_file.open(output_dir + "/" + ele.fn + "_tp.txt", std::ios::out);
  if (if_save_interval)
    interval_file.open(output_dir + "/" + ele.fn + "_interval.csv",
                       std::ios::out);

  cv::VideoWriter output_video;
  if (if_save_video) {
    output_video.open(output_dir + "/" + ele.fn + ".mp4",
                      cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 25,
                      cv::Size(1920, 1536));
  }

  int frame_cnt = par.get<int>("start_frame");
  decoder.JumpFrame(par.get<int>("start_frame"));

  uint64_t last_tp = 0, last_frame_index = 0;
  constexpr int64_t standard_interval = 40000;

  bool no_tp_err = true;
  while (ros::ok()) {
    decoder.ReadOneFrame(&f);
    std::cout << "------------- decoding " << frame_cnt++ << "th "
              << "frame -------------" << std::endl;
    ROS_DEBUG_STREAM("frame index: " << f.frame_index_);
    ROS_DEBUG_STREAM("frame exposure: " << f.exposure_);
    ROS_DEBUG_STREAM("frame timestamp: " << f.timestamp_);
    std::string local_time = utility_tool::TimeStamp2LocalTime(f.timestamp_);
    ROS_DEBUG_STREAM("local time: " << local_time);

    if (last_tp != 0) {
      int64_t interval = f.timestamp_ - last_tp;
      int64_t frame_interval = f.frame_index_ - last_frame_index;
      PCM_PRINT_DEBUG("current tp interval is: %ld \n", interval);
      PCM_PRINT_DEBUG("current frame interval is: %ld \n", frame_interval);
      float average_interval = static_cast<float>(interval) / frame_interval;
      PCM_PRINT_DEBUG("current frame average interval is: %f \n",
                      average_interval);

      if (interval > 1.5 * standard_interval * step ||
          interval < 0.5 * standard_interval * step) {
        no_tp_err = false;
        PCM_PRINT_ERROR("tp error found with interval: %ld\n", interval);

        if (!tp_err_file.is_open()) {
          tp_err_file.open(output_dir + "/" + ele.fn + "_tp_err.txt",
                           std::ios::out);
        }
        tp_err_file << "--------------ERR--------------" << std::endl;
        tp_err_file << "frame cnt: " << frame_cnt << std::endl;
        tp_err_file << "frame index: " << f.frame_index_ << std::endl;
        tp_err_file << "frame timestamp_: " << f.timestamp_ << std::endl;
        tp_err_file << "tp interval: " << interval << std::endl;
        tp_err_file << "frame interval: " << frame_interval << std::endl;
        tp_err_file << "average interval:  " << average_interval << std::endl;
      }

      if (if_save_interval)
        interval_file << local_time << "," << frame_cnt << "," << f.frame_index_
                      << "," << f.timestamp_ << "," << interval << ","
                      << average_interval << std::endl;
    }

    last_tp = f.timestamp_;
    last_frame_index = f.frame_index_;

    if (if_preview_img) {
      cv::imshow("image", f.img_);
      cv::waitKey(1);
    }

    if (if_save_img) {
      char img_index[10];
      snprintf(img_index, sizeof(img_index), "%06d", frame_cnt);
      cv::imwrite(output_dir + "/" + std::string(img_index) + "_" +
                      std::to_string(f.frame_index_) + "_" +
                      std::to_string(f.timestamp_) + ".png",
                  f.img_);
    }

    if (if_save_video) {
      output_video.write(f.img_);
    }

    if (if_save_tp) {
      ts_file << local_time << "," << frame_cnt << "," << f.frame_index_ << " "
              << f.timestamp_ << std::endl;
    }

    if (!decoder.JumpFrame(step)) break;
  }

  ROS_WARN("end of decoding! %s", bin_path.c_str());
  if (if_save_tp) {
    ts_file.close();
  }
  if (if_save_video) {
    output_video.release();
  }

  if (if_save_interval) {
    interval_file.close();
  }

  if (no_tp_err) {
    no_tp_err_file.open(output_dir + "/" + ele.fn + "_no_tp_err",
                        std::ios::out);
    no_tp_err_file.close();
  } else {
    tp_err_file.close();
  }

  return 0;
}
