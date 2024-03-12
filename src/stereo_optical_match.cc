/*******************************************************************************
 *   Copyright (C) 2023 CASIA. All rights reserved.
 *
 *   @Filename: stereo_optical_match.cc
 *
 *   @Author: shun li
 *
 *   @Email: shun.li.at.casia@outlook.com
 *
 *   @Date: 24/10/2023
 *
 *   @Description:
 *
 *******************************************************************************/

#include "utility_tool/cmdline.h"
#include "utility_tool/print_ctrl_macro.h"
#include "image_imu_file_processor/img_file_name_checker.h"
#include "image_imu_file_processor/img_decoder.h"
#include "image_algorithm/stereo_optical_matcher.h"

#include <ros/ros.h>

#include <unordered_set>

int main(int argc, char **argv) {
  cmdline::parser par;
  par.add<std::string>("img_paths", 0,
                       "the image bin file path, which should be "
                       "cam_ch_xx.bin,cam_ch_xx.bin..., the first channel will "
                       "be taken as timestamp reference.",
                       true);
  par.add<int>("step", 0, "", false, 1);
  par.add<int>("jump", 0, "", false, 0);

  par.parse_check(argc, argv);

  // check the img path
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

  ros::init(argc, argv, "img_alian_node");
  ros::NodeHandle nh("~");

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

  int i = 0;
  int step = par.get<int>("step");
  int jump = par.get<int>("jump");
  if (jump !=0) {
    for (size_t i = 0; i < img_decoders.size(); ++i) {
      img_decoders[i]->JumpFrame(jump);
    }
  }
  while (ros::ok()) {
    std::vector<ImgDecoder::Frame> frames;
    for (size_t i = 0; i < img_decoders.size(); ++i) {
      ImgDecoder::Frame f;
      img_decoders[i]->ReadOneFrame(&f);
      img_decoders[i]->JumpFrame(step);
      frames.push_back(f);
    }

    assert(frames.size() == 2);
    cv::Mat match_img, filtered_m_img;
    cv::Mat left_img, right_img;
    cv::cvtColor(frames[0].img_, left_img, cv::COLOR_BGR2GRAY);
    cv::cvtColor(frames[1].img_, right_img, cv::COLOR_BGR2GRAY);
    image_algorithm::StereoOpticalMatcher::MatchLR(
        left_img, right_img, 100, 30, &match_img, &filtered_m_img);

    cv::imshow("filtered_m_img", filtered_m_img);
    cv::imwrite(std::to_string(i) + ".png", filtered_m_img);
    cv::waitKey(0);
    ++i;
  }
  return 0;
}
