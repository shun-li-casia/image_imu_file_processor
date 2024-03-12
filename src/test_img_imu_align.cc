/*******************************************************************************
 *   Copyright (C) 2023 CASIA. All rights reserved.
 *
 *   @Filename: test_img_imu_align.cc
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

ros::Time Tp2RosT(uint64_t tp) {
  std::string buf = std::to_string(tp);
  std::string s_sec = buf.substr(0, 10), s_mui_sec = buf.substr(10, 6);
  uint32_t sec = std::atoll(s_sec.c_str());
  uint32_t mui_sec = std::atoll(s_mui_sec.c_str());
  return ros::Time(sec, mui_sec * 1e3);
}

void PubImus(const std::vector<ImuReader::ImuData>& imus,
             ros::Publisher* imu_puber) {
  for (auto& i : imus) {
    sensor_msgs::Imu imu_msg;
    imu_msg.header.stamp = Tp2RosT(i.timestamp_);
    imu_msg.header.frame_id = "body";
    imu_msg.angular_velocity.x = i.gyr_x_;
    imu_msg.angular_velocity.y = i.gyr_y_;
    imu_msg.angular_velocity.z = i.gyr_z_;
    imu_msg.linear_acceleration.x = i.acc_x_;
    imu_msg.linear_acceleration.y = i.acc_y_;
    imu_msg.linear_acceleration.z = i.acc_z_;
    imu_puber->publish(imu_msg);
  }
}

void PubImgs(const std::vector<ImgDecoder::Frame>& img_frames,
             const std::vector<ImgNameChecker::BinNameEle>& bin_eles,
             std::vector<ros::Publisher>* img_pubers) {
  // publish
  for (size_t i = 0; i < img_frames.size(); ++i) {
    if (img_frames[i].img_.empty()) continue;
    sensor_msgs::ImagePtr ros_img =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_frames[i].img_)
            .toImageMsg();
    ros_img->header.stamp = Tp2RosT(img_frames[i].timestamp_);
    ros_img->header.frame_id = "cam_ch_" + bin_eles[i].ch;
    (*img_pubers)[i].publish(ros_img);
  }
}

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

  ros::init(argc, argv, "img_imu_alian_node");
  ros::NodeHandle nh("~");
  // imu reader...
  ImuReader imu_reader(imu_path);
  if (!imu_reader.OpenFile()) {
    PCM_PRINT_ERROR("can not open %s!\n", imu_path.c_str());
    return -1;
  } else {
    PCM_PRINT_INFO("open imu file: %s\n", imu_path.c_str());
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

  // preapre the publisher
  std::vector<ros::Publisher> img_pubers;
  for (size_t i = 0; i < bin_eles.size(); ++i) {
    img_pubers.push_back(nh.advertise<sensor_msgs::Image>(
        "/align_img_ch_" + bin_eles[i].ch, 1000));
  }
  ros::Publisher imu_puber = nh.advertise<sensor_msgs::Imu>("/align_imu", 1000);
  ros::Rate loop_rate(25);

  ImgImuAlign align;

  // take the 0th channel as the ref to index all of the measuremets
  uint64_t last_ref_t{0}, last_imu_t{0}, imu_t{0};
  ImgDecoder::Frame ref_f;

  bool bad_frame = false;

  uint64_t align_cnt = -1;
  while (ros::ok()) {
    align_cnt++;
    // find images
    img_decoders[0]->ReadOneFrame(&ref_f);
    if (last_ref_t == ref_f.timestamp_) break;

    std::vector<ImgDecoder::Frame> img_frames;
    img_frames.push_back(ref_f);

    // from 1 to n
    for (size_t i = 1; i < img_decoders.size(); ++i) {
      ImgDecoder::Frame tmp_f;
      uint64_t jumps =
          align.FindNearestImg(ref_f.timestamp_, img_decoders[i], &tmp_f);

      uint64_t diff = align.abs_a_b(ref_f.timestamp_, tmp_f.timestamp_);
      PCM_PRINT_INFO(
          "match ch_0, index: %ld, tp: %ld(%s) <--> ch_%s, index; %ld, tp: "
          "%ld(%s) --> "
          "diff: %ld\n",
          img_decoders[0]->get_cur_frame_index(), ref_f.timestamp_,
          utility_tool::TimeStamp2LocalTime(ref_f.timestamp_).c_str(),
          bin_eles[i].ch.c_str(), img_decoders[i]->get_cur_frame_index(),
          tmp_f.timestamp_,
          utility_tool::TimeStamp2LocalTime(tmp_f.timestamp_).c_str(), diff);

      if (diff > 1 * ONE_FRAME_TIME) {
        PCM_PRINT_WARN(
            "bad align found: channel[00, %s], ref: %ld, tmp: %ld, diff: %ld\n",
            bin_eles[i].ch.c_str(), ref_f.timestamp_, tmp_f.timestamp_, diff);
        PCM_PRINT_WARN("jump to next ref frame.\n");

        bad_frame = true;
        // reverse the file pointer.
        img_decoders[i]->JumpFrame(-jumps);
        break;
      } else {
        bad_frame = false;
        img_frames.push_back(tmp_f);
      }
    }

    if (bad_frame) {
      if (!img_decoders[0]->JumpFrame(1)) break;
      continue;
    }

    std::thread img_thread(PubImgs, img_frames, bin_eles, &img_pubers);
    // the start imu tp

    if (last_ref_t != 0) {
      std::vector<ImuReader::ImuData> imus;
      ImuReader::ImuData d;
      PCM_PRINT_DEBUG("before find left imu, cur_line: %ld\n",
                      imu_reader.get_cur_line());
      align.FindNearestImu(last_ref_t, &imu_reader, &d);
      PCM_PRINT_DEBUG("after find left imu, cur_line: %ld\n",
                      imu_reader.get_cur_line());
      last_imu_t = d.timestamp_;

      // read the next frame and then find the next
      PCM_PRINT_DEBUG("before find right imu, cur_line: %ld\n",
                      imu_reader.get_cur_line());
      align.FindNearestImu(ref_f.timestamp_, &imu_reader, &d);
      PCM_PRINT_DEBUG("after find right imu, cur_line: %ld\n",
                      imu_reader.get_cur_line());
      imu_t = d.timestamp_;

      // get the imu_date between (last_t, t];
      PCM_PRINT_INFO(
          "image timestamp in (%ld(%s), %ld(%s)]\n", last_ref_t,
          utility_tool::TimeStamp2LocalTime(last_ref_t).c_str(),
          ref_f.timestamp_,
          utility_tool::TimeStamp2LocalTime(ref_f.timestamp_).c_str());
      PCM_PRINT_INFO("imu   timestamp in (%ld(%s), %ld(%s)]\n", last_imu_t,
                     utility_tool::TimeStamp2LocalTime(last_imu_t).c_str(),
                     imu_t, utility_tool::TimeStamp2LocalTime(imu_t).c_str());
      PCM_PRINT_INFO("imu and img diff in (%ld, %ld]\n",
                     align.abs_a_b(last_ref_t, last_imu_t),
                     align.abs_a_b(ref_f.timestamp_, imu_t));
      align.GetImuBetween(last_imu_t, imu_t, &imu_reader, &imus);

      PCM_PRINT_DEBUG("imu nums in (%ld(%s), %ld(%s)] is %ld.\n", last_imu_t,
                      utility_tool::TimeStamp2LocalTime(last_imu_t).c_str(),
                      imu_t, utility_tool::TimeStamp2LocalTime(imu_t).c_str(),
                      imus.size());
      for (auto& i : imus) PCM_STREAM_DEBUG(i);
      std::thread imu_thread(PubImus, imus, &imu_puber);
      imu_thread.join();
    }
    img_thread.join();

    // update
    last_imu_t = imu_t;
    last_ref_t = ref_f.timestamp_;
    if (!img_decoders[0]->JumpFrame(1)) break;

    if (PCM_PRINT_LEVEL >= PCM_DEBUG) getchar();

    loop_rate.sleep();
  }

  return 0;
}
