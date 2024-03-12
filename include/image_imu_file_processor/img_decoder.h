/*******************************************************************************
 *   Copyright (C) 2023 CASIA. All rights reserved.
 *
 *   @Filename: img_decoder.h
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

#ifndef IMG_DECODER_H_
#define IMG_DECODER_H_

#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <memory>

#include <opencv2/opencv.hpp>
#include "utility_tool/print_ctrl_macro.h"

constexpr uint64_t IMG_WIDTH = 1920, IMG_HEIGHT = 1536;
constexpr uint64_t HEADER_LEN = 1068, IMAGE_LEN = IMG_WIDTH * IMG_HEIGHT * 2;
constexpr uint64_t ONE_FRAME_LEN = HEADER_LEN + IMAGE_LEN;
constexpr uint64_t FRAME_INDEX_BEGIN = 0, FRAME_INDEX_LEN = 4;
constexpr uint64_t EXP_BEGIN = 14, EXP_LEN = 4;
constexpr uint64_t TP_BEGIN = 34, TP_LEN = 8;
constexpr uint64_t IMAGE_BEIGN = 1068;

class ImgDecoder {
 public:
  struct Frame {
    uint32_t frame_index_{0};
    uint64_t timestamp_{0};
    float exposure_{0.0f};
    cv::Mat img_;
  };

  typedef std::shared_ptr<ImgDecoder> Ptr;

  explicit ImgDecoder(const std::string& file_name) : file_name_(file_name) {}

  ~ImgDecoder() {
    if (inf_.is_open()) inf_.close();
  }

  bool OpenFile() {
    // open here
    if (!inf_.is_open()) inf_.open(file_name_, std::ifstream::binary);
    if (!inf_.is_open()) return false;

    inf_.seekg(0, std::ios::end);  // move pointer to the end;
    total_len_ = inf_.tellg();
    inf_.clear();
    inf_.seekg(0, std::ios::beg);  // move pointer to the beginning.
    return true;
  }

  uint64_t get_cur_pose() const { return cur_pose_; }
  uint64_t get_cur_frame_index() const { return cur_pose_ / ONE_FRAME_LEN; }

  bool JumpFrame(const int steps) {
    // move the pointer
    cur_pose_ += steps * ONE_FRAME_LEN;
    inf_.seekg(cur_pose_, std::ios::beg);

    if (cur_pose_ < 0) {
      cur_pose_ = 0;
      return false;
    }
    if (inf_.eof() || cur_pose_ >= total_len_) {
      PCM_PRINT_WARN("reach the img.bin end! cur_pose: %ld, total_len: %ld\n",
                     cur_pose_, total_len_);
      cur_pose_ = total_len_;
      return false;
    }
    return true;
  }

  void reset() {
    cur_pose_ = 0;
    inf_.clear();
    inf_.seekg(0, std::ios::beg);  // move pointer to the beginning.
  }

  void ReadOneFrame(Frame* f) {
    unsigned char frame[ONE_FRAME_LEN + 1];
    inf_.read(reinterpret_cast<char*>(frame), ONE_FRAME_LEN);
    inf_.seekg(ONE_FRAME_LEN, std::ios::cur);  // reverse the pointer.

    // print_uchar_hex(frame, 20);

    // frame index
    memcpy(&f->frame_index_, frame + FRAME_INDEX_BEGIN, FRAME_INDEX_LEN);

    // frame index
    memcpy(&f->exposure_, frame + EXP_BEGIN, EXP_LEN);

    // time stamp
    memcpy(&f->timestamp_, frame + TP_BEGIN, TP_LEN);

    // image
    cv::Mat img(IMG_HEIGHT, IMG_WIDTH, CV_8UC2, frame + IMAGE_BEIGN);
    cv::cvtColor(img, f->img_, cv::COLOR_YUV2BGR_YUYV);
  }

 private:
  // file pointer here.
  uint64_t cur_pose_{0}, total_len_{0};
  const std::string file_name_;

  std::ifstream inf_;

  void print_uchar_hex(const unsigned char* data, const int len) const {
    for (int i = 0; i < len; ++i) {
      std::cout << std::hex << std::setw(2) << std::setfill('0')
                << static_cast<int>(*(data + i)) << " ";
      if ((i + 1) % 15 == 0) std::cout << std::endl;
    }

    getchar();
  }
};

#endif
