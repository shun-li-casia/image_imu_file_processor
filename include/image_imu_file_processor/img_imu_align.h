/*******************************************************************************
 *   Copyright (C) 2023 CASIA. All rights reserved.
 *
 *   @Filename: img_imu_align.h
 *
 *   @Author: shun li
 *
 *   @Email: shun.li.at.casia@outlook.com
 *
 *   @Date: 11/08/2023
 *
 *   @Description:
 *
 *******************************************************************************/

#ifndef IMG_IMU_ALIGN_H_
#define IMG_IMU_ALIGN_H_

#include "image_imu_file_processor/imu_reader.h"
#include "image_imu_file_processor/img_file_name_checker.h"
#include "image_imu_file_processor/img_decoder.h"
#include "utility_tool/system_lib.h"
#include "utility_tool/print_ctrl_macro.h"
#include "utility_tool/system_lib.h"

#include <string>
#include <vector>
#include <fstream>
#include <cmath>
#include <memory>

constexpr uint64_t ONE_FRAME_TIME = 1.0f / 25.0f * 1e6;

class ImgImuAlign {
 public:
  struct DataFrame {
    std::vector<ImuReader::ImuData> imus_;
    std::vector<ImgDecoder::Frame> imgs_;
  };

  ImgImuAlign() {}

  ImgImuAlign(const std::string imu_path,
              const std::vector<std::string>& img_paths) {
    // STEP: 1 process the img paths
    for (auto& tmp_path : img_paths) {
      ImgNameChecker::BinNameEle ele;
      ImgNameChecker::GetImgBinNameEle(tmp_path, &ele);
      if (ele.tail != "bin") {
        PCM_STREAM_ERROR(tmp_path << ": wrong extension name!" << std::endl);
        return;
      }
      if (ele.pre != "cam_ch") {
        PCM_STREAM_ERROR(tmp_path << ": wrong file name!" << std::endl);
        return;
      }
      bin_eles_.push_back(ele);
    }

    // STEP: 2 init imu reader...
    imu_reader_ = std::make_shared<ImuReader>(imu_path);
    if (!imu_reader_->OpenFile()) {
      PCM_PRINT_ERROR("can not open %s!\n", imu_path.c_str());
      return;
    } else {
      PCM_PRINT_INFO("open imu file: %s\n", imu_path.c_str());
    }

    // STEP: 3 img decoders
    for (auto p : img_paths) {
      ImgDecoder::Ptr d = std::make_shared<ImgDecoder>(p);
      img_decoders_.push_back(d);
      if (!d->OpenFile()) {
        PCM_PRINT_ERROR("can not open %s!\n", p.c_str());
        return;
      } else {
        PCM_PRINT_INFO("open img file: %s\n", p.c_str());
      }
    }
  }

  bool ReadOneFrame(DataFrame* data_frame) {
    bool proper_initialized =
        (imu_reader_ != nullptr) && (!img_decoders_.empty());
    if (!proper_initialized) {
      PCM_PRINT_ERROR(
          "the ImgImuAlign instance was not initialized properly!\n");
      return false;
    }

    data_frame->imgs_.clear();
    data_frame->imus_.clear();

    // find images
    img_decoders_[0]->ReadOneFrame(&ref_f_);
    if (last_ref_t_ == ref_f_.timestamp_) return false;

    data_frame->imgs_.push_back(ref_f_);

    // from 1 to n
    for (size_t i = 1; i < img_decoders_.size(); ++i) {
      ImgDecoder::Frame tmp_f;
      uint64_t jumps =
          FindNearestImg(ref_f_.timestamp_, img_decoders_[i], &tmp_f);

      uint64_t diff = abs_a_b(ref_f_.timestamp_, tmp_f.timestamp_);
      PCM_PRINT_INFO(
          "match ch_0, index: %ld, tp: %ld(%s) <--> ch_%s, index; %ld, tp: "
          "%ld(%s) --> "
          "diff: %ld\n",
          img_decoders_[0]->get_cur_frame_index(), ref_f_.timestamp_,
          utility_tool::TimeStamp2LocalTime(ref_f_.timestamp_).c_str(),
          bin_eles_[i].ch.c_str(), img_decoders_[i]->get_cur_frame_index(),
          tmp_f.timestamp_,
          utility_tool::TimeStamp2LocalTime(tmp_f.timestamp_).c_str(), diff);

      if (diff > 1 * ONE_FRAME_TIME) {
        PCM_PRINT_WARN(
            "bad align found: channel[00, %s], ref: %ld, tmp: %ld, diff: %ld\n",
            bin_eles_[i].ch.c_str(), ref_f_.timestamp_, tmp_f.timestamp_, diff);
        PCM_PRINT_WARN("add empty img to ch %s.\n", bin_eles_[i].ch.c_str());

        bad_frame_ = true;
        // reverse the file pointer.
        img_decoders_[i]->JumpFrame(-jumps);
        data_frame->imgs_.push_back(ImgDecoder::Frame());
        break;
      } else {
        bad_frame_ = false;
        data_frame->imgs_.push_back(tmp_f);
      }
    }

    if (last_ref_t_ != 0) {
      ImuReader::ImuData d;
      FindNearestImu(last_ref_t_, imu_reader_.get(), &d);
      last_imu_t_ = d.timestamp_;

      // read the next frame and then find the next
      FindNearestImu(ref_f_.timestamp_, imu_reader_.get(), &d);
      imu_t_ = d.timestamp_;

      // get the imu_date between (last_t, t];
      uint64_t img_interval = ref_f_.timestamp_ - last_ref_t_;
      PCM_PRINT_INFO(
          "image timestamp in (%ld(%s), %ld(%s)], diff: %zu\n", last_ref_t_,
          utility_tool::TimeStamp2LocalTime(last_ref_t_).c_str(),
          ref_f_.timestamp_,
          utility_tool::TimeStamp2LocalTime(ref_f_.timestamp_).c_str(),
          img_interval);

      uint64_t imu_interval = imu_t_ - last_imu_t_;
      PCM_PRINT_INFO(
          "imu   timestamp in (%ld(%s), %ld(%s)], diff: %zu\n", last_imu_t_,
          utility_tool::TimeStamp2LocalTime(last_imu_t_).c_str(), imu_t_,
          utility_tool::TimeStamp2LocalTime(imu_t_).c_str(), imu_interval);

      PCM_PRINT_INFO("imu and img diff in (%ld, %ld]\n",
                     abs_a_b(last_ref_t_, last_imu_t_),
                     abs_a_b(ref_f_.timestamp_, imu_t_));
      GetImuBetween(last_imu_t_, imu_t_, imu_reader_.get(), &data_frame->imus_);

      PCM_PRINT_INFO("imu nums in (%ld(%s), %ld(%s)] is %ld.\n", last_imu_t_,
                     utility_tool::TimeStamp2LocalTime(last_imu_t_).c_str(),
                     imu_t_, utility_tool::TimeStamp2LocalTime(imu_t_).c_str(),
                     data_frame->imus_.size());
    }

    // update
    last_imu_t_ = imu_t_;
    last_ref_t_ = ref_f_.timestamp_;
    if (!img_decoders_[0]->JumpFrame(1)) return false;

    return true;
  }

  /**
   * @brief get the imu in (ref_imu_start, ref_imu_tp_end], will move the imu
   * file pointer to the "ref_imu_tp_end".
   * @return jumps
   */
  int GetImuBetween(const uint64_t ref_imu_tp_start,
                    const uint64_t ref_imu_tp_end, ImuReader* imu_reader,
                    std::vector<ImuReader::ImuData>* imus) {
    int jump = 0;
    ImuReader::ImuData d;
    imu_reader->ReadOneLine(&d);
    while (d.timestamp_ <= ref_imu_tp_start) {
      imu_reader->JumpLine(1);
      jump++;
      imu_reader->ReadOneLine(&d);
    }

    {
      ImuReader::ImuData tmp_d;
      imu_reader->ReadOneLine(&tmp_d);
    }
    // now it is the first bigger than the start.

    while (d.timestamp_ <= ref_imu_tp_end) {
      (*imus).push_back(d);
      imu_reader->JumpLine(1);
      jump++;
      imu_reader->ReadOneLine(&d);
    }

    // NOTE: make sure the left bound
    imu_reader->JumpLine(-1);

    return jump;
  }

  /**
   * @brief from current pos to find the nearst imu without changing the
   * imu_reader file pointer.
   */
  void FindNearestImu(const uint64_t ref_tp, ImuReader* imu_reader,
                      ImuReader::ImuData* d) {
    int jump = 0;
    uint64_t min_diff = UINT64_MAX, last_min_diff = UINT64_MAX;
    ImuReader::ImuData tmp_d, last_tmp_d;

    // NOTE: from current position to minize the finding complexity.
    imu_reader->ReadOneLine(&tmp_d);
    min_diff = abs_a_b(ref_tp, tmp_d.timestamp_);
    while (min_diff < last_min_diff) {
      last_min_diff = min_diff;
      last_tmp_d = tmp_d;
      bool res = imu_reader->JumpLine(1);
      jump++;

      if (!res) {
        break;  // keep decresing.
      }
      imu_reader->ReadOneLine(&tmp_d);
      min_diff = abs_a_b(ref_tp, tmp_d.timestamp_);
    }

    *d = last_tmp_d;

    imu_reader->JumpLine(-jump);
  }

  /**
   * @brief forwardly find the nearest time stamp to ref_tp from the current pos
   * in image decoder. It will move the img_decoder file pointer to the nearest
   * frame.
   *
   * @param[in] ref_tp
   * @param[in & out] img_decoder
   * @param[out] f
   * @return return jumps
   */
  uint64_t FindNearestImg(const uint64_t ref_tp, ImgDecoder::Ptr img_decoder,
                          ImgDecoder::Frame* f) {
    int jump = 0;
    uint64_t min_diff = UINT64_MAX, last_min_diff = UINT64_MAX;
    ImgDecoder::Frame tmp_f, last_tmp_f;

    // NOTE: from current position to minize the finding complexity.
    img_decoder->ReadOneFrame(&tmp_f);
    min_diff = abs_a_b(ref_tp, tmp_f.timestamp_);
    while (min_diff < last_min_diff) {
      last_min_diff = min_diff;
      last_tmp_f = tmp_f;
      bool res = img_decoder->JumpFrame(1);
      ++jump;
      if (!res) return --jump;  // keep decresing.
      img_decoder->ReadOneFrame(&tmp_f);
      min_diff = abs_a_b(ref_tp, tmp_f.timestamp_);
    }

    *f = last_tmp_f;
    // prev one is the mini one!
    img_decoder->JumpFrame(-1);
    return --jump;
  }

  template <typename T>
  T abs_a_b(const T& a, const T& b) const {
    return a >= b ? a - b : b - a;
  }

  ImuReader::Ptr GetImuReader() { return imu_reader_; }

  std::vector<ImgDecoder::Ptr> GetImgDecoders() { return img_decoders_; }

  std::vector<ImgNameChecker::BinNameEle> GetImgsInfo() { return bin_eles_; }

 private:
  ImuReader::Ptr imu_reader_{nullptr};
  std::vector<ImgDecoder::Ptr> img_decoders_;
  std::vector<ImgNameChecker::BinNameEle> bin_eles_;
  bool bad_frame_{false};

  uint64_t last_ref_t_{0}, last_imu_t_{0}, imu_t_{0};
  ImgDecoder::Frame ref_f_;
};

#endif
