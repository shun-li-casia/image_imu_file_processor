/*******************************************************************************
 *   Copyright (C) 2023 CASIA. All rights reserved.
 *
 *   @Filename: imu_reader.h
 *
 *   @Author: shun li
 *
 *   @Email: shun.li.at.casia@outlook.com
 *
 *   @Date: 14/08/2023
 *
 *   @Description:
 *
 *******************************************************************************/

#ifndef IMU_READER_H_
#define IMU_READER_H_

#include "utility_tool/print_ctrl_macro.h"

#include <cassert>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <sstream>
#include <fstream>

class ImuReader {
 public:
  typedef std::shared_ptr<ImuReader> Ptr;

  struct ImuData {
    uint64_t timestamp_;
    double gyr_x_;
    double gyr_y_;
    double gyr_z_;
    double acc_x_;
    double acc_y_;
    double acc_z_;
  };

  explicit ImuReader(const std::string& imu_file) : file_name_(imu_file) {}

  ~ImuReader() {
    if (inf_.is_open()) inf_.close();
  }

  uint64_t get_cur_line() const { return cur_line_; }

  bool OpenFile() {
    // open here
    if (!inf_.is_open()) inf_.open(file_name_);
    if (!inf_.is_open()) return false;

    cur_line_ = 0;

    inf_.seekg(0, std::ios::end);  // move pointer to the end;
    total_len_ = inf_.tellg();
    inf_.clear();
    inf_.seekg(0, std::ios::beg);  // move pointer to the beginning.
    return true;
  }

  void reset() {
    cur_line_ = 0;
    inf_.clear();
    inf_.seekg(0, std::ios::beg);  // move pointer to the beginning.
  }

  bool JumpLine(const int lines) {
    if (lines < 0) {
      // go backward
      const int target = -lines;

      for (int i = 0; i < target; ++i) {
        if (line_len_map_.find(cur_line_ - 1) != line_len_map_.end()) {
          inf_.seekg(-line_len_map_.at(cur_line_ - 1), std::ios::cur);
        }
        if (cur_line_ == 0) break;
        cur_line_--;
      }

      return true;

    } else {
      // go forward!
      std::string buff;
      for (int i = 0; i < lines; ++i) {
        uint64_t pre = inf_.tellg();
        getline(inf_, buff);
        uint64_t cur = inf_.tellg();
        uint64_t len = cur - pre;
        // NOTE: for example cur_line_ == 1, we need to minus the length of 0th
        // line to get the pointer back!
        line_len_map_[cur_line_] = len;
        cur_line_++;
        if (static_cast<uint64_t>(inf_.tellg()) >= total_len_) {
          inf_.clear();
          PCM_PRINT_INFO(" reatch the end! cur_line: %ld\n", cur_line_);
          return false;
        }
      }

      return true;
    }
  }

  bool ReadOneLine(ImuData* d) {
    std::string buff;

    std::streampos pre = inf_.tellg();
    if (!std::getline(inf_, buff)) return false;
    inf_.seekg(pre);

    std::stringstream ss(buff);
    ss >> d->timestamp_ >> d->gyr_x_ >> d->gyr_y_ >> d->gyr_z_ >> d->acc_x_ >>
        d->acc_y_ >> d->acc_z_;
    return true;
  }

  uint64_t get_inf_pose() { return inf_.tellg(); }

 private:
  std::ifstream inf_;
  uint64_t cur_line_{0}, total_len_{0};
  std::string file_name_;
  std::unordered_map<uint64_t, uint64_t> line_len_map_;
};

inline std::ostream& operator<<(std::ostream& out, ImuReader::ImuData& a) {
  out << a.timestamp_ << " " << a.gyr_x_ << " " << a.gyr_y_ << " " << a.gyr_z_
      << " " << a.acc_x_ << " " << a.acc_y_ << " " << a.acc_z_ << std::endl;

  return out;
}
#endif
