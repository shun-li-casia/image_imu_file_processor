/*******************************************************************************
 *   Copyright (C) 2023 CASIA. All rights reserved.
 *
 *   @Filename: resize_image.cc
 *
 *   @Author: shun li
 *
 *   @Email: shun.li.at.casia@outlook.com
 *
 *   @Date: 21/03/2024
 *
 *   @Description:
 *
 *******************************************************************************/

#include <sys/stat.h>
#include "utility_tool/cmdline.h"
#include "utility_tool/print_ctrl_macro.h"
#include "utility_tool/pcm_debug_helper.h"
#include "utility_tool/system_lib.h"

#include <opencv2/opencv.hpp>

int main(int argc, char **argv) {
  cmdline::parser par;
  par.add<std::string>("data_path", 0, "the images path", true);
  const std::string data_path = par.get<std::string>("data_path");
  std::vector<std::string> imgs_name;
  utility_tool::GetFilesInDirectory(data_path, &imgs_name);

  std::string data_path_resize = data_path + "/resize";
  utility_tool::ShellMkdir(data_path_resize);

  for (size_t i = 0; i < imgs_name.size(); ++i) {
    PCM_PRINT_INFO("resize %s\n", imgs_name[i].c_str());
    cv::Mat img = cv::imread(imgs_name[i]);
    cv::resize(img, img, cv::Size(640, 480));
    cv::imwrite(data_path_resize + "/" + imgs_name[i], img);
  }
  return 0;
}
