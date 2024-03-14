/*******************************************************************************
 *   Copyright (C) 2023 CASIA. All rights reserved.
 *
 *   @Filename: test_stereo_match.cc
 *
 *   @Author: shun li
 *
 *   @Email: shun.li.at.casia@outlook.com
 *
 *   @Date: 07/02/2024
 *
 *   @Description:
 *
 *******************************************************************************/

#include "image_algorithm/disparity_calculator.h"
#include "utility_tool/system_lib.h"
#include "utility_tool/cmdline.h"

int main(int argc, char **argv) {
  cmdline::parser par;
  // read the image and remap them, then save to the rect_ dir
  par.add<std::string>("data_path", 0,
                       "the left and right images' path, which should contains "
                       "{left} & {right} sub-directory",
                       true);
  par.add<std::string>("sbgm_cfg", 0, "the sbgm config file", true);
  par.add<int>("patch_x", 0, "the patch start x position", false, -1);
  par.add<int>("patch_y", 0, "the patch start y position", false, -1);
  par.add<int>("patch_width", 0, "the patch width", false, -1);
  par.add<int>("patch_height", 0, "the patch height", false, -1);

  par.parse_check(argc, argv);

  std::string data_path = par.get<std::string>("data_path");
  std::string rect_left_dir = data_path + "/rect_left";
  std::string rect_right_dir = data_path + "/rect_right";

  std::string disp_dir = data_path + "/disp";
  utility_tool::ShellMkdir(disp_dir);
  std::string disp_rgb_dir = data_path + "/disp_rgb";
  utility_tool::ShellMkdir(disp_rgb_dir);

  std::vector<std::string> l_imgs_name, r_imgs_name;
  utility_tool::GetFilesInDirectory(rect_left_dir, &l_imgs_name);
  utility_tool::GetFilesInDirectory(rect_right_dir, &r_imgs_name);

  assert(l_imgs_name.size() == r_imgs_name.size());

  image_algorithm::DisparityCalculator::Param param;
  param.ReadFromYaml(par.get<std::string>("sbgm_cfg"));

  image_algorithm::DisparityCalculator calculator;
  std::vector<int> compress_param;
  compress_param.push_back(cv::IMWRITE_JPEG_QUALITY);
  compress_param.push_back(100);

  int patch_x = par.get<int>("patch_x");
  int patch_y = par.get<int>("patch_y");
  int patch_w = par.get<int>("patch_width");
  int patch_h = par.get<int>("patch_height");

  for (size_t i = 0; i < l_imgs_name.size(); ++i) {
    PCM_PRINT_DEBUG("No. %zu left image name is %s\n", i,
                    l_imgs_name[i].c_str());
    cv::Mat l_tmp_img = cv::imread(l_imgs_name[i], cv::IMREAD_GRAYSCALE);
    // right image
    PCM_PRINT_DEBUG("No. %zu right image name is %s\n", i,
                    r_imgs_name[i].c_str());
    cv::Mat r_tmp_img = cv::imread(r_imgs_name[i], cv::IMREAD_GRAYSCALE);

    if (patch_x == -1 || patch_y == -1 || patch_h == -1 || patch_w == -1) {
      // get the give patch
      cv::resize(l_tmp_img, l_tmp_img,
                 cv::Size(l_tmp_img.cols / 2, l_tmp_img.rows / 2));
      cv::resize(r_tmp_img, r_tmp_img,
                 cv::Size(r_tmp_img.cols / 2, r_tmp_img.rows / 2));
    } else {
      cv::Mat l_full = l_tmp_img.clone(), r_full = r_tmp_img.clone();
      cv::Rect rect(patch_x, patch_y, patch_w, patch_h);
      l_tmp_img = cv::Mat(l_full, rect);
      r_tmp_img = cv::Mat(r_full, rect);
    }

    cv::Mat disp, rgb_disp;
    calculator.CalcuDisparitySGBM(l_tmp_img, r_tmp_img, param, &disp, &rgb_disp);
    // calculator.CalcuDisparityCuda(l_tmp_img, r_tmp_img, &disp, &rgb_disp);

    char img_index[11];
    snprintf(img_index, sizeof(img_index), "%04zu", i);
    cv::imwrite(disp_dir + "/d_" + std::string(img_index) + ".jpg", disp,
                compress_param);
    cv::imwrite(disp_rgb_dir + "/d_rgb_" + std::string(img_index) + ".jpg",
                rgb_disp, compress_param);
  }

  return 0;
}
