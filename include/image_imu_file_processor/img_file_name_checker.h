/*******************************************************************************
 *   Copyright (C) 2023 CASIA. All rights reserved.
 *
 *   @Filename: img_file_name_checker.h
 *
 *   @Author: shun li
 *
 *   @Email: shun.li.at.casia@outlook.com
 *
 *   @Date: 23/08/2023
 *
 *   @Description:
 *
 *******************************************************************************/

#ifndef IMG_FILE_NAME_CHECKER_H_
#define IMG_FILE_NAME_CHECKER_H_

#include <string>
#include "utility_tool/print_ctrl_macro.h"

class ImgNameChecker {
 public:
  struct BinNameEle {
    // file name without path.
    std::string fn{""};
    // file name without extentions.
    std::string pure_fn{""};
    // file extentions.
    std::string tail{""};
    // file channel string.
    std::string ch{""};
    // file name before ch_num.
    std::string pre{""};
  };

  static void GetImgBinNameEle(const std::string bin_path, BinNameEle* ele) {
    // get the file name
    std::string::size_type ipos = bin_path.find_last_of('/') + 1;
    ele->fn = bin_path.substr(ipos, bin_path.length() - ipos);

    ele->pure_fn = ele->fn.substr(0, ele->fn.rfind("."));
    ele->tail = ele->fn.substr(ele->fn.rfind(".") + 1);
    ele->ch = ele->pure_fn.substr(ele->fn.rfind("_") + 1);
    ele->pre = ele->pure_fn.substr(0, ele->fn.rfind("_"));
  }
};

#endif
