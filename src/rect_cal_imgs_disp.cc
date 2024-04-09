/*******************************************************************************
 *   Copyright (C) 2023 CASIA. All rights reserved.
 *
 *   @Filename: rect_cal_imgs_disp.cc
 *
 *   @Author: shun li
 *
 *   @Email: shun.li.at.casia@outlook.com
 *
 *   @Date: 14/03/2024
 *
 *   @Description:  subscribe the original images rect them firt, then calculate
 *   the disparity; advertise the rected images and disparity images.
 *
 *******************************************************************************/

#include "utility_tool/cmdline.h"
#include "utility_tool/system_lib.h"
#include "utility_tool/pcm_debug_helper.h"
#include "sensor_config/camera_models_kalibr.h"

#include "image_algorithm/disparity_calculator.h"
#include "image_algorithm/stereo_rectifier.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Geometry>

int main(int argc, char** argv) {
  cmdline::parser par;
  par.add<std::string>("kalibr_param", 0, "the kalibr config file", true);
  // read the image and remap them, then save to the rect_ dir
  par.add<std::string>("data_path", 0,
                       "the left and right images' path, which should contains "
                       "{left} & {right} sub-directory",
                       true);
  par.add<double>("img_scale", 0, "the image scale", true);
  par.parse_check(argc, argv);

  // read the kailbr config
  const std::string kalibr_param = par.get<std::string>("kalibr_param");
  const double scale = par.get<double>("img_scale");
  sensor_config::ImgImuConfig conf;
  sensor_config::ConfigManager::ReadKalibr(kalibr_param, &conf, scale);

  // generate the map for rectification
  Eigen::Matrix3d rect_Rrl;
  Eigen::Vector3d rect_trl;
  camodocal::PinholeCamera::Parameters l_cam = conf.cam_params_[0];
  camodocal::PinholeCamera::Parameters r_cam = conf.cam_params_[1];
  std::pair<cv::Mat, cv::Mat> l_map, r_map;

  image_algorithm::StereoRectifier::RectStereoParam(
      conf.r_rl_, conf.t_rl_, &rect_Rrl, &rect_trl, &l_cam, &r_cam, &l_map,
      &r_map);

  // generate the rected kailbr config
  sensor_config::ImgImuConfig rect_conf;
  rect_conf.r_rl_ = rect_Rrl;
  rect_conf.t_rl_ = rect_trl;
  rect_conf.cam_params_.push_back(l_cam);
  rect_conf.cam_params_.push_back(r_cam);
  rect_conf.rostopic_.push_back(conf.rostopic_[0] + "_rect");
  rect_conf.rostopic_.push_back(conf.rostopic_[1] + "_rect");
  rect_conf.cam_overlaps_ = conf.cam_overlaps_;
  sensor_config::ConfigManager::WriteKalibr(rect_conf, "rect_" + kalibr_param);

  ros::init(argc, argv, "rect_cal_imgs_disp_node");
  ros::NodeHandle nh;
  ros::Publisher rect_l_pub =
      nh.advertise<sensor_msgs::Image>(rect_conf.rostopic_[0], 10);
  ros::Publisher rect_r_pub =
      nh.advertise<sensor_msgs::Image>(rect_conf.rostopic_[1], 10);
  ros::Publisher disp_rgb_pub =
      nh.advertise<sensor_msgs::Image>("disp_rgb", 10);

  // load the images in the left and right image
  std::string data_path = par.get<std::string>("data_path");
  std::string left_dir = data_path + "/left";
  std::string right_dir = data_path + "/right";
  std::vector<std::string> l_imgs_name, r_imgs_name;
  utility_tool::GetFilesInDirectory(left_dir, &l_imgs_name);
  utility_tool::GetFilesInDirectory(right_dir, &r_imgs_name);
  assert(l_imgs_name.size() == r_imgs_name.size());

  // the disparity calculator
  image_algorithm::DisparityCalculator calculator;
  image_algorithm::DisparityCalculator::Param param;
  param.block_size = 5;
  param.num_disparities = 64;

  utility_tool::Timer timer;

  for (size_t i = 0; i < l_imgs_name.size() && ros::ok(); ++i) {
    timer.Start();

    cv::Mat l_tmp_img = cv::imread(l_imgs_name[i], cv::IMREAD_GRAYSCALE);
    cv::Mat r_tmp_img = cv::imread(r_imgs_name[i], cv::IMREAD_GRAYSCALE);
    cv::resize(l_tmp_img, l_tmp_img, cv::Size(), scale, scale);
    cv::resize(r_tmp_img, r_tmp_img, cv::Size(), scale, scale);

    cv::Mat l_rect_img, r_rect_img;
    cv::remap(l_tmp_img, l_rect_img, l_map.first, l_map.second,
              cv::INTER_LINEAR);
    cv::remap(r_tmp_img, r_rect_img, r_map.first, r_map.second,
              cv::INTER_LINEAR);

    // publish the rect
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    sensor_msgs::ImagePtr rect_l_msg =
        cv_bridge::CvImage(header, "mono8", l_rect_img).toImageMsg();
    rect_l_pub.publish(rect_l_msg);
    sensor_msgs::ImagePtr rect_r_msg =
        cv_bridge::CvImage(header, "mono8", r_rect_img).toImageMsg();
    rect_r_pub.publish(rect_r_msg);

    // calculate the disparity

    cv::Mat disp, rgb_disp;
    calculator.CalcuDisparitySGBM(l_rect_img, r_rect_img, param, &disp,
                                        &rgb_disp);
    sensor_msgs::ImagePtr disp_msg =
        cv_bridge::CvImage(header, "bgr8", rgb_disp).toImageMsg();
    disp_rgb_pub.publish(disp_msg);

    PCM_PRINT_DEBUG("time cost: %f, fps: %f\n", timer.End(),
                    1000.0 / timer.End());
  }

  return 0;
}
