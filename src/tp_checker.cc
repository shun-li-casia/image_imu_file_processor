/*******************************************************************************
 *   Copyright (C) 2023 CASIA. All rights reserved.
 *
 *   @Filename: tp_checker.cc
 *
 *   @Author: shun li
 *
 *   @Email: shun.li.at.casia@outlook.com
 *
 *   @Date: 07/09/2024
 *
 *   @Description:
 *
 *******************************************************************************/

#include <utility_tool/cmdline.h>
#include <sensor_config/tools/tp_checker.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>

int main(int argc, char** argv) {
  cmdline::parser par;
  par.add<std::string>("type", 'i', "file name", true);
  par.add<std::string>("topic", 't', "file name", true);
  par.add<std::string>("bag", 'b', "ros bag name", false, "");
  par.parse_check(argc, argv);

  bool if_use_bag = par.exist("bag");
  std::string bag_name = par.get<std::string>("bag");

  ros::init(argc, argv,
            "tp_checker" + utility_tool::GetCurLocalTimeStr("%H%M%S"));
  ros::NodeHandle nh;

  std::string topic = par.get<std::string>("topic");
  std::string type = par.get<std::string>("type");

  if (type == "imu") {
    if (if_use_bag && !bag_name.empty()) {
      sensor_config::TpChecker<sensor_msgs::Imu> tp_checker(topic, bag_name,
                                                            "");
      tp_checker.BagRun();
    } else {
      sensor_config::TpChecker<sensor_msgs::Imu> tp_checker(topic);
      tp_checker.Spin();
    }
  } else if (type == "image") {
    if (if_use_bag && !bag_name.empty()) {
      sensor_config::TpChecker<sensor_msgs::Image> tp_checker(topic, bag_name,
                                                            "");
      tp_checker.BagRun();
    } else {
      sensor_config::TpChecker<sensor_msgs::Image> tp_checker(topic);
      tp_checker.Spin();
    }
  } else {
    PCM_PRINT_ERROR("Type %s is not supported", type.c_str());
    return -1;
  }

  return 0;
}
