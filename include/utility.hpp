#ifndef DATASET_TOOLBOX_UTILITY_HPP_
#define DATASET_TOOLBOX_UTILITY_HPP_

#include <geometry_msgs/PoseStamped.h>
#include <prophesee_event_msgs/Event.h>
#include <prophesee_event_msgs/EventArray.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>

using IMU         = sensor_msgs::Imu;
using Event       = prophesee_event_msgs::Event;
using EventArray  = prophesee_event_msgs::EventArray;
using Image       = sensor_msgs::Image;
using PointCloud  = sensor_msgs::PointCloud2;
using PoseStamped = geometry_msgs::PoseStamped;

namespace colorful_char {

std::string info(std::string input_str) {
  return "\033[1;32m>> " + input_str + " \033[0m";
}

std::string warning(std::string input_str) {
  return "\033[1;35m>> WARNING: " + input_str + " \033[0m";
}

std::string error(std::string input_str) {
  return "\033[1;31m>> ERROR: " + input_str + " \033[0m";
}

}  // namespace colorful_char

#endif  // DATASET_TOOLBOX_UTILITY_HPP_
