#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <utility.hpp>

// ------------------------------------------------------------------------- //

int main(int argc, char ** argv) {
  ros::init(argc, argv, "bag2rawfile");
  ros::NodeHandle nh;

  bool receive_imu          = false;
  bool receive_camera_left  = false;
  bool receive_camera_right = false;
  bool receive_kinect_depth = false;
  bool receive_lidar        = false;

  std::string bag_in_path, imu_topic, camera_left_topic, camera_right_topic, kinect_depth_topic, lidar_topic;
  ros::param::get("/bag_path", bag_in_path);
  ros::param::get("/imu_topic", imu_topic);
  ros::param::get("/camera_left_topic", camera_left_topic);
  ros::param::get("/camera_right_topic", camera_right_topic);
  ros::param::get("/kinect_depth_topic", kinect_depth_topic);
  ros::param::get("/lidar_topic", lidar_topic);
  if (!file_path_check(bag_in_path)) {
    ros::shutdown();
    return -1;
  }
  rosbag::Bag bag_in;
  bag_in.open(bag_in_path, rosbag::bagmode::Read);
  std::ofstream imu_txt, left_camera_txt, right_camera_txt;

  std::string folder_path, bag_name, out_path;
  folder_path = bag_in_path.substr(0, bag_in_path.find_last_of("/\\") + 1);
  bag_name    = bag_in_path.substr(bag_in_path.find_last_of("/\\") + 1, bag_in_path.size() - bag_in_path.find_last_of("/\\") - 5);
  out_path    = folder_path + "/raw_data/";
  fs::create_directory(out_path);

  uint32_t msg_idx  = 0;
  uint32_t msg_size = rosbag::View(bag_in).size();
  for (rosbag::MessageInstance const msg : rosbag::View(bag_in)) {
    if (msg.getTopic() == imu_topic) {
      IMU::Ptr imu_msg = msg.instantiate<IMU>();
      if (!receive_imu) {
        receive_imu = true;
        imu_txt     = std::ofstream(out_path + ".imu.txt", std::ofstream::out);
        imu_txt << "# IMU data for " << bag_name << std::endl
                << "# Readings from gyroscope[rad/s], accelerometer[m/s^2], and magnetometer[quaternion]" << std::endl
                << "# timestamp gx gy gz ax ay az qx qy qz qw" << std::endl;
        ROS_INFO("%s", colorful_char::info("Receives data from the IMU!").c_str());
      }
      imu_txt << std::to_string(imu_msg->header.stamp.toSec()) << " " << imu_msg->angular_velocity.x << " " << imu_msg->angular_velocity.y
              << " " << imu_msg->angular_velocity.z << " " << imu_msg->linear_acceleration.x << " " << imu_msg->linear_acceleration.y << " "
              << imu_msg->linear_acceleration.z << " " << imu_msg->orientation.x << " " << imu_msg->orientation.y << " "
              << imu_msg->orientation.z << " " << imu_msg->orientation.w << std::endl;
    }

    else if (msg.getTopic() == camera_left_topic) {
      Image::Ptr img_msg = msg.instantiate<Image>();
      if (!receive_camera_left) {
        receive_camera_left = true;
        fs::create_directory(out_path + "left_regular_camera/");
        left_camera_txt = std::ofstream(out_path + "left_regular_camera/timestamp.txt", std::ofstream::out);
        left_camera_txt << "# Left Regular Camera's timestamp data for " << bag_name << std::endl
                        << "# start and end of the exposure time[s] for each image" << std::endl;
        ROS_INFO("%s", colorful_char::info("Receives data from the Left Regular Camera!").c_str());
      }
      left_camera_txt << std::to_string(img_msg->header.stamp.toSec()) << " " << std::to_string(img_msg->header.stamp.toSec() + 0.010)
                      << std::endl;
      cv::imwrite(out_path + "left_regular_camera/" + std::to_string(img_msg->header.stamp.toSec()) + ".png",
                  cv_bridge::toCvCopy(img_msg)->image);
    }

    else if (msg.getTopic() == camera_right_topic) {
      Image::Ptr img_msg = msg.instantiate<Image>();
      if (!receive_camera_right) {
        receive_camera_right = true;
        fs::create_directory(out_path + "right_regular_camera/");
        right_camera_txt = std::ofstream(out_path + "right_regular_camera/timestamp.txt", std::ofstream::out);
        right_camera_txt << "# Right Regular Camera's timestamp data for " << bag_name << std::endl
                         << "# start and end of the exposure time[s] for each image" << std::endl;
        ROS_INFO("%s", colorful_char::info("Receives data from the Right Regular Camera!").c_str());
      }
      right_camera_txt << std::to_string(img_msg->header.stamp.toSec()) << " " << std::to_string(img_msg->header.stamp.toSec() + 0.010)
                       << std::endl;
      cv::imwrite(out_path + "right_regular_camera/" + std::to_string(img_msg->header.stamp.toSec()) + ".png",
                  cv_bridge::toCvCopy(img_msg)->image);
    }

    else if (msg.getTopic() == kinect_depth_topic) {
      Image::Ptr img_msg = msg.instantiate<Image>();
      if (!receive_kinect_depth) {
        receive_kinect_depth = true;
        fs::create_directory(out_path + "kinect_depth_camera/");
        ROS_INFO("%s", colorful_char::info("Receives data from the Kinect Depth Camera!").c_str());
      }
      cv::imwrite(out_path + "kinect_depth_camera/" + std::to_string(img_msg->header.stamp.toSec()) + ".png",
                  cv_bridge::toCvCopy(img_msg)->image);
    }

    else if (msg.getTopic() == lidar_topic) {
      PointCloud::Ptr cloud_msg = msg.instantiate<PointCloud>();
      if (!receive_lidar) {
        receive_lidar = true;
        fs::create_directory(out_path + "lidar/");
        ROS_INFO("%s", colorful_char::info("Receives data from the LiDAR!").c_str());
      }
      pcl::io::savePCDFile(out_path + "lidar/" + std::to_string(cloud_msg->header.stamp.toSec()) + ".pcd", *cloud_msg,
                           Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), true);
    }

    msg_idx++;
    if (msg_idx % 10000 == 0) ROS_INFO_STREAM(msg_idx << "/" << msg_size << " messages have been processed!");
  }
  ROS_INFO("%s", colorful_char::info("All messages have been processed!").c_str());

  if (receive_imu) imu_txt.close();
  if (receive_camera_left) left_camera_txt.close();
  if (receive_camera_right) right_camera_txt.close();

  ros::shutdown();
  return 0;
}
