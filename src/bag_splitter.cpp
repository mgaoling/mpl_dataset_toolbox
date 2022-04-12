#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <utility.hpp>

void PrintProgressInfo(uint32_t message_num, uint32_t message_size) {
  if (message_num % 2000 == 0) ROS_INFO_STREAM(message_num << "/" << message_size << " messages have been processed!");
}

int main(int argc, char ** argv) {
  ros::init(argc, argv, "bag_splitter");
  ros::NodeHandle nh;

  bool receive_imu          = false;
  bool receive_event_left   = false;
  bool receive_event_right  = false;
  bool receive_camera_left  = false;
  bool receive_camera_right = false;
  bool receive_kinect_color = false;
  bool receive_kinect_depth = false;
  bool receive_lidar        = false;
  bool receive_gt           = false;

  std::string bag_in_path, imu_topic, event_left_topic, event_right_topic, camera_left_topic, camera_right_topic, kinect_color_topic,
    kinect_depth_topic, lidar_topic, gt_topic;
  bool need_compression;
  ros::param::get("/bag_path", bag_in_path);
  ros::param::get("/need_compression", need_compression);
  ros::param::get("/imu_topic", imu_topic);
  ros::param::get("/event_left_topic", event_left_topic);
  ros::param::get("/event_right_topic", event_right_topic);
  ros::param::get("/camera_left_topic", camera_left_topic);
  ros::param::get("/camera_right_topic", camera_right_topic);
  ros::param::get("/kinect_color_topic", kinect_color_topic);
  ros::param::get("/kinect_depth_topic", kinect_depth_topic);
  ros::param::get("/lidar_topic", lidar_topic);
  ros::param::get("/ground_truth_topic", gt_topic);
  if (!file_path_check(bag_in_path)) {
    ros::shutdown();
    return 0;
  }
  rosbag::Bag bag_in, imu_bag_out, event_left_bag_out, event_right_bag_out, camera_left_bag_out, camera_right_bag_out, kinect_color_bag_out,
    kinect_depth_bag_out, lidar_bag_out, gt_bag_out;
  bag_in.open(bag_in_path, rosbag::bagmode::Read);
  std::string bag_path_without_extension = bag_in_path.substr(0, bag_in_path.size() - 4);

  // For every message read in rosbag, re-write it into another bag entitled with its topic name.
  uint32_t processed_msg_num = 0;
  uint32_t msg_size          = rosbag::View(bag_in).size();
  for (rosbag::MessageInstance const msg : rosbag::View(bag_in)) {
    if (msg.getTopic() == imu_topic) {
      if (!receive_imu) {
        receive_imu = true;
        imu_bag_out.open(bag_path_without_extension + ".imu.bag", rosbag::bagmode::Write);
        if (need_compression) imu_bag_out.setCompression(rosbag::compression::BZ2);
        imu_bag_out.write(imu_topic, msg.getTime(), msg);
        processed_msg_num++;
        ROS_INFO("%s", colorful_char::info("Receives data from the IMU!").c_str());
      } else {
        imu_bag_out.write(imu_topic, msg.getTime(), msg);
        processed_msg_num++;
        PrintProgressInfo(processed_msg_num, msg_size);
      }
    }

    else if (msg.getTopic() == event_left_topic) {
      if (!receive_event_left) {
        receive_event_left = true;
        event_left_bag_out.open(bag_path_without_extension + ".left_event.bag", rosbag::bagmode::Write);
        if (need_compression) event_left_bag_out.setCompression(rosbag::compression::BZ2);
        event_left_bag_out.write(event_left_topic, msg.getTime(), msg);
        processed_msg_num++;
        ROS_INFO("%s", colorful_char::info("Receives data from the Left Event Camera!").c_str());
      } else {
        event_left_bag_out.write(event_left_topic, msg.getTime(), msg);
        processed_msg_num++;
        PrintProgressInfo(processed_msg_num, msg_size);
      }
    }

    else if (msg.getTopic() == event_right_topic) {
      if (!receive_event_right) {
        receive_event_right = true;
        event_right_bag_out.open(bag_path_without_extension + ".right_event.bag", rosbag::bagmode::Write);
        if (need_compression) event_right_bag_out.setCompression(rosbag::compression::BZ2);
        event_right_bag_out.write(event_right_topic, msg.getTime(), msg);
        processed_msg_num++;
        ROS_INFO("%s", colorful_char::info("Receives data from the Right Event Camera!").c_str());
      } else {
        event_right_bag_out.write(event_right_topic, msg.getTime(), msg);
        processed_msg_num++;
        PrintProgressInfo(processed_msg_num, msg_size);
      }
    }

    else if (msg.getTopic() == camera_left_topic) {
      if (!receive_camera_left) {
        receive_camera_left = true;
        camera_left_bag_out.open(bag_path_without_extension + ".left_camera.bag", rosbag::bagmode::Write);
        if (need_compression) camera_left_bag_out.setCompression(rosbag::compression::BZ2);
        camera_left_bag_out.write(camera_left_topic, msg.getTime(), msg);
        processed_msg_num++;
        ROS_INFO("%s", colorful_char::info("Receives data from the Left Regular Camera!").c_str());
      } else {
        camera_left_bag_out.write(camera_left_topic, msg.getTime(), msg);
        processed_msg_num++;
        PrintProgressInfo(processed_msg_num, msg_size);
      }
    }

    else if (msg.getTopic() == camera_right_topic) {
      if (!receive_camera_right) {
        receive_camera_right = true;
        camera_right_bag_out.open(bag_path_without_extension + ".right_camera.bag", rosbag::bagmode::Write);
        if (need_compression) camera_right_bag_out.setCompression(rosbag::compression::BZ2);
        camera_right_bag_out.write(camera_right_topic, msg.getTime(), msg);
        processed_msg_num++;
        ROS_INFO("%s", colorful_char::info("Receives data from the Right Regular Camera!").c_str());
      } else {
        camera_right_bag_out.write(camera_right_topic, msg.getTime(), msg);
        processed_msg_num++;
        PrintProgressInfo(processed_msg_num, msg_size);
      }
    }

    else if (msg.getTopic() == kinect_color_topic) {
      if (!receive_kinect_color) {
        receive_kinect_color = true;
        kinect_color_bag_out.open(bag_path_without_extension + ".kinect_color.bag", rosbag::bagmode::Write);
        if (need_compression) kinect_color_bag_out.setCompression(rosbag::compression::BZ2);
        kinect_color_bag_out.write(kinect_color_topic, msg.getTime(), msg);
        processed_msg_num++;
        ROS_INFO("%s", colorful_char::info("Receives data from the Kinect Color Camera!").c_str());
      } else {
        kinect_color_bag_out.write(kinect_color_topic, msg.getTime(), msg);
        processed_msg_num++;
        PrintProgressInfo(processed_msg_num, msg_size);
      }
    }

    else if (msg.getTopic() == kinect_depth_topic) {
      if (!receive_kinect_depth) {
        receive_kinect_depth = true;
        kinect_depth_bag_out.open(bag_path_without_extension + ".kinect_depth.bag", rosbag::bagmode::Write);
        if (need_compression) kinect_depth_bag_out.setCompression(rosbag::compression::BZ2);
        kinect_depth_bag_out.write(kinect_depth_topic, msg.getTime(), msg);
        processed_msg_num++;
        ROS_INFO("%s", colorful_char::info("Receives data from the Kinect Depth Camera!").c_str());
      } else {
        kinect_depth_bag_out.write(kinect_depth_topic, msg.getTime(), msg);
        processed_msg_num++;
        PrintProgressInfo(processed_msg_num, msg_size);
      }
    }

    else if (msg.getTopic() == lidar_topic) {
      if (!receive_lidar) {
        receive_lidar = true;
        lidar_bag_out.open(bag_path_without_extension + ".lidar.bag", rosbag::bagmode::Write);
        if (need_compression) lidar_bag_out.setCompression(rosbag::compression::BZ2);
        lidar_bag_out.write(lidar_topic, msg.getTime(), msg);
        processed_msg_num++;
        ROS_INFO("%s", colorful_char::info("Receives data from the LiDAR!").c_str());
      } else {
        lidar_bag_out.write(lidar_topic, msg.getTime(), msg);
        processed_msg_num++;
        PrintProgressInfo(processed_msg_num, msg_size);
      }
    }

    else if (msg.getTopic() == gt_topic) {
      if (!receive_gt) {
        receive_gt = true;
        gt_bag_out.open(bag_path_without_extension + ".gt.bag", rosbag::bagmode::Write);
        if (need_compression) gt_bag_out.setCompression(rosbag::compression::BZ2);
        gt_bag_out.write(gt_topic, msg.getTime(), msg);
        processed_msg_num++;
        ROS_INFO("%s", colorful_char::info("Receives data from the Ground Truth Signal!").c_str());
      } else {
        gt_bag_out.write(gt_topic, msg.getTime(), msg);
        processed_msg_num++;
        PrintProgressInfo(processed_msg_num, msg_size);
      }
    }
  }
  ROS_INFO("%s", colorful_char::info("All messages have been processed!").c_str());

  if (receive_imu) imu_bag_out.close();
  if (receive_event_left) event_left_bag_out.close();
  if (receive_event_right) event_right_bag_out.close();
  if (receive_camera_left) camera_left_bag_out.close();
  if (receive_camera_right) camera_right_bag_out.close();
  if (receive_kinect_color) kinect_color_bag_out.close();
  if (receive_kinect_depth) kinect_depth_bag_out.close();
  if (receive_lidar) lidar_bag_out.close();
  if (receive_gt) gt_bag_out.close();

  ros::shutdown();
  return 0;
}