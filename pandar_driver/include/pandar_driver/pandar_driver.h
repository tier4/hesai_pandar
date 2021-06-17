#pragma once

#include <ros/ros.h>

namespace pandar_driver
{
class Input;
class PandarDriver
{
public:
  PandarDriver(ros::NodeHandle node, ros::NodeHandle private_nh);
  ~PandarDriver(){};
  bool poll(void);

private:
  std::string device_ip_;
  int lidar_port_;
  int gps_port_;
  double scan_phase_;
  double min_range_;
  double max_range_;
  size_t azimuth_index_;

  std::string model_;
  std::string frame_id_;
  std::string pcap_path_;

  ros::Publisher pandar_packet_pub_;
  std::shared_ptr<Input> input_;

  std::function<bool(size_t)> is_valid_packet_;
};
}  // namespace pandar_driver