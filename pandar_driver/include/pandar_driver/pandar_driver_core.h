#pragma once

#include <rclcpp/rclcpp.hpp>
#include <pandar_msgs/msg/pandar_scan.hpp>
#include <pandar_api/tcp_client.hpp>
namespace pandar_driver
{
class Input;
class PandarDriverCore
{
public:
  PandarDriverCore(rclcpp::Node *node);
  ~PandarDriverCore(){};
  bool poll(void);

private:
  rclcpp::Node * node_;

  std::string device_ip_;
  int lidar_port_;
  int gps_port_;
  double scan_phase_;
  size_t azimuth_index_;

  std::string model_;
  std::string frame_id_;
  std::string pcap_path_;

  rclcpp::Publisher<pandar_msgs::msg::PandarScan>::SharedPtr pandar_packet_pub_;
  std::shared_ptr<Input> input_;
  std::shared_ptr<pandar_api::TCPClient> client_;

  std::function<bool(size_t)> is_valid_packet_;
};
}  // namespace pandar_driver