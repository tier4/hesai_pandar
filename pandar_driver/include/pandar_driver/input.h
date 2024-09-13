#pragma once
#include <pandar_msgs/msg/pandar_scan.hpp>

namespace pandar_driver
{
class Input
{
public:
  enum class PacketType : int {
    LIDAR = 0,
    GPS = 1,
    ERROR = -1
  };
  virtual ~Input(){};
  virtual PacketType getPacket(pandar_msgs::msg::PandarPacket* pkt) = 0;
};
}  // namespace pandar_driver
