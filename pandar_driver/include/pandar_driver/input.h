#pragma once
#include <pandar_msgs/PandarPacket.h>

namespace pandar_driver
{
class Input
{
public:
  enum class PacketType : int {
    LIDAR = 0,
    GPS = 1,
    TIMEOUT = 2,
    ERROR = -1
  };
  virtual ~Input(){};
  virtual PacketType getPacket(pandar_msgs::PandarPacket* pkt) = 0;
};
}  // namespace pandar_driver
