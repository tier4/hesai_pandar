/******************************************************************************
 * Copyright 2019 The Hesai Technology Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include <string>
#include <boost/asio.hpp>
#include "pandar_driver/input.h"

namespace pandar_driver
{
using boost::asio::ip::udp;

class SocketInput : public Input
{
public:
  SocketInput(const std::string& device_ip, uint16_t port, uint16_t gps_port, int timeout=1000);
  ~SocketInput();
  PacketType getPacket(pandar_msgs::PandarPacket* pkt) override;

private:
  boost::asio::io_service io_service_;
  udp::socket lidar_socket_;
  udp::socket gps_socket_;

  boost::asio::ip::address device_ip_;
  int timeout_;
};

}  // namespace pandar_driver
