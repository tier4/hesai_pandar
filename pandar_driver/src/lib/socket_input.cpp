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

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <string.h>
#include <sys/file.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <iostream>
#include <sstream>
#include <time.h>
#include <unistd.h>
// #include "pandar_driver/tcp_util.h"
#include "pandar_driver/socket_input.h"

using namespace pandar_driver;

namespace
{
const size_t ETHERNET_MTU = 1500;
}

SocketInput::SocketInput(uint16_t port, uint16_t gpsPort, int timeout)
{
  // LOG_D("port: %d, gpsPort: %d", port,gpsPort);
  lidar_socket_ = -1;
  lidar_socket_ = socket(PF_INET, SOCK_DGRAM, 0);
  if (lidar_socket_ == -1) {
    perror("socket");  // TODO(Philip.Pi): perror errno.
    return;
  }

  sockaddr_in myAddress;                     // my address information
  memset(&myAddress, 0, sizeof(myAddress));  // initialize to zeros
  myAddress.sin_family = AF_INET;            // host byte order
  myAddress.sin_port = htons(port);          // port in network byte order
  myAddress.sin_addr.s_addr = INADDR_ANY;    // automatically fill in my IP

  if (bind(lidar_socket_, reinterpret_cast<sockaddr*>(&myAddress), sizeof(sockaddr)) == -1) {
    perror("bind");  // TODO(Philip.Pi): perror errno
    return;
  }

  if (fcntl(lidar_socket_, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
    perror("non-block");
    return;
  }

  if (port == gpsPort) {
    socket_num_ = 1;
    return;
  }
  // gps socket
  gps_socket_ = -1;
  gps_socket_ = socket(PF_INET, SOCK_DGRAM, 0);
  if (gps_socket_ == -1) {
    perror("socket");  // TODO(Philip.Pi): perror errno.
    return;
  }

  sockaddr_in myAddressGPS;                        // my address information
  memset(&myAddressGPS, 0, sizeof(myAddressGPS));  // initialize to zeros
  myAddressGPS.sin_family = AF_INET;               // host byte order
  myAddressGPS.sin_port = htons(gpsPort);          // port in network byte order
  myAddressGPS.sin_addr.s_addr = INADDR_ANY;       // automatically fill in my IP

  if (bind(gps_socket_, reinterpret_cast<sockaddr*>(&myAddressGPS), sizeof(sockaddr)) == -1) {
    perror("bind");  // TODO(Philip.Pi): perror errno
    return;
  }

  if (fcntl(gps_socket_, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
    perror("non-block");
    return;
  }
  socket_num_ = 2;
  timeout_ = timeout;
}

SocketInput::~SocketInput(void)
{
  if (gps_socket_ > 0)
    close(gps_socket_);
  if (lidar_socket_ > 0)
    (void)close(lidar_socket_);
}

// return : 0 - lidar
//          1 - gps
//          -1 - error
SocketInput::PacketType SocketInput::getPacket(pandar_msgs::PandarPacket* pkt)
{
  struct pollfd fds[socket_num_];
  if (socket_num_ == 2) {
    fds[0].fd = gps_socket_;
    fds[0].events = POLLIN;

    fds[1].fd = lidar_socket_;
    fds[1].events = POLLIN;
  }
  else if (socket_num_ == 1) {
    fds[0].fd = lidar_socket_;
    fds[0].events = POLLIN;
  }

  sockaddr_in senderAddress;
  socklen_t senderAddressLen = sizeof(senderAddress);
  int retval = poll(fds, socket_num_, timeout_);
  if (retval < 0) {
    // poll() error?
    if (errno != EINTR)
      printf("poll() error: %s", strerror(errno));
    return PacketType::ERROR;
  }else if (retval == 0) {
    // timeout
    return PacketType::TIMEOUT;
  }else
  if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) || (fds[0].revents & POLLNVAL)) {
    // device error
    perror("poll() reports Pandar error");
    return PacketType::ERROR;
  }

  senderAddressLen = sizeof(senderAddress);
  ssize_t nbytes;
  pkt->stamp = ros::Time::now();
  for (int i = 0; i != socket_num_; ++i) {
    if (fds[i].revents & POLLIN) {
      nbytes = recvfrom(fds[i].fd, &pkt->data[0], ETHERNET_MTU, 0, reinterpret_cast<sockaddr*>(&senderAddress),
                        &senderAddressLen);
      break;
    }
  }

  if (nbytes < 0) {
    if (errno != EWOULDBLOCK) {
      perror("recvfail");
      return PacketType::ERROR;
    }
  }
  pkt->size = nbytes;
  // pkt->stamp = time;

  return PacketType::LIDAR;
}
