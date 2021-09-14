#include "pandar_driver/socket_input.h"

using namespace pandar_driver;

namespace
{
const size_t ETHERNET_MTU = 1500;
}

SocketInput::SocketInput(const std::string& device_ip, uint16_t port, uint16_t gps_port, int timeout)
  : io_service_(),
    lidar_socket_(io_service_, udp::endpoint(udp::v4(), port)),
    gps_socket_(io_service_, udp::endpoint(udp::v4(), gps_port))
{
  device_ip_ = boost::asio::ip::address::from_string(device_ip);

  timeout_ = timeout;
  struct timeval tv{0, timeout_};
  setsockopt(lidar_socket_.native_handle(), SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv) );
  setsockopt(lidar_socket_.native_handle(), SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv) );
}

SocketInput::~SocketInput(void)
{
  lidar_socket_.close();
}

SocketInput::PacketType SocketInput::getPacket(pandar_msgs::PandarPacket* pkt)
{
  boost::system::error_code error_code;
  udp::endpoint remote_endpoint;

  size_t packet_size = lidar_socket_.receive_from(boost::asio::buffer(pkt->data, ETHERNET_MTU), remote_endpoint, 0, error_code);
  if(error_code == boost::system::errc::success && remote_endpoint.address() == device_ip_){
    pkt->stamp = ros::Time::now();
    pkt->size = static_cast<uint32_t>(packet_size);
    return PacketType::LIDAR;
  }else if(error_code == boost::system::errc::timed_out){
    return PacketType::TIMEOUT;
  }else{
    return PacketType::ERROR;
  }
}
