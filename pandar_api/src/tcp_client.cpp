#include <string>
#include <iostream>
#include <cstdint>
#include <boost/bind.hpp>

#include "pandar_api/tcp_client.hpp"

namespace{
  const uint16_t API_PORT = 9347;
  const size_t HEADER_SIZE = 8;

  inline uint32_t parse32(uint8_t* raw){
    return htobe32(*((uint32_t*)raw)); 
  }

  inline uint16_t parse16(uint8_t* raw){
    return htobe16(*((uint16_t*)raw));
  }
}

namespace pandar_api
{
TCPClient::TCPClient(const std::string& device_ip, int32_t timeout)
  : io_service_(),
    socket_(io_service_),
    timer_(io_service_),
    return_code_(ReturnCode::SUCCESS)
{
  device_ip_ = boost::asio::ip::address::from_string(device_ip);
}


TCPClient::ReturnCode TCPClient::getLidarCalibration(std::string& content)
{
  header_.cmd = PTC_COMMAND_GET_LIDAR_CALIBRATION;
  connect();
  if(return_code_ == ReturnCode::SUCCESS){
    content = std::string(payload_.data(), payload_.data() + payload_.size());
  }
  return return_code_;
}

TCPClient::ReturnCode TCPClient::getLidarRange(uint16_t* range)
{
  header_.cmd = PTC_COMMAND_GET_LIDAR_CALIBRATION;
  connect();
  if(return_code_ == ReturnCode::SUCCESS){
    if(payload_[0] != 0){
      // not support each-channel / multi-section
      return ReturnCode::NO_SUPPORT;
    }else{
      range[0] = parse16(&payload_[1]);
      range[1] = parse16(&payload_[3]);
      return return_code_;
    }
  }else{
    return return_code_;
  }
}

TCPClient::ReturnCode TCPClient::getLidarStatus(LidarStatus& status)
{
  header_.cmd = PTC_COMMAND_GET_LIDAR_STATUS;
  connect();
  if(return_code_ == ReturnCode::SUCCESS){
    uint8_t* it = payload_.data();

    status.uptime = parse32(it);
    it += 4;
    status.motor_speed = parse16(it);
    it += 2;
    status.temp[8];
    for(int i = 0; i < 8; i++){
      status.temp[i] = parse32(it);
      it += 4;
    }
    status.gps_pps_lock = *it;
    it+=1;

    status.gps_gprmc_status = *it;
    it+=1;

    status.startup_times = parse32(it);
    it += 4;
    
    status.total_operation_time = parse32(it);
    it += 4;
    
    status.ptp_clock_status = *it;

    return return_code_;
  }else{
    return return_code_;
  }
}

void TCPClient::connect()
{

  socket_.async_connect(
    boost::asio::ip::tcp::endpoint(device_ip_, API_PORT),
    boost::bind(&TCPClient::on_connect, this, boost::asio::placeholders::error));

  timer_.expires_from_now(std::chrono::milliseconds(100));
  timer_.async_wait(boost::bind(&TCPClient::on_timer, this, boost::placeholders::_1));
  io_service_.run();
}

void TCPClient::on_connect(const boost::system::error_code& error)
{
  if (error) {
    return_code_ = ReturnCode::CONNECTION_FAILED;
    return;
  }

  // send header
  buffer_.resize(HEADER_SIZE + payload_.size());
  header_.write(buffer_.data());
  std::memcpy(buffer_.data() + HEADER_SIZE, payload_.data(), payload_.size());

  boost::asio::async_write(
      socket_,
      boost::asio::buffer(buffer_),
      boost::bind(&TCPClient::on_send, this, boost::asio::placeholders::error));
}

void TCPClient::on_send(const boost::system::error_code& error)
{
  if (error) {
    return_code_ = ReturnCode::CONNECTION_FAILED;
    return;
  }

  boost::asio::async_read(
    socket_,
    boost::asio::buffer(buffer_),
    boost::asio::transfer_exactly(HEADER_SIZE),
    boost::bind(&TCPClient::on_receive, this, boost::asio::placeholders::error));
}

void TCPClient::on_receive(const boost::system::error_code& error)
{
  if (error) {
    return_code_ = ReturnCode::CONNECTION_FAILED;
    return;
  }
  header_.read(buffer_.data());
  if(header_.protocol_identifier[0] != 0x47 || header_.protocol_identifier[1] != 0x74){
    return_code_ = ReturnCode::CONNECTION_FAILED;
    socket_.close();
    return;
  }else if((ReturnCode)header_.return_code != ReturnCode::SUCCESS) {
    return_code_ = (ReturnCode)header_.return_code;
    socket_.close();
    return;
  }

  // receive payload
  if (header_.payload_length > 0){
    payload_.resize(header_.payload_length);
    boost::asio::async_read(
        socket_,
        boost::asio::buffer(payload_),
        boost::asio::transfer_exactly(payload_.size()),
        [this](const boost::system::error_code& error, std::size_t)
        {
          if (error){
            return_code_ = ReturnCode::CONNECTION_FAILED;
          }
          else {
            timer_.cancel();
            return;
          }
        });
  }
}

void TCPClient::on_timer(const boost::system::error_code& error)
{
  if (!error) {
    return_code_ = ReturnCode::CONNECTION_FAILED;
    socket_.close();
  }
}

}  // namespace pandar_api