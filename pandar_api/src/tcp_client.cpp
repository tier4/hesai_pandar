#include <string>
#include <iostream>
#include <cstdint>

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
TCPClient::TCPClient(const std::string& device_ip)
  : io_service_(),
    socket_(io_service_)
{
  boost::system::error_code error;
  socket_.connect(boost::asio::ip::tcp::endpoint(boost::asio::ip::address::from_string(device_ip), API_PORT), error);
  if (error) {
    std::cout << "connect failed : " << error.message() << std::endl;
  }
}

TCPClient::ReturnCode TCPClient::getLidarCalibration(std::string& content)
{
  MessageHeader header;
  std::vector<uint8_t> payload;

  header.cmd = PTC_COMMAND_GET_LIDAR_CALIBRATION;
  auto return_code = sendCmd(header, payload);
  if(return_code == ReturnCode::SUCCESS){
    content = std::string(payload.data(), payload.data() + payload.size());
    return ReturnCode::SUCCESS;
  }else{
    return return_code;
  }
}

TCPClient::ReturnCode TCPClient::getLidarRange(uint16_t* range)
{
  MessageHeader header;
  std::vector<uint8_t> payload;

  header.cmd = PTC_COMMAND_GET_LIDAR_RANGE;
  auto return_code = sendCmd(header, payload);
  if(return_code == ReturnCode::SUCCESS){
    if(payload[0] != 0){
      // not support each-channel / multi-section
      return ReturnCode::NO_SUPPORT;
    }else{
      range[0] = parse16(&payload[1]);
      range[1] = parse16(&payload[3]);
      return ReturnCode::SUCCESS;
    }
  }else{
    return return_code;
  }
}

TCPClient::ReturnCode TCPClient::getLidarStatus(LidarStatus& status)
{
  MessageHeader header;
  std::vector<uint8_t> payload;

  header.cmd = PTC_COMMAND_GET_LIDAR_STATUS;
  auto return_code = sendCmd(header, payload);
  if(return_code == ReturnCode::SUCCESS){

    uint8_t* it = payload.data();

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

    return ReturnCode::SUCCESS;
  }else{
    return return_code;
  }
}

TCPClient::ReturnCode TCPClient::sendCmd(MessageHeader& header, std::vector<uint8_t>& payload)
{
  boost::system::error_code error;
  std::vector<uint8_t> buffer;

  // send header
  buffer.resize(HEADER_SIZE);
  header.write(buffer.data());
  boost::asio::write(socket_, boost::asio::buffer(buffer), error);
  if(error){
    return ReturnCode::CONNECTION_FAILED;
  }

  // send payload
  if(payload.size() > 0){
    boost::asio::write(socket_, boost::asio::buffer(payload), error);
    if(error){
      // std::cout << error.message() << std::endl;
      return ReturnCode::CONNECTION_FAILED;
    }
  }
  // receive header
  boost::asio::read(socket_, boost::asio::buffer(buffer), boost::asio::transfer_exactly(HEADER_SIZE), error);
  if(error){
    // std::cout << error.message() << std::endl;
    return ReturnCode::CONNECTION_FAILED;
  }else{
    header.read(buffer.data());
    if(header.protocol_identifier[0] != 0x47 || header.protocol_identifier[1] != 0x74){
      return ReturnCode::CONNECTION_FAILED;
    }
    if((ReturnCode)header.return_code != ReturnCode::SUCCESS) {
      return (ReturnCode)header.return_code;
    }
  }

  // receive payload
  if(header.payload_length > 0){
    payload.resize(header.payload_length);
    boost::asio::read(socket_, boost::asio::buffer(payload), boost::asio::transfer_exactly(payload.size()), error);
    if(error){
      // std::cout << error.message() << std::endl;
      return ReturnCode::CONNECTION_FAILED;
    }
  }

  return ReturnCode::SUCCESS;
}

}  // namespace pandar_api