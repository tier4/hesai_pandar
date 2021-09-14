#include <cstdio>
#include <iostream>
#include "pandar_api/tcp_client.hpp"

int main()
{
  pandar_api::TCPClient client("192.168.1.201");
  pandar_api::TCPClient::ReturnCode code;

  //get range
  uint16_t range[2];
  code = client.getLidarRange(range);
  if(code == pandar_api::TCPClient::ReturnCode::SUCCESS){
    printf("[%d, %d]\n", range[0], range[1]);
  }

  //get calibration
  std::string calib;
  code = client.getLidarCalibration(calib);
  if(code == pandar_api::TCPClient::ReturnCode::SUCCESS){
    std::cout << "code: " << (int)code << std::endl;
    std::cout << calib << std::endl;
  }

  //get info
  pandar_api::InventoryInfo info;
  code = client.getInventoryInfo(info);
  if(code == pandar_api::TCPClient::ReturnCode::SUCCESS){
    std::cout << "sn: " << info.sn << std::endl;
    std::cout << "date_of_manufacture: " << info.date_of_manufacture << std::endl;
    std::cout << "mac: " << info.mac.c_str() << std::endl;
    std::cout << "sw_ver: " << info.sw_ver << std::endl;
    std::cout << "hw_ver: " << info.hw_ver << std::endl;
    std::cout << "control_fw_ver: " << info.control_fw_ver << std::endl;
    std::cout << "sensor_fw_ver: " << info.sensor_fw_ver << std::endl;
    std::cout << "model: " << info.model << std::endl;
    std::cout << "motor_type: " << info.motor_type << std::endl;
    std::cout << "angle_offset: " << info.angle_offset << std::endl;
    std::cout << "num_of_lines: " << (int)info.num_of_lines << std::endl;
  }

  //get status
  pandar_api::LidarStatus status;
  code = client.getLidarStatus(status);
  if(code == pandar_api::TCPClient::ReturnCode::SUCCESS){
    printf("uptime: %d\n", status.uptime);
    printf("motor_speed: %d\n", status.motor_speed);
    for(auto t:status.temp){
      printf(" temp: %d\n",t);
    }
    printf("gps_pps_lock: %d\n", status.gps_pps_lock);
    printf("gps_gprmc_status: %d\n", status.gps_gprmc_status);
    printf("startup_times: %d\n", status.startup_times);
    printf("total_operation_time: %d\n", status.total_operation_time);
    printf("ptp_clock_status: %d\n", status.ptp_clock_status);
  }
  
  // pandar_api::PTPDiag diag;
  // code = client.getPTPDiagnostics(diag);

  printf("finished!\n");
  return 0;
}