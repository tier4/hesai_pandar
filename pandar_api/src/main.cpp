#include <cstdio>
#include <iostream>
#include "pandar_api/tcp_client.hpp"

int main()
{
  pandar_api::TCPClient client("192.168.1.201");

  //get calibration
  std::string calib;
  auto code = client.getLidarCalibration(calib);
  std::cout << "code: " << (int)code << std::endl;
  std::cout << calib << std::endl;

  //get range
  uint16_t range[2];
  client.getLidarRange(range);
  printf("[%d, %d]\n", range[0], range[1]);

  //get status
  pandar_api::LidarStatus status;
  client.getLidarStatus(status);
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

  printf("finished!\n");
  return 0;
}