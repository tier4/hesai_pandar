#include <pandar_driver/pandar_driver.h>
#include <pandar_driver/input.h>
#include <pandar_driver/pcap_input.h>
#include <pandar_driver/socket_input.h>
#include <pandar_msgs/PandarPacket.h>
#include <pandar_msgs/PandarScan.h>

using namespace pandar_driver;

PandarDriver::PandarDriver(ros::NodeHandle node, ros::NodeHandle private_nh)
{
  private_nh.getParam("pcap", pcap_path_);
  private_nh.getParam("device_ip", device_ip_);
  private_nh.getParam("lidar_port", lidar_port_);
  private_nh.getParam("gps_port", gps_port_);
  private_nh.getParam("scan_phase", scan_phase_);
  private_nh.getParam("min_range", min_range_);
  private_nh.getParam("max_range", max_range_);
  private_nh.getParam("model", model_);
  private_nh.getParam("frame_id", frame_id_);

  pandar_packet_pub_ = node.advertise<pandar_msgs::PandarScan>("pandar_packets", 10);

  if (!pcap_path_.empty()) {
    input_.reset(new PcapInput(lidar_port_, gps_port_, pcap_path_, model_));
  }
  else {
    input_.reset(new SocketInput(lidar_port_, gps_port_));
  }

  if (model_ == "Pandar40P" || model_ == "Pandar40M") {
    azimuth_index_ = 2;  // 2 + 124 * [0-9]
    is_valid_packet_ = [](size_t packet_size) { return (packet_size == 1262 || packet_size == 1266); };
  }
  else if (model_ == "PandarQT") {
    azimuth_index_ = 12;  // 12 + 258 * [0-3]
    is_valid_packet_ = [](size_t packet_size) { return (packet_size == 1072); };
  }
  else if (model_ == "Pandar64") {
    azimuth_index_ = 8;  // 8 + 192 * [0-5]
    is_valid_packet_ = [](size_t packet_size) { return (packet_size == 1194 || packet_size == 1198); };
  }
  else if (model_ == "Pandar128") {
    azimuth_index_ = 12;  // 12 + 386 * [0-1]
    is_valid_packet_ = [](size_t packet_size) { return (packet_size == 812); };
  }
  else {
    ROS_ERROR("Invalid model name");
  }
}

bool PandarDriver::poll(void)
{
  int scan_phase = static_cast<int>(scan_phase_ * 100.0);
  const int min_range = static_cast<int>(min_range_ * 100.0);
  const int max_range = static_cast<int>(max_range_ * 100.0);

  if(scan_phase < min_range || scan_phase > max_range){
    scan_phase = max_range;
  }

  int debug_phase = 0;
  pandar_msgs::PandarScanPtr scan(new pandar_msgs::PandarScan);
  for (int prev_phase = 0;;) {  // finish scan
    pandar_msgs::PandarPacket packet;
    while (true) {              // until receive lidar packet
      int packet_type = input_->getPacket(&packet);
      if (packet_type == 0 && is_valid_packet_(packet.size)) {
        break;
      }
      else if (packet_type == -1) {
        return false;
      }
    }

    int current_phase = (packet.data[azimuth_index_] & 0xff) | ((packet.data[azimuth_index_ + 1] & 0xff) << 8);
    if(min_range < current_phase && current_phase < max_range){
      scan->packets.push_back(packet);
    }

    current_phase = (current_phase - scan_phase + 36000) % 36000;
    if(scan->packets.size() > 1 && prev_phase > current_phase){
      // has scanned !
      printf("%6d -> %6d -> %6d\n", prev_phase, scan_phase, current_phase);
      printf("---------------------------------\n");
      break;
    }
    prev_phase = current_phase;
  }

  scan->header.stamp = scan->packets.front().stamp;
  scan->header.frame_id = frame_id_;
  pandar_packet_pub_.publish(scan);
  return true;
}
