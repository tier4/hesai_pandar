#include "pandar_pointcloud/decoder/pandar40_decoder.hpp"
#include "pandar_pointcloud/decoder/pandar40.hpp"

namespace
{
static inline double deg2rad(double degrees)
{
  return degrees * M_PI / 180.0;
}
}  // namespace

namespace pandar_pointcloud
{
namespace pandar40
{
Pandar40Decoder::Pandar40Decoder(rclcpp::Node &node, Calibration &calibration, double scan_phase,
                                 const std::vector<double> &angle_range,
                                 const std::vector<double> &distance_range,
                                 double dual_return_distance_threshold, ReturnMode return_mode)
    : logger_(node.get_logger()), clock_(node.get_clock())
{
  firing_order_ = {7, 19, 14, 26, 6, 18, 4, 32, 36, 0, 10, 22, 17, 29, 9, 21, 5, 33, 37, 1,
                   13, 25, 20, 30, 12, 8, 24, 34, 38, 2, 16, 28, 23, 31, 15, 11, 27, 35, 39, 3};

  firing_offset_ = {42.22, 28.47, 16.04, 3.62, 45.49, 31.74, 47.46, 54.67, 20.62, 33.71, 40.91, 8.19, 20.62, 27.16,
                    50.73, 8.19, 14.74, 36.98, 45.49, 52.7, 23.89, 31.74, 38.95, 11.47, 18.65, 25.19, 48.76, 6.23,
                    12.77, 35.01, 21.92, 9.5, 43.52, 29.77, 17.35, 4.92, 42.22, 28.47, 16.04, 3.62};

  for (size_t block = 0; block < BLOCKS_PER_PACKET; ++block)
  {
    block_offset_single_[block] = 55.56f * (BLOCKS_PER_PACKET - block - 1) + 28.58f;
    block_offset_dual_[block] = 55.56f * ((BLOCKS_PER_PACKET - block - 1) / 2) + 28.58f;
  }

  // TODO: add calibration data validation
  // if(calibration.elev_angle_map.size() != num_lasers_){
  //   // calibration data is not valid!
  // }

  for (size_t laser = 0; laser < LASER_COUNT; ++laser) {
    azimuth_offset_[laser] = static_cast<int32_t>(std::round(calibration.azimuth_offset_map[laser] * 100.0));
    double elev_angle = deg2rad(calibration.elev_angle_map[laser]);
    elev_sin_table_.push_back(std::sin(elev_angle));
    elev_cos_table_.push_back(std::cos(elev_angle));
  }

  azim_cos_table_.resize(MAX_AZIMUTH, 0.0);
  azim_sin_table_.resize(MAX_AZIMUTH, 0.0);
  for (size_t azimuth_idx = 0; azimuth_idx < MAX_AZIMUTH; ++azimuth_idx) {
    double azimuth = deg2rad(AZIMUTH_RESOLUTION * azimuth_idx);
    azim_sin_table_[azimuth_idx] = std::sin(azimuth);
    azim_cos_table_[azimuth_idx] = std::cos(azimuth);
  }

  scan_phase_ = static_cast<int>(scan_phase * 100.0);
  angle_range_ = {static_cast<int>(angle_range[0] * 100.0), static_cast<int>(angle_range[1] * 100.0)};
  distance_range_ = distance_range;
  return_mode_ = return_mode;
  dual_return_distance_threshold_ = dual_return_distance_threshold;

  int max_angle = (angle_range_[1] - angle_range_[0] + 36000) % 36000;
  int scan_angle = (scan_phase_ - angle_range_[0] + 36000) % 36000;

  if(max_angle == 0 || scan_angle < max_angle){
    use_overflow_ = true;
  }else{
    use_overflow_ = false;
  }

  last_phase_ = 0;
  has_scanned_ = false;
  reset_scan_ = false;

  scan_pc_.reset(new pcl::PointCloud<PointXYZIRADT>);
  overflow_pc_.reset(new pcl::PointCloud<PointXYZIRADT>);
  }

bool Pandar40Decoder::hasScanned()
{
  return has_scanned_;
}

PointcloudXYZIRADT Pandar40Decoder::getPointcloud()
{
  reset_scan_ = true;
  return scan_pc_;
}

void Pandar40Decoder::unpack(const pandar_msgs::msg::PandarPacket& raw_packet)
{
  if (!parsePacket(raw_packet)) {
    return;
  }

  bool dual_return = (packet_.return_mode == DUAL_RETURN);

  if (!dual_return) {
    if ((packet_.return_mode == STRONGEST_RETURN && return_mode_ != ReturnMode::STRONGEST) || 
        (packet_.return_mode == LAST_RETURN && return_mode_ != ReturnMode::LAST)) {
      RCLCPP_WARN(logger_, "Sensor return mode configuration does not match requested return mode");
    }
  }

  auto step = dual_return ? 2 : 1;

  if (reset_scan_) {
    if(use_overflow_){
      scan_pc_ = overflow_pc_;
      overflow_pc_.reset(new pcl::PointCloud<PointXYZIRADT>);
      has_scanned_ = false;
      reset_scan_ = false;
    }else{
      scan_pc_.reset(new pcl::PointCloud<PointXYZIRADT>);
      reset_scan_ = false;
    }
  }

  if(use_overflow_){
    for (size_t block_id = 0; block_id < BLOCKS_PER_PACKET; block_id += step) {
      auto block_pc = dual_return ? convert_dual(block_id) : convert(block_id);
      int current_phase = (static_cast<int>(packet_.blocks[block_id].azimuth) - scan_phase_ + 36000) % 36000;
      if (current_phase > last_phase_ && !has_scanned_) {
        *scan_pc_ += *block_pc;
      }
      else {
        *overflow_pc_ += *block_pc;
        has_scanned_ = true;
      }
      last_phase_ = current_phase;
    }
  }else{
    for (size_t block_id = 0; block_id < BLOCKS_PER_PACKET; block_id += step) {
      auto block_pc = dual_return ? convert_dual(block_id) : convert(block_id);
      int current_phase = (static_cast<int>(packet_.blocks[block_id].azimuth) - angle_range_[0] + 36000) % 36000;
      int max_phase = (angle_range_[1] - angle_range_[0] + 36000) % 36000;
      if (current_phase < max_phase) {
        *scan_pc_ += *block_pc;
      }
    }
  }

  return;
}

PointXYZIRADT Pandar40Decoder::build_point(int block_id, int unit_id, uint8_t return_type)
{
  const auto& block = packet_.blocks[block_id];
  const auto& unit = block.units[unit_id];
  double unix_second = static_cast<double>(timegm(&packet_.t));
  bool dual_return = (packet_.return_mode == DUAL_RETURN);
  PointXYZIRADT point;

  const auto azimuth_index = (block.azimuth + azimuth_offset_[unit_id] + MAX_AZIMUTH) % MAX_AZIMUTH;
  const double xyDistance = unit.distance * elev_cos_table_[unit_id];
  point.x = xyDistance * azim_sin_table_[azimuth_index];
  point.y = xyDistance * azim_cos_table_[azimuth_index];
  point.z = unit.distance * elev_sin_table_[unit_id];

  point.intensity = unit.intensity;
  point.distance = unit.distance;
  point.ring = unit_id;
  point.azimuth = azimuth_index;
  point.return_type = return_type;
  point.time_stamp = unix_second + (static_cast<double>(packet_.usec)) / 1000000.0;

  point.time_stamp -= dual_return ? (static_cast<double>(block_offset_dual_[block_id] + firing_offset_[unit_id]) / 1000000.0f) :
                                    (static_cast<double>(block_offset_single_[block_id] + firing_offset_[unit_id]) / 1000000.0f); 

  return point;
}

PointcloudXYZIRADT Pandar40Decoder::convert(const int block_id)
{
  PointcloudXYZIRADT block_pc(new pcl::PointCloud<PointXYZIRADT>);


  const auto& block = packet_.blocks[block_id];
  for (auto unit_id : firing_order_) {
    const auto& unit = block.units[unit_id];
    // skip invalid points
    if (unit.distance <= distance_range_[0] || unit.distance > distance_range_[1]) {
      continue;
    }
    block_pc->push_back(build_point(block_id, unit_id, (packet_.return_mode == STRONGEST_RETURN) ? ReturnType::SINGLE_STRONGEST : ReturnType::SINGLE_LAST)); 
  }

  return block_pc;
}

PointcloudXYZIRADT Pandar40Decoder::convert_dual(const int block_id)
{
  //   Under the Dual Return mode, the measurements from each round of firing are stored in two adjacent blocks:
  // · The even number block is the last return, and the odd number block is the strongest return
  // · If the last and strongest returns coincide, the second strongest return will be placed in the odd number block
  // · The Azimuth changes every two blocks
  // · Important note: Hesai datasheet block numbering starts from 0, not 1, so odd/even are reversed here
  PointcloudXYZIRADT block_pc(new pcl::PointCloud<PointXYZIRADT>);

  int even_block_id = block_id;
  int odd_block_id = block_id + 1;
  const auto& even_block = packet_.blocks[even_block_id];
  const auto& odd_block = packet_.blocks[odd_block_id];

  for (auto unit_id : firing_order_) {

    const auto& even_unit = even_block.units[unit_id];
    const auto& odd_unit = odd_block.units[unit_id];

    bool even_usable = (even_unit.distance <= distance_range_[0] || even_unit.distance > distance_range_[1]) ? 0 : 1;
    bool odd_usable = (odd_unit.distance <= distance_range_[0] || odd_unit.distance > distance_range_[1]) ? 0 : 1;  

    if (return_mode_ == ReturnMode::STRONGEST) {
      // Strongest return is in even block when both returns coincide
      if (even_unit.intensity >= odd_unit.intensity && even_usable) {
        block_pc->push_back(build_point(even_block_id, unit_id, ReturnType::SINGLE_STRONGEST));        
      }
      else if (even_unit.intensity < odd_unit.intensity && odd_usable) {
        block_pc->push_back(build_point(odd_block_id, unit_id, ReturnType::SINGLE_STRONGEST)); 
      }      
    }
    else if (return_mode_ == ReturnMode::LAST && even_usable) {
      // Last return is always in even block
      block_pc->push_back(build_point(even_block_id, unit_id, ReturnType::SINGLE_LAST)); 
    }
    else if (return_mode_ == ReturnMode::DUAL) {
      // If the two returns are too close, only return the last one
      if ((abs(even_unit.distance - odd_unit.distance) < dual_return_distance_threshold_) && even_usable) {
        block_pc->push_back(build_point(even_block_id, unit_id, ReturnType::DUAL_ONLY));
      }
      else if (even_unit.intensity >= odd_unit.intensity) {
        // Strongest return is in even block when it is also the last
        if (odd_usable) {
          block_pc->push_back(build_point(odd_block_id, unit_id, ReturnType::DUAL_WEAK_FIRST));
        }
        if (even_usable) {
          block_pc->push_back(build_point(even_block_id, unit_id, ReturnType::DUAL_STRONGEST_LAST));
        }
      }
      else {
        // Normally, strongest return is in odd block and last return is in even block
        if (odd_usable) {
          block_pc->push_back(build_point(odd_block_id, unit_id, ReturnType::DUAL_STRONGEST_FIRST));
        }
        if (even_usable) {
          block_pc->push_back(build_point(even_block_id, unit_id, ReturnType::DUAL_WEAK_LAST));
        }      
      }
    }
  }
  return block_pc;
}

bool Pandar40Decoder::parsePacket(const pandar_msgs::msg::PandarPacket& raw_packet)
{
  if (raw_packet.size != PACKET_SIZE && raw_packet.size != PACKET_SIZE + SEQ_NUM_SIZE) {
    // packet size mismatch !
    return false;
  }

  // auto buf = raw_packet.data;
  const uint8_t* buf = &raw_packet.data[0];

  int index = 0;
  for (size_t i = 0; i < BLOCKS_PER_PACKET; i++) {
    Block& block = packet_.blocks[i];

    block.sob = (buf[index] & 0xff) | ((buf[index + 1] & 0xff) << 8);
    block.azimuth = (buf[index + 2] & 0xff) | ((buf[index + 3] & 0xff) << 8);
    index += SOB_ANGLE_SIZE;

    for (size_t j = 0; j < LASER_COUNT; j++) {
      Unit& unit = block.units[j];
      uint32_t range = (buf[index] & 0xff) | ((buf[index + 1] & 0xff) << 8);

      unit.distance = (static_cast<double>(range)) * LASER_RETURN_TO_DISTANCE_RATE;
      unit.intensity = (buf[index + 2] & 0xff);

      index += RAW_MEASURE_SIZE;
    }
  }

  index += RESERVE_SIZE;

  index += REVOLUTION_SIZE;

  packet_.usec = (buf[index] & 0xff) | (buf[index + 1] & 0xff) << 8 | ((buf[index + 2] & 0xff) << 16) |
                 ((buf[index + 3] & 0xff) << 24);
  packet_.usec %= 1000000;

  index += TIMESTAMP_SIZE;
  packet_.return_mode = buf[index] & 0xff;

  index += FACTORY_INFO_SIZE + RETURN_SIZE;

  packet_.t.tm_year = (buf[index + 0] & 0xff) + 100;

  // in case of time error
  if (packet_.t.tm_year >= 200) {
    packet_.t.tm_year -= 100;
  }

  packet_.t.tm_mon = (buf[index + 1] & 0xff) - 1;
  packet_.t.tm_mday = buf[index + 2] & 0xff;
  packet_.t.tm_hour = buf[index + 3] & 0xff;
  packet_.t.tm_min = buf[index + 4] & 0xff;
  packet_.t.tm_sec = buf[index + 5] & 0xff;
  packet_.t.tm_isdst = 0;

  return true;
}

}  // namespace pandar40
}  // namespace pandar_pointcloud
