#pragma once

#include <array>
#include <rclcpp/rclcpp.hpp>
#include "pandar_pointcloud/calibration.hpp"
#include "packet_decoder.hpp"
#include "pandar40.hpp"

namespace pandar_pointcloud
{
namespace pandar40
{
class Pandar40Decoder : public PacketDecoder
{
public:
  enum class ReturnMode : int8_t
  {
    DUAL,
    STRONGEST,
    LAST,
  };
  enum ReturnType : uint8_t
  {
    INVALID = 0,
    SINGLE_STRONGEST,
    SINGLE_LAST,
    DUAL_STRONGEST_FIRST,
    DUAL_STRONGEST_LAST,
    DUAL_WEAK_FIRST,
    DUAL_WEAK_LAST,
    DUAL_ONLY,
  };

  Pandar40Decoder(rclcpp::Node &node,
                  Calibration &calibration,
                  double scan_phase,
                  const std::vector<double> &angle_range,
                  const std::vector<double> &distance_range,
                  double dual_return_distance_threshold = 0.1,
                  ReturnMode return_mode = ReturnMode::DUAL);
  void unpack(const pandar_msgs::msg::PandarPacket& raw_packet) override;
  bool hasScanned() override;
  PointcloudXYZIRADT getPointcloud() override;

private:
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;

  bool parsePacket(const pandar_msgs::msg::PandarPacket& raw_packet);
  PointXYZIRADT build_point(int block_id, int unit_id, uint8_t return_type);
  PointcloudXYZIRADT convert(const int block_id);
  PointcloudXYZIRADT convert_dual(const int block_id);

  std::array<int32_t, LASER_COUNT> azimuth_offset_;
  std::vector<double> elev_sin_table_;
  std::vector<double> elev_cos_table_;
  std::vector<double> azim_sin_table_;
  std::vector<double> azim_cos_table_;

  std::array<float, LASER_COUNT> firing_offset_;
  std::array<float, BLOCKS_PER_PACKET> block_offset_single_;
  std::array<float, BLOCKS_PER_PACKET> block_offset_dual_;

  std::array<size_t, LASER_COUNT> firing_order_;

  std::vector<int> angle_range_;
  std::vector<double> distance_range_;
  ReturnMode return_mode_;
  double dual_return_distance_threshold_;
  Packet packet_;

  PointcloudXYZIRADT scan_pc_;
  PointcloudXYZIRADT overflow_pc_;

  int scan_phase_;
  int last_phase_;
  bool has_scanned_;
  bool use_overflow_;
  bool reset_scan_;
};

}  // namespace pandar40
}  // namespace pandar_pointcloud