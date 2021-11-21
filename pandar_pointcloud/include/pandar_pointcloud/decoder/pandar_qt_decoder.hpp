#pragma once

#include <array>
#include <rclcpp/rclcpp.hpp>
#include "pandar_pointcloud/calibration.hpp"
#include "packet_decoder.hpp"
#include "pandar_qt.hpp"

namespace pandar_pointcloud
{
namespace pandar_qt
{
class PandarQTDecoder : public PacketDecoder
{
public:
  enum class ReturnMode : int8_t
  {
    DUAL,
    FIRST,
    LAST,
  };
  enum ReturnType : uint8_t
  {
    INVALID = 0,
    SINGLE_FIRST,
    SINGLE_LAST,
    DUAL_FIRST,
    DUAL_LAST,
    DUAL_ONLY,
  };

  PandarQTDecoder(rclcpp::Node &node,
                  Calibration &calibration,
                  double scan_phase,
                  const std::vector<double> &angle_range,
                  const std::vector<double> &distance_range,
                  double dual_return_distance_threshold = 0.1,
                  ReturnMode return_mode = ReturnMode::DUAL);
  void unpack(const pandar_msgs::msg::PandarPacket& raw_packet) override;
  PointXYZIRADT build_point(int block_id, int unit_id, uint8_t return_type);
  bool hasScanned() override;
  PointcloudXYZIRADT getPointcloud() override;

private:
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  
  bool parsePacket(const pandar_msgs::msg::PandarPacket& raw_packet);
  PointcloudXYZIRADT convert(const int block_id);
  PointcloudXYZIRADT convert_dual(const int block_id);

  // std::array<float, UNIT_NUM> elev_angle_;
  std::array<int32_t, UNIT_NUM> azimuth_offset_;

  std::vector<double> elev_sin_table_;
  std::vector<double> elev_cos_table_;
  std::vector<double> azim_sin_table_;
  std::vector<double> azim_cos_table_;

  std::array<float, UNIT_NUM> firing_offset_;
  std::array<float, BLOCK_NUM> block_offset_single_;
  std::array<float, BLOCK_NUM> block_offset_dual_;

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

}  // namespace pandar_qt
}  // namespace pandar_pointcloud
