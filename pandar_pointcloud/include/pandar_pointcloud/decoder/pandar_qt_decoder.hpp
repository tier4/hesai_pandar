#pragma once

#include <array>
#include "pandar_pointcloud/calibration.hpp"
#include "packet_decoder.hpp"
#include "pandar_qt.hpp"

static const float QT_ROTATION_RESOLUTION = 0.001f;
static const uint32_t QT_ROTATION_MAX_UNITS = 360000u;

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

  PandarQTDecoder(Calibration& calibration, float scan_phase = 0.0f, ReturnMode return_mode = ReturnMode::DUAL);
  void unpack(const pandar_msgs::msg::PandarPacket& raw_packet) override;
  bool hasScanned() override;
  PointcloudXYZIRADT getPointcloud() override;

private:
  bool parsePacket(const pandar_msgs::msg::PandarPacket& raw_packet);
  PointcloudXYZIRADT convert(const int block_id);
  PointcloudXYZIRADT convert_dual(const int block_id);

  std::array<int32_t, UNIT_NUM> elev_angle_;
  std::array<int32_t, UNIT_NUM> azimuth_offset_;

  std::array<float, UNIT_NUM> firing_offset_;
  std::array<float, BLOCK_NUM> block_offset_single_;
  std::array<float, BLOCK_NUM> block_offset_dual_;

  ReturnMode return_mode_;
  Packet packet_;

  PointcloudXYZIRADT scan_pc_;
  PointcloudXYZIRADT overflow_pc_;

  uint16_t scan_phase_;
  int last_phase_;
  bool has_scanned_;

  double sin_rot_table_[QT_ROTATION_MAX_UNITS];
  double cos_rot_table_[QT_ROTATION_MAX_UNITS];

};

}  // namespace pandar_qt
}  // namespace pandar_pointcloud
