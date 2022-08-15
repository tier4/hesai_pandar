#pragma once

#include <array>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/mat.hpp>
#include "pandar_pointcloud/calibration.hpp"
#include "packet_decoder.hpp"
#include "pandar_qt.hpp"

namespace pandar_pointcloud
{
namespace pandar_qt
{
class ExpoNullNullPandarQTDecoder : public PacketDecoder
{
public:
  enum class ReturnMode : int8_t
  {
    DUAL,
    FIRST,
    LAST,
  };
  enum class RunMode : int8_t
  {
    NORMAL,
    MAP,
    SUBTRACT,
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

  ExpoNullNullPandarQTDecoder(rclcpp::Node & node, Calibration& calibration, float scan_phase = 0.0f, float min_angle = 0.0, float max_angle = 360.0, double dual_return_distance_threshold = 0.1, ReturnMode return_mode = ReturnMode::DUAL, RunMode run_mode = RunMode::NORMAL, std::string background_map_path = "");
  void unpack(const pandar_msgs::msg::PandarPacket& raw_packet) override;
  PointXYZIRADT build_point(int block_id, int unit_id, uint8_t return_type);
  bool hasScanned() override;
  PointcloudXYZIRADT getPointcloud() override;
  PointcloudXYZIRADT getBackgroundPointcloud();
  PointcloudXYZIRADT getObjectsPointcloud();

private:
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  
  bool parsePacket(const pandar_msgs::msg::PandarPacket& raw_packet);
  void convert(const int block_id, bool overflow);
  void convert_dual(const int block_id, bool overflow);

  std::array<float, pandar_qt::UNIT_NUM> elev_angle_;
  std::array<float, pandar_qt::UNIT_NUM> azimuth_offset_;

  std::array<float, pandar_qt::UNIT_NUM> firing_offset_;
  std::array<float, pandar_qt::BLOCK_NUM> block_offset_single_;
  std::array<float, pandar_qt::BLOCK_NUM> block_offset_dual_;

  ReturnMode return_mode_;
  RunMode run_mode_;
  double dual_return_distance_threshold_;
  pandar_qt::Packet packet_;

  PointcloudXYZIRADT scan_pc_;
  PointcloudXYZIRADT overflow_pc_;
  PointcloudXYZIRADT background_pc_;
  PointcloudXYZIRADT objects_pc_;
  PointcloudXYZIRADT background_overflow_pc_;
  PointcloudXYZIRADT objects_overflow_pc_;

  uint16_t scan_phase_;
  float min_angle_;
  float max_angle_;
  int last_phase_;
  bool has_scanned_;
  uint16_t scan_counter_;
  std::string background_map_path_;
  cv::Mat map_image_;
  uint16_t num_map_samples_[64][600];
  float map_[64][600];
};

}  // namespace pandar_qt
}  // namespace pandar_pointcloud
