/*
 * Copyright 2021 Tier IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include "pandar_monitor/pandar_monitor.hpp"
#include "boost/algorithm/string/join.hpp"

#define FMT_HEADER_ONLY
#include "fmt/format.h"

namespace pandar_monitor
{
PandarMonitor::PandarMonitor(const rclcpp::NodeOptions & options)
: Node("pandar_monitor"),
  updater_(this)
{
  timeout_ = declare_parameter("timeout", 0.5);
  ip_address_ = declare_parameter("ip_address", "192.168.1.201");
  temp_cold_warn_ = declare_parameter("temp_cold_warn", -5.0);
  temp_cold_error_ = declare_parameter("temp_cold_error", -10.0);
  temp_hot_warn_ = declare_parameter("temp_hot_warn", 75.0);
  temp_hot_error_ = declare_parameter("temp_hot_error", 80.0);
  rpm_ratio_warn_ = declare_parameter("rpm_ratio_warn", 0.80);
  rpm_ratio_error_ = declare_parameter("rpm_ratio_error", 0.70);

  updater_.add(std::string(this->get_namespace()) + ": pandar_connection", this, &PandarMonitor::checkConnection);
  updater_.add(std::string(this->get_namespace()) + ": pandar_temperature", this, &PandarMonitor::checkTemperature);
  updater_.add(std::string(this->get_namespace()) + ": pandar_ptp", this, &PandarMonitor::checkPTP);

  client_ = std::make_unique<pandar_api::TCPClient>(ip_address_, static_cast<int>(timeout_ * 1000));

  updater_.setHardwareID("pandar");
}

void PandarMonitor::checkConnection(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  pandar_api::InventoryInfo info;
  auto code = client_->getInventoryInfo(info);
  if(code != pandar_api::TCPClient::ReturnCode::SUCCESS){
    stat.summary(DiagStatus::ERROR, "ERROR");
    return;
  }

  updater_.setHardwareIDf(
    "%s: %s", info.model.c_str(), info.sn.c_str());

  stat.summary(DiagStatus::OK, "OK");
}

void PandarMonitor::checkTemperature(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  pandar_api::LidarStatus status;
  auto code = client_->getLidarStatus(status);

  if(code != pandar_api::TCPClient::ReturnCode::SUCCESS){
    stat.summary(DiagStatus::ERROR, "ERROR");
    return;
  }

  int error = DiagStatus::OK;
  int warn = DiagStatus::OK;
  std::vector<std::string> msg;  

  for(size_t i = 0; i < 8; ++i){
    float temp = static_cast<float>(status.temp[i]) / 100.0f;
    auto pos = position_[i];
    stat.addf(position_[i], "%.2lf DegC", temp);

    // Check board temperature
    if (temp < temp_cold_error_) {
      error = DiagStatus::ERROR;
      msg.emplace_back(fmt::format("{} temperature too cold", pos));
    } else if (temp < temp_cold_warn_) {
      warn = DiagStatus::WARN;
      msg.emplace_back(fmt::format("{} temperature cold", pos));
    } else if (temp > temp_hot_error_) {
      error = DiagStatus::ERROR;
      msg.emplace_back(fmt::format("{} temperature too hot", pos));
    } else if (temp > temp_hot_warn_) {
      warn = DiagStatus::WARN;
      msg.emplace_back(fmt::format("{} temperature hot", pos));
    }
  }

  if (msg.empty()) msg.emplace_back("OK");
  stat.summary(std::max(error, warn), boost::algorithm::join(msg, ", "));
}

void PandarMonitor::checkPTP(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  pandar_api::LidarStatus status;
  auto code = client_->getLidarStatus(status);
  if(code != pandar_api::TCPClient::ReturnCode::SUCCESS){
    stat.summary(DiagStatus::ERROR, "ERROR");
    return;
  }

  int level = DiagStatus::OK;
  if(status.ptp_clock_status == 0 || status.ptp_clock_status == 3){
    level = DiagStatus::WARN;
  }
  stat.summary(level, ptp_[status.ptp_clock_status]);
}
}  // namespace pandar_monitor
