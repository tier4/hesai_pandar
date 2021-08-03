#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include "pandar_driver/pandar_driver_core.h"


namespace pandar_driver
{
class PandarDriver : public rclcpp::Node
{
public:
  PandarDriver(const rclcpp::NodeOptions & options)
  : Node("pandar_driver_node", options),
    running_(false)
  {
    onInit();
  }

  ~PandarDriver()
  {
    RCLCPP_INFO(this->get_logger(), "shutting down driver thread");
    running_ = false;
    if (deviceThread_->joinable()) {
      deviceThread_->join();
    }
    RCLCPP_INFO(this->get_logger(), "driver thread stopped");
  }

private:
  virtual void onInit(void);
  virtual void devicePoll(void);

  volatile bool running_;
  std::shared_ptr<std::thread> deviceThread_;
  std::shared_ptr<PandarDriverCore> driver_;
};

void PandarDriver::onInit()
{
  driver_.reset(new PandarDriverCore(this));

  running_ = true;
  deviceThread_ = std::make_shared<std::thread>(std::bind(&PandarDriver::devicePoll, this));
}

void PandarDriver::devicePoll()
{
  while (rclcpp::ok()) {
    running_ = driver_->poll();
    if (!running_)
      break;
  }
  running_ = false;
}

}  // namespace pandar_driver

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(pandar_driver::PandarDriver)