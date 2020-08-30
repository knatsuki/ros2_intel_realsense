// Copyright (c) 2019 Intel Corporation. All Rights Reserved
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "realsense/rs_factory.hpp"

#include <memory>
#include <string>

#include "lifecycle_msgs/msg/state.hpp"
#include "realsense/rs_d435.hpp"
#include "realsense/rs_d435i.hpp"
#include "realsense/rs_t265.hpp"
#include "opencv2/opencv.hpp"

namespace {
}

namespace realsense
{

RealSenseNodeFactory::RealSenseNodeFactory(const rclcpp::NodeOptions & node_options)
: LifecycleNode(NODE_NAME, node_options), rs_node_{nullptr}, ctx_{nullptr}, dev_{nullptr}, pipeline_{nullptr}
{
  auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
  param_desc.read_only = true;
  auto param_value = declare_parameter("serial_no");
}

void RealSenseNodeFactory::initializeNode()
{
  std::string pid_str = dev_.get_info(RS2_CAMERA_INFO_PRODUCT_ID);
  uint16_t pid = std::stoi(pid_str, 0, 16);
  switch (pid) {
    case RS435_RGB_PID:
    case RS415_PID:
    case RS_USB2_PID:
      RCLCPP_INFO(this->get_logger(), "Create a node for D4X5 Camera");
      rs_node_ = std::make_unique<RealSenseD435>(ctx_, dev_, pipeline_, *this);
      break;
    case RS435i_RGB_PID:
      RCLCPP_INFO(this->get_logger(), "Create a node for D435i Camera");
      rs_node_ = std::make_unique<RealSenseD435I>(ctx_, dev_, pipeline_, *this);
      break;
    case RS_T265_PID:
      // Resetting hardware is necessary for the t265 device to start properly.
      RCLCPP_INFO(this->get_logger(),
                  "Resetting device with serial number %s. Wait for 3 second...",
                  dev_.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
      dev_.hardware_reset();
      std::this_thread::sleep_for(std::chrono::seconds(3));
      RCLCPP_INFO(this->get_logger(), "Resetting device with serial number %s. Done.",
        dev_.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));

      RCLCPP_INFO(this->get_logger(), "Create a node for T265 Camera");
      rs_node_ = std::make_unique<RealSenseT265>(ctx_, dev_, pipeline_, *this);
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unsupported device! Product ID: 0x%s", pid_str.data());
      throw UnsupportedDeviceError();
  }
}

void RealSenseNodeFactory::changeDeviceCallback(rs2::event_information & info)
{
  if (info.was_removed(dev_)) {
    RCLCPP_ERROR(this->get_logger(), "The device has been disconnected! Shutting down.");
    this->shutdown();
  }
}

void RealSenseNodeFactory::getDevice(rs2::device_list & list)
{
  if (!dev_) {
    if (0 == list.size()) {
      RCLCPP_ERROR(this->get_logger(), "No RealSense devices were found!");
    } else {
      bool found = false;
      for (auto && dev : list) {
        auto sn = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        RCLCPP_INFO(this->get_logger(), "Device with serial number %s was found.", sn);
        if (serial_no_.empty() || sn == serial_no_) {
          dev_ = dev;
          serial_no_ = sn;
          found = true;
          break;
        }
      }
      if (!found) {
        // T265 could be caught by another node.
        RCLCPP_ERROR(
          this->get_logger(), "The Device with serial number %s is not found. Please connect it.",
          serial_no_.c_str());
      }
    }
    // https://github.com/IntelRealSense/librealsense/pull/3339
    bool remove_tm2_handle(dev_ && RS_T265_PID != std::stoi(
        dev_.get_info(
          RS2_CAMERA_INFO_PRODUCT_ID), 0, 16));
    if (remove_tm2_handle) {
      ctx_.unload_tracking_module();
    }
  }
}

using rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;

LifecycleNodeInterface::CallbackReturn
RealSenseNodeFactory::on_configure(
    const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Configuring Realsense Node1");

  ctx_ = rs2::context();
  pipeline_ = rs2::pipeline(ctx_);

  auto serial_no_param = get_parameter("serial_no");
  if (serial_no_param.get_type() == rclcpp::PARAMETER_NOT_SET) {
    RCLCPP_ERROR(
        this->get_logger(), "Please set serial number param: 'serial_no'.");

    return LifecycleNodeInterface::CallbackReturn::FAILURE;
  } else {
    serial_no_ = std::to_string(serial_no_param.get_value<rclcpp::PARAMETER_INTEGER>());
  }

  try {
    using std::chrono::steady_clock;
    std::chrono::milliseconds TIMESPAN(600);
    const auto time_limit = steady_clock::now() + std::chrono::seconds(5);
    while (!dev_) {
      if (steady_clock::now() > time_limit) {
        RCLCPP_ERROR(get_logger(), "Timed out waiting for a device.");
        return LifecycleNodeInterface::CallbackReturn::FAILURE;
      }

      auto dev_lst = ctx_.query_devices();
      getDevice(dev_lst);

      if (!dev_) { std::this_thread::sleep_for(TIMESPAN); }
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(this->get_logger(), "Failed to find device: An exception has been thrown: %s", ex.what());
    return LifecycleNodeInterface::CallbackReturn::FAILURE;
  } catch (...) {
    RCLCPP_ERROR(this->get_logger(), "Failed to find device: Unknown exception has occured!");
    return LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  std::function<void(rs2::event_information &)> changeDeviceCallback_function =
      [this](rs2::event_information & info) {changeDeviceCallback(info);};
  ctx_.set_devices_changed_callback(changeDeviceCallback_function);

  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/**
 * \brief Activate ros2 publishers and the pipeline for librealsense. Currently
 *        the lifecycle is set up so that the publisher and pipeline configuration
 *        will be initialized from scratch on each activation and deallocated when
 *        deactivated. This design choice is driven to avoid not having to set up
 *        multiple on-parameter change hook for each camera parameters, keeping
 *        the overall design simple.
 * \param previous_state
 * \return
 */
LifecycleNodeInterface::CallbackReturn
RealSenseNodeFactory::on_activate(
    const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Activating Realsense Node");

  try {
    initializeNode();
    rs_node_->activatePublishers();
    rs_node_->startPipeline();
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(this->get_logger(), "Failed to activate. An exception has been thrown: %s", ex.what());
    return LifecycleNodeInterface::CallbackReturn::FAILURE;
  } catch (...) {
    RCLCPP_ERROR(this->get_logger(), "Failed to activate. Unknown exception has occured!");
    return LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RealSenseNodeFactory::on_deactivate(
    const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Dectivating Realsense Node");

  // Calling reset will stop the pipeline in addition to deallocating all the
  // publisher resource.
  rs_node_.reset();

  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RealSenseNodeFactory::on_cleanup(
    const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Cleaning up Realsense Node");

  resetState();

  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RealSenseNodeFactory::on_shutdown(
    const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Shutting down Realsense Node");

  resetState();

  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RealSenseNodeFactory::on_error(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Error processing Realsense Node");

  resetState();

  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void RealSenseNodeFactory::resetState() {
  rs_node_.reset();
  dev_.hardware_reset();
  dev_ = rs2::device(nullptr);
  ctx_ = rs2::context(nullptr);
  pipeline_ = rs2::pipeline(nullptr);
  serial_no_ = "";
}
}  // namespace realsense

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(realsense::RealSenseNodeFactory)
