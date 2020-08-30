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

#ifndef REALSENSE__RS_FACTORY_HPP_
#define REALSENSE__RS_FACTORY_HPP_

#include <memory>
#include <string>
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "librealsense2/rs.hpp"
#include "realsense/rs_constants.hpp"
#include "realsense/rs_base.hpp"

namespace realsense
{
class RealSenseNodeFactory : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit RealSenseNodeFactory(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  ~RealSenseNodeFactory() override = default;

  CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn
  on_error(const rclcpp_lifecycle::State &previous_state) override;

private:
  void initializeNode();
  void changeDeviceCallback(rs2::event_information & info);
  void getDevice(rs2::device_list & list);
  void resetState();

  std::unique_ptr<RealSenseBase> rs_node_;
  rs2::context ctx_;
  rs2::device dev_;
  rs2::pipeline pipeline_;
  std::string serial_no_;
};
}  // namespace realsense

#endif  // REALSENSE__RS_FACTORY_HPP_
