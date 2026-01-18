// Copyright 2023 ros2_control Development Team
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

#include "robot_control/robot_hardware_interface.hpp"
#include <string>
#include <vector>

#include <iostream>

namespace robot_interface
{
CallbackReturn Robot7DoF::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }
  test_parameter_1_ = std::stod(info_.hardware_parameters["testParam"])
  return hardware_interface::CallbackReturn::SUCCESS;
}

CallbackReturn Robot7DoF::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // reset values always when configuring hardware
  for (const auto & [name, descr] : joint_state_interfaces_)
  {
    set_state(name, 0.0);
  }
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    set_command(name, 0.0);
  }
  for (const auto & [name, descr] : sensor_state_interfaces_)
  {
    set_state(name, 0.0);
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> export_state_interfaces()
{
std::vector<hardware_interface::StateInterface> state_interfaces;
  for(auto i=0u; i<info_.joints; i++) 
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::StateInterface> export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
    for(auto i=0u; i<info_.joints; i++) 
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_command_[i]));
    }
    return command_interfaces;
}

return_type Robot7DoF::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // TODO(pac48) set sensor_states_ values from subscriber

  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    const auto name_vel = info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY;
    const auto name_pos = info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION;
    set_state(name_vel, get_command(name_vel));
    set_state(name_pos, get_state(name_pos) + get_state(name_vel) * period.seconds());
  }
  return return_type::OK;
}

return_type Robot7DoF::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  return return_type::OK;
}

}  // namespace robot_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    robot_interface::Robot7DoF, hardware_interface::SystemInterface)
