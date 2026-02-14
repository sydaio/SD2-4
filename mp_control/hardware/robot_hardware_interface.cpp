// Copyright 2023
// Apache 2.0 License

#include "robot_control/robot_hardware_interface.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>

namespace robot_interface
{

// ========================
// Binary packet definition
// ========================
#pragma pack(push, 1)
struct VelocityPacket
{
  uint8_t header;
  int16_t v[4];
};
#pragma pack(pop)

// ========================
// Lifecycle: init
// ========================
CallbackReturn Robot7DoF::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_velocities_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);

  serial_fd_ = -1;

  return CallbackReturn::SUCCESS;
}

// ========================
// Lifecycle: configure
// ========================
CallbackReturn Robot7DoF::on_configure(
  const rclcpp_lifecycle::State &)
{
  const char * port = "/dev/ttyUSB0";

  serial_fd_ = open(port, O_RDWR | O_NOCTTY | O_SYNC);
  if (serial_fd_ < 0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("Robot7DoF"),
      "Failed to open serial port %s", port);
    return CallbackReturn::ERROR;
  }

  termios tty{};
  tcgetattr(serial_fd_, &tty);

  cfsetospeed(&tty, B115200);
  cfsetispeed(&tty, B115200);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~(PARENB | PARODD);
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  tty.c_iflag = 0;
  tty.c_oflag = 0;
  tty.c_lflag = 0;

  tcsetattr(serial_fd_, TCSANOW, &tty);

  RCLCPP_INFO(
    rclcpp::get_logger("Robot7DoF"),
    "Serial port %s opened at 115200 baud", port);

  return CallbackReturn::SUCCESS;
}

// ========================
// State interfaces
// ========================
std::vector<hardware_interface::StateInterface>
Robot7DoF::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_positions_[i]);

    state_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_VELOCITY,
      &hw_velocities_[i]);
  }

  return state_interfaces;
}

// ========================
// Command interfaces
// ========================
std::vector<hardware_interface::CommandInterface>
Robot7DoF::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_VELOCITY,
      &hw_commands_[i]);
  }

  return command_interfaces;
}

// ========================
// Read (no feedback yet)
// ========================
hardware_interface::return_type
Robot7DoF::read(
  const rclcpp::Time &,
  const rclcpp::Duration & period)
{
  for (size_t i = 0; i < hw_positions_.size(); i++)
  {
    hw_velocities_[i] = hw_commands_[i];
    hw_positions_[i] += hw_velocities_[i] * period.seconds();
  }

  return hardware_interface::return_type::OK;
}

// ========================
// Write (BINARY SERIAL SEND)
// ========================
hardware_interface::return_type
Robot7DoF::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  if (serial_fd_ < 0)
  {
    return hardware_interface::return_type::ERROR;
  }

  VelocityPacket pkt;
  pkt.header = 0xAA;

  constexpr double SCALE = 100.0;  // velocity → int16

  for (size_t i = 0; i < 4; i++)
  {
    double cmd = hw_commands_[i];

    if (cmd >  327.67) cmd =  327.67;
    if (cmd < -327.68) cmd = -327.68;

    pkt.v[i] = static_cast<int16_t>(cmd * SCALE);
  }

  ssize_t written = ::write(serial_fd_, &pkt, sizeof(pkt));

  if (written != sizeof(pkt))
  {
    RCLCPP_WARN(
      rclcpp::get_logger("Robot7DoF"),
      "Serial write incomplete (%ld bytes)", written);
  }

  return hardware_interface::return_type::OK;
}

}  // namespace robot_interface

// ========================
// Plugin export
// ========================
PLUGINLIB_EXPORT_CLASS(
  robot_interface::Robot7DoF,
  hardware_interface::SystemInterface)
