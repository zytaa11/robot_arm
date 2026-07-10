#include "robot_hardware_interface/usb_serial_hardware.hpp"

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <sstream>
#include <string>
#include <vector>

// Linux 串口头文件
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace robot_hardware_interface
{

// ===================================================================
//  内部工具函数
// ===================================================================
namespace
{

rclcpp::Logger logger()
{
  return rclcpp::get_logger("USBSerialHardware");
}

speed_t to_speed_t(int baud)
{
  switch (baud)
  {
    case 9600:    return B9600;
    case 19200:   return B19200;
    case 38400:   return B38400;
    case 57600:   return B57600;
    case 115200:  return B115200;
    case 230400:  return B230400;
    case 460800:  return B460800;
    case 921600:  return B921600;
    default:      return B115200;
  }
}

}  // namespace

// ===================================================================
//  构造 / 析构
// ===================================================================

USBSerialHardware::~USBSerialHardware()
{
  close_serial();
}

// ===================================================================
//  on_init — 读取参数
// ===================================================================

hardware_interface::CallbackReturn USBSerialHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != NUM_JOINTS)
  {
    RCLCPP_ERROR(logger(), "Expected %zu joints, got %zu", NUM_JOINTS, info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 从 URDF <hardware><param> 读取串口参数
  const auto & hp = info_.hardware_parameters;
  if (hp.count("serial_port")) serial_port_ = hp.at("serial_port");
  if (hp.count("baud_rate")) baud_rate_ = std::stoi(hp.at("baud_rate"));
  if (hp.count("protocol")) protocol_ = hp.at("protocol");

  // 读取关节配置（可选）
  for (size_t i = 0; i < NUM_JOINTS; ++i)
  {
    const auto & jp = info_.joints[i].parameters;
    if (jp.count("sign")) signs_[i] = std::stoi(jp.at("sign"));
  }

  // 初始化状态
  for (size_t i = 0; i < NUM_JOINTS; ++i)
  {
    hw_positions_[i] = 0.0;
    hw_velocities_[i] = 0.0;
    hw_commands_[i] = 0.0;
  }

  RCLCPP_INFO(logger(), "Initialized: port=%s, baud=%d, protocol=%s",
              serial_port_.c_str(), baud_rate_, protocol_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ===================================================================
//  on_configure — 打开串口
// ===================================================================

hardware_interface::CallbackReturn USBSerialHardware::on_configure(
  const rclcpp_lifecycle::State &)
{
  if (!open_serial())
  {
    RCLCPP_ERROR(logger(), "Failed to open serial port: %s", serial_port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(logger(), "Serial port opened: %s", serial_port_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ===================================================================
//  on_activate / on_deactivate
// ===================================================================

hardware_interface::CallbackReturn USBSerialHardware::on_activate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger(), "Hardware activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn USBSerialHardware::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger(), "Hardware deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ===================================================================
//  接口注册
// ===================================================================

std::vector<hardware_interface::StateInterface> USBSerialHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (size_t i = 0; i < NUM_JOINTS; ++i)
  {
    interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> USBSerialHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (size_t i = 0; i < NUM_JOINTS; ++i)
  {
    interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
  }
  return interfaces;
}

// ===================================================================
//  read — 从串口读取反馈（可选）
// ===================================================================

hardware_interface::return_type USBSerialHardware::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // 如果 MCU 回传当前位置，在这里解析
  // 没有回传则保持上次值
  read_feedback();
  return hardware_interface::return_type::OK;
}

// ===================================================================
//  write — 将关节命令通过串口发出 ★ 核心函数
// ===================================================================

hardware_interface::return_type USBSerialHardware::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  bool ok = false;

  if (protocol_ == "binary")
  {
    ok = write_binary(hw_commands_);
  }
  else
  {
    ok = write_ascii(hw_commands_);
  }

  if (!ok)
  {
    RCLCPP_WARN(logger(), "Serial write failed");
    return hardware_interface::return_type::ERROR;
  }

  // 将命令回填到 state，使 joint_state_publisher 显示正确
  for (size_t i = 0; i < NUM_JOINTS; ++i)
  {
    hw_positions_[i] = hw_commands_[i];
  }

  return hardware_interface::return_type::OK;
}

// ===================================================================
//  串口操作
// ===================================================================

bool USBSerialHardware::open_serial()
{
  fd_ = ::open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (fd_ < 0)
  {
    RCLCPP_ERROR(logger(), "open(%s) failed: %s", serial_port_.c_str(), strerror(errno));
    return false;
  }

  struct termios tty;
  if (tcgetattr(fd_, &tty) != 0)
  {
    RCLCPP_ERROR(logger(), "tcgetattr failed: %s", strerror(errno));
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  cfsetospeed(&tty, to_speed_t(baud_rate_));
  cfsetispeed(&tty, to_speed_t(baud_rate_));

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;  // 8-bit
  tty.c_iflag &= ~IGNBRK;
  tty.c_lflag = 0;
  tty.c_oflag = 0;
  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 5;  // 500ms read timeout

  tty.c_iflag &= ~(IXON | IXOFF | IXANY | INLCR | ICRNL);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~(PARENB | PARODD);
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  if (tcsetattr(fd_, TCSANOW, &tty) != 0)
  {
    RCLCPP_ERROR(logger(), "tcsetattr failed: %s", strerror(errno));
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  return true;
}

void USBSerialHardware::close_serial()
{
  if (fd_ >= 0)
  {
    ::close(fd_);
    fd_ = -1;
  }
}

bool USBSerialHardware::write_ascii(const std::array<double, NUM_JOINTS> & positions)
{
  if (fd_ < 0) return false;

  // 格式: "J1:1.234 J2:-0.567 J3:0.890 ...\n"
  std::stringstream ss;
  ss.precision(4);
  ss.setf(std::ios::fixed);

  for (size_t i = 0; i < NUM_JOINTS; ++i)
  {
    ss << "J" << (i + 1) << ":"
       << (signs_[i] * positions[i])  // 方向修正
       << " ";
  }
  ss << "\n";

  std::string frame = ss.str();
  size_t bytes_written = 0;
  while (bytes_written < frame.size())
  {
    ssize_t ret = ::write(fd_, frame.data() + bytes_written, frame.size() - bytes_written);
    if (ret < 0)
    {
      if (errno == EINTR || errno == EAGAIN) continue;
      return false;
    }
    bytes_written += static_cast<size_t>(ret);
  }

  return true;
}

bool USBSerialHardware::write_binary(const std::array<double, NUM_JOINTS> & positions)
{
  if (fd_ < 0) return false;

  // 简单二进制协议:
  // [0xAA] [0x07] [joint_1 rad * 1e4 as int16] ... [joint_7] [XOR checksum]
  // 共 1 + 1 + 7*2 + 1 = 17 字节
  std::vector<uint8_t> frame;
  frame.reserve(17);
  frame.push_back(0xAA);         // SOF
  frame.push_back(NUM_JOINTS);   // 关节数

  uint8_t checksum = 0xAA ^ NUM_JOINTS;
  for (size_t i = 0; i < NUM_JOINTS; ++i)
  {
    int16_t val = static_cast<int16_t>(
      std::clamp(signs_[i] * positions[i] * 1e4, -32768.0, 32767.0));
    frame.push_back(static_cast<uint8_t>(val & 0xFF));
    frame.push_back(static_cast<uint8_t>((val >> 8) & 0xFF));
    checksum ^= static_cast<uint8_t>(val & 0xFF);
    checksum ^= static_cast<uint8_t>((val >> 8) & 0xFF);
  }
  frame.push_back(checksum);

  size_t bytes_written = 0;
  while (bytes_written < frame.size())
  {
    ssize_t ret = ::write(fd_, frame.data() + bytes_written, frame.size() - bytes_written);
    if (ret < 0)
    {
      if (errno == EINTR || errno == EAGAIN) continue;
      return false;
    }
    bytes_written += static_cast<size_t>(ret);
  }

  return true;
}

bool USBSerialHardware::read_feedback()
{
  if (fd_ < 0) return false;

  char buf[256];
  ssize_t n = ::read(fd_, buf, sizeof(buf) - 1);
  if (n > 0)
  {
    buf[n] = '\0';
    // 可以在这里解析 MCU 回传的当前位置
    // 格式约定: 例如 "P1:1.23 P2:-0.45 ...\n"
    RCLCPP_DEBUG(logger(), "Serial feedback: %s", buf);
    return true;
  }
  return false;
}

}  // namespace robot_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  robot_hardware_interface::USBSerialHardware,
  hardware_interface::SystemInterface)
