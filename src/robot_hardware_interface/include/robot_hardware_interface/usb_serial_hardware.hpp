#pragma once

#include <array>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace robot_hardware_interface
{

/**
 * @brief ros2_control 硬件插件：通过 USB 串口输出关节角度到 MCU。
 *
 * 生命周期: on_init → on_configure(开串口) → on_activate →
 *           read/write 循环 → on_deactivate → on_cleanup
 *
 * write() 将 7 个关节目标位置格式化为 ASCII 字符串通过串口发送。
 * read()  可选接收 MCU 回传的当前关节位置。
 */
class USBSerialHardware : public hardware_interface::SystemInterface
{
public:
  ~USBSerialHardware() override;

  // ─── 生命周期 ────────────────────────────────────────
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  // ─── 接口注册 ─────────────────────────────────────────
  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  // ─── 实时控制 ─────────────────────────────────────────
  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  static constexpr size_t NUM_JOINTS = 7;

  // 串口参数
  std::string serial_port_ = "/dev/ttyUSB0";
  int baud_rate_ = 115200;
  std::string protocol_ = "ascii";  // "ascii" | "binary"

  // 串口文件描述符
  int fd_ = -1;

  // 关节状态与指令缓存
  std::array<double, NUM_JOINTS> hw_positions_{};
  std::array<double, NUM_JOINTS> hw_velocities_{};
  std::array<double, NUM_JOINTS> hw_commands_{};

  // 串口操作
  bool open_serial();
  void close_serial();
  bool write_ascii(const std::array<double, NUM_JOINTS> & positions);
  bool write_binary(const std::array<double, NUM_JOINTS> & positions);
  bool read_feedback();

  // 机器人参数（从 URDF 读取）
  std::array<int, NUM_JOINTS> signs_{1, 1, 1, 1, 1, 1, 1};
};

}  // namespace robot_hardware_interface
