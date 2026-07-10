# robot_arm_ws

基于 ROS2 Humble + MoveIt2 的通用机械臂仿真与运动规划工作空间。

## 分支架构

```
robot_arm_ws/
+-- src/
|   +-- robot_description/
|   |   +-- urdf/robot.urdf.xacro
|   |   +-- meshes/visual/
|   |   +-- meshes/collision/
|   |   +-- rviz/
|   |
|   +-- robot_moveit_config/
|   |   +-- config/
|   |   |   +-- robot_arm.urdf.xacro
|   |   |   +-- robot_arm.srdf
|   |   |   +-- robot_arm.sim.ros2_control.xacro
|   |   |   +-- robot_arm.ros2_control.xacro
|   |   |   +-- kinematics.yaml
|   |   |   +-- moveit_controllers.yaml
|   |   |   +-- ros2_controllers.yaml
|   |   |   +-- ompl_planning.yaml
|   |   |   +-- joint_limits.yaml
|   |   |   +-- moveit.rviz
|   |   +-- launch/
|   |       +-- demo.launch.py
|   |       +-- rsp.launch.py
|   |       +-- move_group.launch.py
|   |       +-- moveit_rviz.launch.py
|   |       +-- spawn_controllers.launch.py
|   |
|   +-- robot_hardware_interface/
|   |   +-- include/
|   |   +-- src/usb_serial_hardware.cpp
|   |   +-- hardware_interface_plugins.xml
|   |
|   +-- trac_ik/
|       +-- trac_ik_lib/
|       +-- trac_ik_kinematics_plugin/
|       +-- trac_ik_examples/
|       +-- trac_ik_python/
```

## 功能

- **MoveIt2 运动规划** — IK 求解、路径规划（RRTConnect / RRTstar / LBKPIECE）
- **TRAC-IK 运动学** — 比默认 KDL 更快速、更可靠的 IK 求解
- **RViz 可视化** — 交互式标记拖拽规划、路径动画预览
- **ros2_control 仿真** — 使用 mock_components/GenericSystem 模拟硬件
- **ROS2 控制器** — joint_state_broadcaster + joint_trajectory_controller
- **硬件接口模板** — 预置 USBSerial 硬件插件，可适配不同电机通信方式
- **可替换 URDF** — 更换 robot_description/urdf/robot.urdf.xacro 即可适配其他机械臂

## 环境要求

- Ubuntu 22.04 / WSL2
- ROS2 Humble
- MoveIt2
- TRAC-IK 依赖：libnlopt-cxx-dev

## 安装

```bash
# 安装依赖
sudo apt-get update
sudo apt-get install -y libnlopt-cxx-dev

# 编译
cd ~/robot_arm_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-skip robot_hardware_interface
source install/setup.bash
```

## 启动仿真

```bash
source ~/robot_arm_ws/install/setup.bash
ros2 launch robot_moveit_config demo.launch.py
```

启动后：
1. RViz 窗口显示机械臂模型
2. 拖拽末端（粉紫色球）调整目标位姿
3. 点击 Plan 计算路径（紫色轨迹显示）
4. 点击 Execute 执行运动

## 自定义 URDF

替换 robot_description/urdf/robot.urdf.xacro 文件，并同步更新：

- robot_moveit_config/config/robot_arm.srdf — 规划组、禁用碰撞
- robot_moveit_config/config/kinematics.yaml — IK 求解器配置
- robot_moveit_config/config/joint_limits.yaml — 关节限位
- robot_moveit_config/config/robot_arm.urdf.xacro — ROS2 control 包含

## 硬件适配

robot_hardware_interface 包提供 USBSerialHardware 示例实现（继承 hardware_interface::SystemInterface），参考该实现适配你的电机通信协议：

- CANopen
- USB 串口
- EtherCAT / Modbus / 自定义协议

编译硬件接口：
```bash
colcon build --packages-select robot_hardware_interface
```

## 后续更新

- **2026.08.30** — VR 摇杆操作 + 重力补偿 + 阻抗控制
