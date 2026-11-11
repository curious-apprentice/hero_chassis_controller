# hero_chassis_controller

ROS Noetic `ros_control` 插件，用于完成任务中的轮速 PID 控制与麦克纳姆轮逆运动学部分。机器人模型和 Gazebo 硬件接口由 `rm_description_for_task` 提供。

## 已实现范围

- 订阅 `geometry_msgs/Twist` 话题 `/cmd_vel`，将 `linear.x`、`linear.y`、`angular.z` 视为 `base_link` 坐标系下的底盘速度。
- 根据麦克纳姆轮逆运动学计算左前、右前、左后、右后四个轮子的目标角速度。
- 每个轮子独立使用一个 `control_toolbox::Pid`，以目标角速度和测量角速度之差计算 EffortJointInterface 力矩命令。
- 从 YAML 读取轮子几何、速度/力矩限幅、命令超时和各轮 PID 参数；可通过 `rqt_reconfigure` 在线调整 PID。
- 在底盘速度指令进入逆运动学前，对平面线速度向量和角速度分别施加 YAML 可配置的加速度限制；命令超时后也会按限制平滑减速。
- 提供键盘遥控 launch，使用 `teleop_twist_keyboard` 向控制器的 `/cmd_vel` 发布速度指令。
- 通过四轮实际角速度做正运动学，积分估计平面位姿，并发布 `nav_msgs/Odometry` 到 `/odom`，同时发布 `odom → base_link` TF。
- 命令超时、无效数值或非正控制周期时停止输出非零轮速/力矩命令。

本包尚未实现全局坐标速度模式。轮式里程计是基于轮速的估计，仿真打滑、轮径误差或几何参数偏差都会影响精度。PID 初始增益是仿真起点，需按实际响应调整，不代表已经完成动态整定。

## 依赖和构建

在工作空间根目录执行：

```bash
rosdep install --from-paths src --ignore-src -r -y
catkin build hero_chassis_controller
source devel/setup.bash
```

CLion 的 CMake 配置应使用该包源码目录和工作空间对应的构建/开发目录；不要把包的独立 CMake 构建目录当成 catkin 工作空间的 `devel` 空间。

## 启动仿真

```bash
roslaunch hero_chassis_controller run_simulation_and_controller.launch
```

控制器成功加载后，从另一个已加载工作空间环境的终端发送例如：

```bash
rostopic pub -r 20 /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

可用 `rqt_reconfigure` 调节四个轮子的 `kp`、`ki`、`kd` 和 `i_clamp`。验证时观察 `/odom`、`/tf`、控制器状态、轮关节速度和 Gazebo 中底盘响应；检查 TF 树中只有一个节点发布 `odom → base_link`。具体行为还取决于仿真模型、PID 整定和执行器限制。

### 键盘遥控

先启动仿真与控制器，再在另一个已加载工作空间环境的终端运行：

```bash
roslaunch hero_chassis_controller keyboard_teleop.launch
```

键盘输入终端打开后，按 `i` 前进、`,` 后退、`j`/`l` 原地转向、`k` 停止；按 `q` 退出。键盘节点需要 `teleop_twist_keyboard`，launch 会通过 `x-terminal-emulator` 打开交互终端。

### 加速度限制

在 `config/controllers.yaml` 的控制器配置中调整：

```yaml
max_linear_acceleration: 1.0  # m/s^2，限制平面线速度变化
max_angular_acceleration: 2.0 # rad/s^2，限制角速度变化
```

线速度限制作用于 `(linear.x, linear.y)` 向量的变化量，因此横移和前进组合时也遵守同一个平面加速度上限。超时停车同样经过限速器；控制器停止/卸载时仍立即将力矩命令清零。
