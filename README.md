# hero_chassis_controller

ROS Noetic `ros_control` 插件，用于完成任务中的轮速 PID 控制与麦克纳姆轮逆运动学部分。机器人模型和 Gazebo 硬件接口由 `rm_description_for_task` 提供。

## 已实现范围

- 订阅 `geometry_msgs/Twist` 话题 `/cmd_vel`，将 `linear.x`、`linear.y`、`angular.z` 视为 `base_link` 坐标系下的底盘速度。
- 根据麦克纳姆轮逆运动学计算左前、右前、左后、右后四个轮子的目标角速度。
- 每个轮子独立使用一个 `control_toolbox::Pid`，以目标角速度和测量角速度之差计算 EffortJointInterface 力矩命令。
- 从 YAML 读取轮子几何、速度/力矩限幅、命令超时和各轮 PID 参数；可通过 `rqt_reconfigure` 在线调整 PID。
- 命令超时、无效数值或非正控制周期时停止输出非零轮速/力矩命令。

本包不实现正运动学、里程计、TF、全局坐标速度模式、加速度限制或键盘遥控；这些属于后续任务内容。PID 初始增益是仿真起点，需按实际响应调整，不代表已经完成动态整定。

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

可用 `rqt_reconfigure` 调节四个轮子的 `kp`、`ki`、`kd` 和 `i_clamp`。验证时同时观察控制器状态、轮关节速度和 Gazebo 中底盘响应；具体行为还取决于仿真模型、PID 整定和执行器限制。
