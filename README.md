# hero_chassis_controller

A ROS 1 catkin package scaffold for a four-wheel mecanum chassis controller for the RoboMaster Hero robot.

The controller implementation has not been added yet. This repository is the starting point for the assignment; the plugin, configuration, launch files, and tests will be added as those features are implemented.

## Planned features

- A `ros_control` controller plugin loaded through `pluginlib`.
- One `control_toolbox::Pid` instance per wheel, with configurable gains.
- Mecanum inverse kinematics from `/cmd_vel` to wheel speed targets.
- Forward kinematics, `/odom` publishing, and the `odom` to `base_link` transform.
- A configurable chassis-frame or world-frame velocity command mode.
- Runtime PID tuning with `rqt_reconfigure` and signal inspection with `rqt_plot`.

## Dependencies

The package manifest declares the ROS Noetic dependencies planned for the controller. Install missing workspace dependencies with `rosdep` from the catkin workspace root.

## Build

Build this package with `catkin_tools` from the workspace root:

```bash
rosdep install --from-paths src --ignore-src -r -y
catkin build hero_chassis_controller
source devel/setup.bash
```

The assignment requires compilation and debugging through CLion. If using catkin from the command line, use `catkin build` rather than `catkin_make`.

## Run

There is no controller or launch file to run yet. Add usage instructions here when the plugin and its launch configuration are implemented.

## License

This project retains the BSD license included with the source template. See [LICENSE](LICENSE).
