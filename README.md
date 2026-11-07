# hero_chassis_controller

A ROS 1 catkin package scaffold for a four-wheel mecanum chassis controller for the RoboMaster Hero robot.

The controller implementation has not been added yet. The package includes the initial directory structure and launch wiring to the supplied `rm_description_for_task` Hero model. The plugin class, PID configuration, and tests are placeholders for the next implementation steps.

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

The model-only launch can be used before the controller is implemented:

```bash
roslaunch hero_chassis_controller simulation_only.launch
```

`run_simulation_and_controller.launch` also starts the controller spawner. It will not load successfully until `HeroChassisController` is implemented and exported by the plugin library.

## License

This project retains the BSD license included with the source template. See [LICENSE](LICENSE).
