# Minesweepers Gazebo

Gazebo simulation framework for the [Minesweepers](https://landminefree.org) competition. This repository contains the ROS environment and Gazebo plugins to run the simulation of the competition.


## Dependencies

* [Husky](https://github.com/husky/husky) from Clearpath.

* [Metal Detector model](https://github.com/ras-sight/metal_detector_msgs) from HRATC competition.

## Environment

Please add the `models` directory to the `GAZEBO_MODEL_PATH` environment variable. You can add the path in your `.bashrc`:

```bash
export GAZEBO_MODEL_PATH=$PATH_TO_MINESWEEPERS_GAZEBO/models:$GAZEBO_MODEL_PATH
```
