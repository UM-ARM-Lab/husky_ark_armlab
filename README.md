# Husky ARK ARM Lab

## Extracting waypoints from ARK

You can use the ARK GUI to find the XY, theta positions of the different waypoints.
After creating the waypoints, you can go into "Drive" mode and select waypoints to
navigate to. When you select a waypoint, its position and orientation will be displayed.

## You may need to export the Python path before running

```
export PYTHONPATH=$PYTHONPATH:/usr/lib/python2.7/dist-packages
```

## Dependencies

- http://wiki.ros.org/laser_filters

## Overview

This is a template: replace, remove, and add where required. Describe here what this package does and what it's meant for in a few sentences.

**Keywords:** example, package, template

Or, add some keywords to the Bitbucket or GitHub repository.


## Installation

### Installation from Packages

To install all packages from the this repository as Debian packages use

    sudo apt-get install ros-noetic-...

Or better, use `rosdep`:

    sudo rosdep install --from-paths src

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [Eigen] (linear algebra library)

  sudo rosdep install --from-paths src

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

    cd catkin_workspace/src
    git clone https://github.com/ethz-asl/ros_best_practices.git
    cd ../
    rosdep install --from-paths . --ignore-src
    catkin_make


## Usage

Run the main node with


    roslaunch husky_ark_armlab husky_ark_armlab.launch route:=/home/eric/catkin_ws/src/husky-ark/config/robotics-to-wilson.json


## Config files

Config file folder/set 1

- **default.yaml** This specifies the default route to use
- **laser_filter_config.yaml** This configures the laser filter
- **robotics-to-wilson.json** Example route JSON 
- **wilson-to-robotics.json** Example of automatically reversed route

## Launch files

- **husky_ark_armlab.launch:** this will launch autonomous navigation using a specified route

  Required Arguments

  - **`route`** The absolute path to the route JSON file to use

  Optional Arguments

  - **`map`** The map to start navigation from
  - **`waypoint`** The waypoint number to start navigating from (zero-indexed)

## Nodes

### husky_ark_armlab

Reads temperature measurements and computed the average.

#### Subscribed Topics

- **`/temperature`** ([sensor_msgs/Temperature])

  The temperature measurements from which the average is computed.

#### Published Topics

...

#### Services

- **`get_average`** ([std_srvs/Trigger])

  Returns information about the current average. For example, you can trigger the computation from the console with

      rosservice call /ros_package_template/get_average

#### Parameters

- **`subscriber_topic`** (string, default: "/temperature")

  The name of the input topic.

- **`cache_size`** (int, default: 200, min: 0, max: 1000)

  The size of the cache.

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/ethz-asl/ros_best_practices/issues).

[ros]: http://www.ros.org
[rviz]: http://wiki.ros.org/rviz
[eigen]: http://eigen.tuxfamily.org
[std_srvs/trigger]: http://docs.ros.org/api/std_srvs/html/srv/Trigger.html
[sensor_msgs/temperature]: http://docs.ros.org/api/sensor_msgs/html/msg/Temperature.html
