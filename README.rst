================
husky_ark_armlab
================

This package is meant to be used with the Clearpath Autonomous Research Kit (ARK) and provides
additional autonomy functionality not provided by the ARK. Although the ARK GUI has map, waypoint,
and route creation, it only supports this for a single map. The GUI does not support switching maps,
and all route data is erased for a previous map when a new map is created. To use routes that used
multiple maps, I created this ROS package to interact with the ARK's ROS API.

To see a video of this package being used to navigate autonomously between buildings,
please see below.

.. raw:: html

    <div style="position: relative; padding-bottom: 56.25%; height: 0; overflow: hidden; max-width: 100%; height: auto;">
        <iframe src="https://www.youtube.com/embed/3S68QvfWEnQ" frameborder="0" allowfullscreen style="position: absolute; top: 0; left: 0; width: 100%; height: 100%;"></iframe>
    </div>

|

Installation
------------
You should have already have setup the ARK (or more likely Clearpath has
set it up for you). If this is not the case, see the documentation on installing the ARK.

To install the husky_ark_armlab package, you should first source the workspace
you installed the ARK in and then you should create a new workspace. Ex::

    source ~/ark_ws/devel/setup.bash
    mkdir -p ~/catkin_ws/src
    mkdir ~/catkin_ws/
    catkin init

You can then install the husky_ark_armlab package and dependencies::

    cd ~/catkin_ws/src/
    git clone https://github.com/EricWiener/husky-ark husky_ark_armlab
    git clone --single-branch --branch feat/range-filter-dynamic-reconfigure https://github.com/EricWiener/laser_filters.git
    cd ~/catkin_ws
    rosdep install --from-paths . --ignore-src
    catkin build

You should now source your workspace and you should be ready to run the package::

    source ~/catkin_ws/devel/setup.bash

Routes
------
Routes are defined through a JSON file that defines a list of maps and which map to start on.
Each map contains a list of waypoints and which map to switch to upon completion. An example
of a JSON route is shown below::

    {
        "start_map": "top-floor-robotics",
        "maps": {
            "top-floor-robotics": {
                "description": "...",
                "next_map": "bottom-floor",
                "route": [
                    {
                        "name": "origin",
                        "description": "...",
                        "x": -0.121,
                        "y": -0.001,
                        "euler_angle": 177
                    },
                    {
                        "name": "outside-lab-balcony",
                        "description": "...",
                        "x": 2.029,
                        "y": -7.975,
                        "euler_angle": 355
                    },
                ]
            }
        }
    }

Waypoints are specified using X, Y, and rotation with respect to the z-axis. It is not necessary to specify the Z position
because the ARK only supports 2D mapping.

You can use the ARK GUI to find the XY, theta positions of the different waypoints.
After creating the waypoints, you can go into "Drive" mode and select waypoints to
navigate to. When you select a waypoint, its position and orientation will be displayed.

Route Tools
-----------
To speed up route creation, the package contains scripts to reverse an existing route,
validate a route, and simulate a route by printing out the order that waypoints will
be visited.

Reversing a route is useful because the route only needed to be specified in
one direction, and the return route would be automatically generated (including rotating
poses by 180 degrees, reversing waypoint order, and reversing the order of maps). You can do this with::

    python src/husky_ark_armlab reverse_route.py -r ./config/initial-route.json -o ./config/reversed-route.json

Validating a route is useful to ensure that any changes to the route were valid and catch issues with
the route before navigation began. You can do this with::

    python src/husky_ark_armlab route_loader.py -r ./config/myroute.json -v

Simulating a route was useful for debugging without requiring the robot to be running or using
a ROS environment. This can be done with::

    python src/husky_ark_armlab simulate_route -r ./config/myroute.json

Keyboard Commands
-----------------
While running the husky_ark_armlab ROS node, you can use multiple keyboard commands to interact
with the node. All commands should be typed in the same terminal that the node is running in.

- Pause/resume autonomoy: press <space>
- Skip a waypoint: press "n"
- Increase the max lidar threshold: press "w"
- Shrink the max lidar threshold: press "s"
- Set the max lidar threshold to a specific value: press any digit between 0-9

Multi-Map Navigation
--------------------
To support reduce the accumulated drift over time and make maps easier to update, you should
break your route into multiple smaller maps. Each map's final waypoint
should correspond to the first waypoint in the following map. This allows chaining together
multiple maps.

To make sure the ARK accurately localizes when starting a new map, the husky_ark_armlab node will
provide the ARK with a starting pose, which is the first waypoint defined in a map.

Multiple maps can be used to support navigation between floors of a building. The ARK
(and most navigation packages) only supports 2D navigation. This means a map can not have multiple
floors. To solve this, you should include an elevator in the map for a floor.

The Husky should either began a map or end a map inside an elevator (when using an elevator).
This allowed us to switch maps inside an elevator when the doors were closed. The Husky can then
re-localize with respect to the new map once the next floor was reached and the elevator doors opened.
When creating maps, you should erase elevator doors to allow the Husky to treat them as only a
temporary obstacle. The Husky will wait outside the elevator until the doors opened, and then it
will proceed inside.

When choosing the location to switch maps (for start/end positions that weren't inside elevators),
you should choose a location where the Husky can localize well and can easily tell which direction
it is facing (ex. switching maps between a set of double doors is not recommended because the ARK
will not be able to tell which direction it is facing).

Adjustable Lidar Thresholds
---------------------------
A major issue I faced during autonomous navigation was dealing with the sloped ground.
The ARK would recognize sloped ground as an obstacle (as shown below). This made it very difficult
to path plan up or down a sloped path.

.. figure:: https://github.com/EricWiener/husky-ark/blob/master/docs/_static/lidar-sloped-ground-figure.png?raw=true
      :alt: lidar-sloped-ground-figure
      :class: with-shadow
      :width: 400px

      When trying to navigate up or down a sloped path, the ground ahead appears as an obstacle. The Husky is represented as a grey box, the hill as the curved black line, and the lidar scan as the red arrow.

To solve this, you can manually (see keyboard commands) adjust the max lidar thresholds during navigation or set per-waypoint
and per-map thresholds through the route JSON configuration. Reducing the radius of the laser scans avoids detecting sloped
ground as obstacles.

Because the laser_filters package doesn't support dynamically changing thresholds for a range filter, you
should use the `fork I have created <https://github.com/EricWiener/laser_filters>`_.

Config files
------------

-  **default.yaml** This specifies the default route to use
-  **laser_filter_config.yaml** This configures the laser filter
-  **robotics-to-wilson.json** Example route JSON
-  **wilson-to-robotics.json** Example of automatically reversed route

Launch files
------------

-  **husky_ark_armlab.launch:** this will launch autonomous navigation
   using a specified route

   Required Arguments

   -  **route** The absolute path to the route JSON file to use

   Optional Arguments

   -  **map** The map to start navigation from. This is useful for debugging.
   -  **waypoint** The waypoint number to start navigating from
      (zero-indexed)

Usage
-----

Run the main node with

::

    roslaunch husky_ark_armlab husky_ark_armlab.launch route:=<absolute path to route>

For example::

    roslaunch husky_ark_armlab husky_ark_armlab.launch route:=/home/eric/catkin_ws/src/husky-ark/config/robotics-to-wilson.json
