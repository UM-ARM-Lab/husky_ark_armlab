- realsense_laser_scan.launch: this can convert Depth Image to a laser scan on /scan topic


## Exploration with Husky
In seperate terminals run:
```
roslaunch husky_navigation gmapping_demo.launch
```

```
cd ~/repositories/husky-autonomy/realsense_laser_scan
roslaunch realsense_laser_scan.launch
```

```
roslaunch husky_viz view_robot.launch
```