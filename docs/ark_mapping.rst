.. _ark_mapping:

Map Creation
============
To create a map with the ARK, you should use the ARK GUI, which shows
the map being created in real-time. While mapping, you should drive very slowly
around the region you are mapping. Additionally, it is crucial to turn very slowly
because the Husky uses skid-steering, which involves the left and right
wheels turning at different speeds in forward and reverse. This causes
the wheels or tracks to slip, or skid, on the ground and turn about the
center of the vehicle. Although skid-steering allows for sharp turns,
for robots, it “is a severe disadvantage because of the negative effect
it has on odometry: wheels that are skidding are not tracking the exact
movement of the robot” `source`_.

To mitigate the negative effects of skid-steering on odometry
(and as a consequence localization), you should turn
very slowly so the ARK is able to localize with the previously scanned
landmarks vs. needing to rely on erroneous odometry data.

After creating a map, you should manually erase any incorrectly detected
obstacles and also add exclusion zones for regions the Husky
should avoid. For instance, lidars do not detect glass, so glass
walls do not show up as obstacles. To avoid the Husky navigating into
glass walls, exclusion zones should be added for these areas.

Additionally, the ARK sometimes plans routes through un-mapped regions
because they are treated as free space. However, this often resulted in
incorrect routes, so it is necessary to surround routes with exclusion zones
to prevent the ARK from planning incorrect shortcuts. With the map created,
incorrect data erased, and exclusion zones added, you can add waypoints to the map
that the ARK should navigate to. This is done simply by clicking a
region on the map and deciding what orientation the Husky should have at
this location. You can then add the information for these waypoints to the
JSON route configuration file. This covers the basic mapping process, but doors and
outdoor navigation presented additional difficulties.

Doors
-----
The ARK does not have any built-in concept of doors and only supports
creating maps with free space or permanent obstacles. The ARK will stop
to avoid hitting any new obstacles that appear during navigation (such
as pedestrians), but it won’t update its route to avoid obstacles, not
on the map. This can be an issue for doors because they can be both
closed and open, but the ARK does not have a way to represent this. If a
door were added to the map when it was closed, the ARK would treat the
door as a permanent obstacle and never plan a route through it.

To mitigate this issue, you should allow maps to be built with the doors
closed and manually erase the doors after the mapping was completed.
This works well for single doors but is sometimes an issue when dealing with
double doors. If both doors are erased, the ARK will plan a route
through the center of both doors and will not re-plan to navigate
through a single door held open. Additionally, if too much of a single
door is erased and the nearby wall is removed, the ARK will have a
hard time localizing when the doors are closed because it expects a
free space region (this makes updating maps near existing doorways
particularly challenging).

The best approach is to erase just enough of a door for the ARK
to plan a route through. Additionally, for double doors, you should
only erase a single one of the doors of the ARK will try to navigate
into the middle of both doors.

Outdoor Navigation
------------------

Outdoor mapping is particularly difficult for the ARK. The lidars on the
Husky can detect obstacles approximately 5 meters away, but often obstacles are
not close enough to be detected when navigating outdoors. This
results in empty regions in the generated maps, as shown below.

An additional issue is that the Husky has no ability to detect where a
sidewalk ends when navigating on top of it. This means that nothing
prevents the Husky from going off curbs. Although you can add exclusion
zones to try to solve this, the lack of obstacles to localize with makes it
difficult for the ARK to avoid these areas reliably.

You can mitigate this issue by having the Husky navigate in the street (vs. on
the sidewalk), so the curb would appear as an obstacle.

.. figure:: https://github.com/UM-ARM-Lab/husky_ark_armlab/blob/master/docs/_static/outdoor-map-no-objects.png?raw=true
   :alt: outdoor-map-no-objects

   A map of an outdoor region without objects to localize with.

.. _source: https://groups.csail.mit.edu/drl/courses/cs54-2001s/skidsteer.html
