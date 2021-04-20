.. _ark_quick_reference:

ARK Quick Reference
===================

This page is meant to serve as a useful reference for working with the ARK.
This is meant to be supplementary information only. Please consult the ARK guide
and API manual for more information.

Startup
-------

Run the following in seperate terminals

::

   roslaunch husky_cpr_ark_navigation husky_ark_navigation.launch

::

   rosservice call /ark_bridge/start_autonomy "req_data: {}"
   google-chrome http://192.168.132.111:5000

Services
--------

List maps
~~~~~~~~~

::

   rosservice call /ark_bridge/map_data_list_maps "req_data: {}"

Load map
~~~~~~~~

::

   rosservice call /ark_bridge/map_data_load_map_from_disk "req_data:
     data: 'top-floor-robotics'"

Save map
~~~~~~~~

Please be careful when saving a map. You can overwrite an existing map
if you save a map with the same name (without warning).

If you edit a map in the web interface, you need to make sure that you
save it using the ROS command line interface so it doesn’t get deleted.
If you just save the map using the “Save” button in the web interface it
will be erased if you click “New Map”. **The only way to avoid
overwriting the previous map is to use the ROS interface.**

The map topic should always be ``/slam/map``. You can name the file
whatever you want. For instance, naming the file ``top-floor-robotics``
will save a map to ``/var/tmp/ros/top-floor-robotics.map``

Please note you can't interact with the map files directly (they are stored
on the ARK computer and you can't access them).

::

   rosservice call /ark_bridge/map_data_save_map_to_disk "req_data:
     map_topic: '/slam/map'
     timeout:
       secs: 0
       nsecs: 0
     filename: 'top-floor-robotics'"

Delete maps
~~~~~~~~~~~

Deleting a map is permanent and irreversible, so please make sure you
are deleting the correct map and that you really want to delete it.

::

   rosservice call /ark_bridge/map_data_delete_map "req_data:
     data: ''"

Check the ARK is running
~~~~~~~~~~~~~~~~~~~~~~~~

You can check it is running by running the following (in a new
terminal):

.. code:: bash

   rostopic echo /ark_bridge/clock_echo

You should see a new timestamp about every second.


Diagonistics
~~~~~~~~~~~~

You can look at the status of the ARK with:

.. code:: bash

   rosrun rqt_robot_monitor rqt_robot_monitor
