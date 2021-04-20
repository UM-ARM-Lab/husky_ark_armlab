.. _ark_installation:

The ARK should have already been setup by Clearpath for you. However, this information
is here in case you need it. This information can also be found in the ARK User Manual
(please contact Clearpath Support if you don't have this already), but its a PDF and
you can’t copy and paste from it.

Installing the ARK and its dependencies
=======================================

Install LCM
-----------

LCM is the middleware that communicates with the ARK.

.. code:: cpp

   cd ~
   sudo apt-get install -y unzip libglib2.0-dev build-essential
   wget https://github.com/lcm-proj/lcm/releases/download/v1.3.1/lcm-1.3.1.zip
   unzip lcm-1.3.1.zip
   cd lcm-1.3.1
   ./configure
   make
   sudo make install
   cd ~

Setup Your Workspace
--------------------

If your robot has a workspace already, you can use it and skip ahead. If
not, follow the next step to create a workspace for ARK.

.. code:: cpp

   mkdir ~/ark_ws
   cd ~/ark_ws
   mkdir src
   cd src
   catkin_init_workspace

Clone the necessary repositories
--------------------------------

.. code:: cpp

   git clone https://github.com/autonomyresearchkit/ark_bridge.git
   git clone https://github.com/autonomyresearchkit/husky_cpr_ark_navigation.git

Create the ARK bridge nodes
---------------------------

.. code:: cpp

   cd ~/ark_ws/src/ark_bridge
   ./build_client.sh

   cd ~/ark_ws
   catkin_make

Establishing connection
-----------------------

Run the following in a terminal on the robot:

.. code:: bash

   roslaunch husky_cpr_ark_navigation husky_ark_navigation.launch

You can check it is running by running the following (in a new
terminal):

.. code:: bash

   rostopic echo /ark_bridge/clock_echo

You should see a new timestamp about every second.

Configure ARK
-------------

You need to configure the ARK to run by using the config file in
``husky_cpr_ark_navigation/config``. You should already have run
``roslaunch husky_cpr_ark_navigation husky_ark_navigation.launch`` in
another terminal.

.. code:: bash

   # Navigate to the correct folder
   cd /home/administrator/ark_ws/src/husky_cpr_ark_navigation/config
   rosrun ark_bridge configure_ark.py husky_ark_configuration.yaml

You should see the following output:

.. code:: bash

   Starting...
   Configuring ARK
   Ark Configuration <Success>

If the configuration fails, try stopping the
``husky_cpr_ark_navigation`` you have running and restart it.
