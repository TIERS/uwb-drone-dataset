# ROS Interface with DWM1001 Tags

The dwm1001_interface ROS node

The code has been tested under Ubuntu 16.04 and ROS Kinetic only.

## Scripts

This repository contains ROS nodes written in Python to interface with the two types of UWB Tags running Decawave's default RTLS firmware. The nodes communicate with the tags throught Decawave's UART API.

The node `dwm1001_active.py` obtains position information of all the anchors in the DRTLS system within range, the distances to them and the tag position. The node `dwm1001_passive.py` gives the positions of all other active tags in the DRTLS system.

The active tags publishes anchor positions to `/dwm1001/anchor/AN*/position`, and its own position and distances to anchors to `/dwm1001/tag/tag_name/position` and `/dwm1001/tag/tag_name/to/anchor/AN*/distance`, respectively. The __tag_name_ is given as a parameter.

## Usage

Copy the __dwm1001_interface__ folder under `your_catkin_ws/src/`. Then simply run `rosrun dwm1001_interface dwm1001_active.py _port:=/dev/ttyACM*/ _tag_name:="your_tag_name"` or `rosrun dwm1001_interface dwm1001_passive.py _port:=/dev/ttyACM*/` with the corresponding port where the UWB tag is connected.
