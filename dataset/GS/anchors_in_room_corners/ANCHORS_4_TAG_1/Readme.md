# UWB Dataset - GS - 1xUWB Tags at UAV

Data recorded with a passive tag at the ground station, with a delay of approximately 400ms between the optitrack data and the UWB location data.

The data in this folder has been recorded with 4 UWB anchors in the room corners, at a height of approximately 1.8m, and 1 UWB tag on the UAV.

The first flight was recorded at a lower speed, the second flight was recorded with the UAV flying faster in general and at different heights.

The UWB topics (4xAnchors, `/dwm1001/anchor/AN*/position` and 2xTags `/dwm1001/tag/****/position` are recorded at approximately 10 Hz). The ground truth given by the `/vrpn_client_node/drone/pose` topic was recorded at 50Hz.

Main details from the rosbags (for more details run `rosbag info *.bag`):

```
path:        flight01.bag
duration:    3:01s (181s)
size:        2.2 MB
topics:      /dwm1001/anchor/AN0/position   1820 msgs    : geometry_msgs/Pose
             /dwm1001/anchor/AN1/position   1820 msgs    : geometry_msgs/Pose       
             /dwm1001/anchor/AN2/position   1819 msgs    : geometry_msgs/Pose       
             /dwm1001/anchor/AN3/position   1800 msgs    : geometry_msgs/Pose       
             /dwm1001/tag/D3B8/position     1820 msgs    : geometry_msgs/Pose       
             /vrpn_client_node/drone/pose   9056 msgs    : geometry_msgs/PoseStamped
---
path:        flight02.bag
duration:    3:16s (196s)
size:        2.3 MB
topics:      /dwm1001/anchor/AN0/position   1965 msgs    : geometry_msgs/Pose
             /dwm1001/anchor/AN1/position   1965 msgs    : geometry_msgs/Pose       
             /dwm1001/anchor/AN2/position   1964 msgs    : geometry_msgs/Pose       
             /dwm1001/anchor/AN3/position   1944 msgs    : geometry_msgs/Pose       
             /dwm1001/tag/D3B8/position     1966 msgs    : geometry_msgs/Pose       
             /vrpn_client_node/drone/pose   9522 msgs    : geometry_msgs/PoseStamped
```
