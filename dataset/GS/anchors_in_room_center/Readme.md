# UWB Dataset - GS - 1xUWB Tags at UAV

Data recorded with a passive tag at the ground station (GS), with a delay of approximately 400ms between the optitrack data and the UWB location data.

The data in this folder has been recorded with 4 UWB anchors in the center of the optitrack motion capture arena, at a height of approximately 0.1m, and 4 UWB tags on the UAV.

The UWB topics (4xAnchors, `/dwm1001/anchor/AN*/position` and 4xTags `/dwm1001/tag/****/position` are recorded at approximately 10 Hz). The ground truth given by the `/vrpn_client_node/drone/pose` topic was recorded at 50Hz.

**NOTE:** In this setup, the GS is connected to both an active tag (to obtain anchor positions) and a passive tag (to obtain the positions of all active tags, including the four in the UAV and the one in the GS). The topics `/dwm1001/tag/581E/position` and `/dwm1001/tag/fixed_tag/position` give the same information: the position of the active tag in the GS. The former one is recorded through the passive tag, while the latter one is given directly by the activa tag itself.

The topics recorded from the active tag give an idea of the stability of the UWB measurements.

Main details from the rosbags (for more details run `rosbag info *.bag`):

```
path:        flight01.bag
duration:    1:59s (119s)
size:        13.2 MB
topics:      /dwm1001/anchor/AN0/position                     1194 msgs    : geometry_msgs/Pose
             /dwm1001/anchor/AN1/position                     1193 msgs    : geometry_msgs/Pose       
             /dwm1001/anchor/AN2/position                     1187 msgs    : geometry_msgs/Pose       
             /dwm1001/anchor/AN3/position                     1098 msgs    : geometry_msgs/Pose       
             /dwm1001/tag/581E/position                       1193 msgs    : geometry_msgs/Pose       
             /dwm1001/tag/8A09/position                       1194 msgs    : geometry_msgs/Pose       
             /dwm1001/tag/91B8/position                       1193 msgs    : geometry_msgs/Pose       
             /dwm1001/tag/9320/position                       1193 msgs    : geometry_msgs/Pose       
             /dwm1001/tag/D3B8/position                       1193 msgs    : geometry_msgs/Pose       
             /dwm1001/tag/fixed_tag/position                  1113 msgs    : geometry_msgs/Pose       
             /dwm1001/tag/fixed_tag/to/anchor/AN0/distance    1193 msgs    : std_msgs/Float64         
             /dwm1001/tag/fixed_tag/to/anchor/AN1/distance    1193 msgs    : std_msgs/Float64         
             /dwm1001/tag/fixed_tag/to/anchor/AN2/distance    1187 msgs    : std_msgs/Float64         
             /dwm1001/tag/fixed_tag/to/anchor/AN3/distance    1098 msgs    : std_msgs/Float64         
             /vrpn_client_node/drone/pose                     5260 msgs    : geometry_msgs/PoseStamped
---
path:        flight02.bag
duration:    2:36s (156s)
size:        16.9 MB
topics:      /dwm1001/anchor/AN0/position                     1566 msgs    : geometry_msgs/Pose
             /dwm1001/anchor/AN1/position                     1564 msgs    : geometry_msgs/Pose       
             /dwm1001/anchor/AN2/position                     1560 msgs    : geometry_msgs/Pose       
             /dwm1001/anchor/AN3/position                     1523 msgs    : geometry_msgs/Pose       
             /dwm1001/tag/581E/position                       1446 msgs    : geometry_msgs/Pose       
             /dwm1001/tag/8A09/position                       1568 msgs    : geometry_msgs/Pose       
             /dwm1001/tag/91B8/position                       1447 msgs    : geometry_msgs/Pose       
             /dwm1001/tag/9320/position                       1447 msgs    : geometry_msgs/Pose       
             /dwm1001/tag/D3B8/position                       1447 msgs    : geometry_msgs/Pose       
             /dwm1001/tag/fixed_tag/position                  1547 msgs    : geometry_msgs/Pose       
             /dwm1001/tag/fixed_tag/to/anchor/AN0/distance    1556 msgs    : std_msgs/Float64         
             /dwm1001/tag/fixed_tag/to/anchor/AN1/distance    1564 msgs    : std_msgs/Float64         
             /dwm1001/tag/fixed_tag/to/anchor/AN2/distance    1560 msgs    : std_msgs/Float64         
             /dwm1001/tag/fixed_tag/to/anchor/AN3/distance    1523 msgs    : std_msgs/Float64         
             /vrpn_client_node/drone/pose                     6019 msgs    : geometry_msgs/PoseStamped

```
