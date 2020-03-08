# UWB Dataset - GS - 2xUWB Tags at UAV

Data recorded with a passive tag at the ground station, with a delay of approximately 400ms between the optitrack data and the UWB location data.

The data in this folder has been recorded with 4 UWB anchors in the room corners, at a height of approximately 1.8m, and 2 UWB tags on the UAV.

The first 5 flights do not have significant changes in orientation. The last 4 flights change the orientation at different heights, positions and speeds.

The UWB topics (4xAnchors, `/dwm1001/anchor/AN*/position` and 2xTags `/dwm1001/tag/****/position` are recorded at approximately 10 Hz). The ground truth given by the `/vrpn_client_node/drone/pose` topic was recorded at 50Hz.

Main details from the rosbags (for more details run `rosbag info *.bag`):

```
path:        flight01.bag
duration:    3:34s (214s)
size:        2.7 MB
topics:      /dwm1001/anchor/AN0/position    2140 msgs    : geometry_msgs/Pose
             /dwm1001/anchor/AN1/position    2140 msgs    : geometry_msgs/Pose
             /dwm1001/anchor/AN2/position    2140 msgs    : geometry_msgs/Pose
             /dwm1001/anchor/AN3/position    2099 msgs    : geometry_msgs/Pose
             /dwm1001/tag/581E/position      2140 msgs    : geometry_msgs/Pose
             /dwm1001/tag/D3B8/position      2140 msgs    : geometry_msgs/Pose
             /vrpn_client_node/drone/pose   10143 msgs    : geometry_msgs/PoseStamped
---
path:        flight02.bag
duration:    1:00s (60s)
size:        822.6 KB
topics:      /dwm1001/anchor/AN0/position    609 msgs    : geometry_msgs/Pose
             /dwm1001/anchor/AN1/position    608 msgs    : geometry_msgs/Pose
             /dwm1001/anchor/AN2/position    607 msgs    : geometry_msgs/Pose
             /dwm1001/anchor/AN3/position    596 msgs    : geometry_msgs/Pose
             /dwm1001/tag/581E/position      608 msgs    : geometry_msgs/Pose
             /dwm1001/tag/D3B8/position      608 msgs    : geometry_msgs/Pose
             /vrpn_client_node/drone/pose   3041 msgs    : geometry_msgs/PoseStamped
---
path:        flight03.bag
duration:    40.0s
size:        541.3 KB
topics:      /dwm1001/anchor/AN0/position    400 msgs    : geometry_msgs/Pose
             /dwm1001/anchor/AN1/position    399 msgs    : geometry_msgs/Pose
             /dwm1001/anchor/AN2/position    399 msgs    : geometry_msgs/Pose
             /dwm1001/anchor/AN3/position    396 msgs    : geometry_msgs/Pose
             /dwm1001/tag/581E/position      400 msgs    : geometry_msgs/Pose
             /dwm1001/tag/D3B8/position      400 msgs    : geometry_msgs/Pose
             /vrpn_client_node/drone/pose   1956 msgs    : geometry_msgs/PoseStamped
---
path:        flight04.bag
duration:    51.5s
size:        697.1 KB
topics:      /dwm1001/anchor/AN0/position    515 msgs    : geometry_msgs/Pose
             /dwm1001/anchor/AN1/position    515 msgs    : geometry_msgs/Pose
             /dwm1001/anchor/AN2/position    515 msgs    : geometry_msgs/Pose
             /dwm1001/anchor/AN3/position    501 msgs    : geometry_msgs/Pose
             /dwm1001/tag/581E/position      515 msgs    : geometry_msgs/Pose
             /dwm1001/tag/D3B8/position      515 msgs    : geometry_msgs/Pose
             /vrpn_client_node/drone/pose   2562 msgs    : geometry_msgs/PoseStamped
---
path:        flight05.bag
duration:    59.6s
size:        804.1 KB
topics:      /dwm1001/anchor/AN0/position    597 msgs    : geometry_msgs/Pose
             /dwm1001/anchor/AN1/position    596 msgs    : geometry_msgs/Pose
             /dwm1001/anchor/AN2/position    596 msgs    : geometry_msgs/Pose
             /dwm1001/anchor/AN3/position    592 msgs    : geometry_msgs/Pose
             /dwm1001/tag/581E/position      596 msgs    : geometry_msgs/Pose
             /dwm1001/tag/D3B8/position      596 msgs    : geometry_msgs/Pose
             /vrpn_client_node/drone/pose   2954 msgs    : geometry_msgs/PoseStamped
---
path:        flight06.bag
duration:    1:05s (65s)
size:        882.8 KB
topics:      /dwm1001/anchor/AN0/position    656 msgs    : geometry_msgs/Pose
             /dwm1001/anchor/AN1/position    656 msgs    : geometry_msgs/Pose
             /dwm1001/anchor/AN2/position    656 msgs    : geometry_msgs/Pose
             /dwm1001/anchor/AN3/position    644 msgs    : geometry_msgs/Pose
             /dwm1001/tag/581E/position      656 msgs    : geometry_msgs/Pose
             /dwm1001/tag/D3B8/position      656 msgs    : geometry_msgs/Pose
             /vrpn_client_node/drone/pose   3250 msgs    : geometry_msgs/PoseStamped
---
path:        flight07.bag
duration:    1:27s (87s)
size:        1.1 MB
topics:      /dwm1001/anchor/AN0/position    876 msgs    : geometry_msgs/Pose
             /dwm1001/anchor/AN1/position    875 msgs    : geometry_msgs/Pose
             /dwm1001/anchor/AN2/position    875 msgs    : geometry_msgs/Pose
             /dwm1001/anchor/AN3/position    857 msgs    : geometry_msgs/Pose
             /dwm1001/tag/581E/position      876 msgs    : geometry_msgs/Pose
             /dwm1001/tag/D3B8/position      876 msgs    : geometry_msgs/Pose
             /vrpn_client_node/drone/pose   4317 msgs    : geometry_msgs/PoseStamped
---
path:        flight08.bag
version:     2.0
duration:    1:50s (110s)
size:        1.4 MB
topics:      /dwm1001/anchor/AN0/position   1103 msgs    : geometry_msgs/Pose
             /dwm1001/anchor/AN1/position   1102 msgs    : geometry_msgs/Pose
             /dwm1001/anchor/AN2/position   1102 msgs    : geometry_msgs/Pose
             /dwm1001/anchor/AN3/position   1082 msgs    : geometry_msgs/Pose
             /dwm1001/tag/581E/position     1102 msgs    : geometry_msgs/Pose
             /dwm1001/tag/D3B8/position     1102 msgs    : geometry_msgs/Pose
             /vrpn_client_node/drone/pose   5441 msgs    : geometry_msgs/PoseStamped
---
path:        flight09.bag
duration:    38.2s
size:        517.4 KB
topics:      /dwm1001/anchor/AN0/position    383 msgs    : geometry_msgs/Pose
             /dwm1001/anchor/AN1/position    383 msgs    : geometry_msgs/Pose
             /dwm1001/anchor/AN2/position    382 msgs    : geometry_msgs/Pose
             /dwm1001/anchor/AN3/position    370 msgs    : geometry_msgs/Pose
             /dwm1001/tag/581E/position      382 msgs    : geometry_msgs/Pose
             /dwm1001/tag/D3B8/position      383 msgs    : geometry_msgs/Pose
             /vrpn_client_node/drone/pose   1869 msgs    : geometry_msgs/PoseStamped
```
