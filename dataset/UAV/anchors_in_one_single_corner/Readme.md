# UWB Dataset - UAV - 1xUWB Tags at UAV

Data recorded with an active UWB tag onboard the UAV.

The data in this folder has been recorded with 4 UWB anchors in one of the corners of the optitrack motion capture arena, separated by approximately 0.5m, three of them at a height of 0.1m and a fourth one at a height of 0.6m. More information about the anchor positions and motivation behind this recording is available on the manuscript.

The UWB topics (4xAnchors, `/dwm1001/anchor/AN*/position` and 1xTags `/dwm1001/tag/drone/position` are recorded at approximately 10 Hz). The ground truth given by the `/vrpn_client_node/drone/pose` topic was recorded at 50Hz.

**NOTE:** The topics `/dwm1001/tag/drone/to/anchor/AN*/distance` give information about the raw distances calculated by Decawave's DRTLS.


Main details from the rosbags (for more details run `rosbag info *.bag`):

```
path:        flight01.bag
duration:    2:56s (176s)
size:        12.6 MB
topics:      /dwm1001/anchor/AN0/position                1761 msgs    : geometry_msgs/Pose
             /dwm1001/anchor/AN1/position                1760 msgs    : geometry_msgs/Pose       
             /dwm1001/anchor/AN2/position                1749 msgs    : geometry_msgs/Pose       
             /dwm1001/anchor/AN3/position                1709 msgs    : geometry_msgs/Pose       
             /dwm1001/tag/drone/position                 1741 msgs    : geometry_msgs/Pose       
             /dwm1001/tag/drone/to/anchor/AN0/distance   1761 msgs    : std_msgs/Float64         
             /dwm1001/tag/drone/to/anchor/AN1/distance   1761 msgs    : std_msgs/Float64         
             /dwm1001/tag/drone/to/anchor/AN2/distance   1750 msgs    : std_msgs/Float64         
             /dwm1001/tag/drone/to/anchor/AN3/distance   1711 msgs    : std_msgs/Float64         
             /vrpn_client_node/drone/pose                7700 msgs    : geometry_msgs/PoseStamped
```
