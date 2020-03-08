# UWB Dataset - UAV - 1xUWB Tags at UAV

Data recorded with an active UWB tag onboard the UAV. In this recording, the UAV was flying autonomously following a circular trajectory (for details, please refer to the manuscript).

The data in this folder has been recorded with 4 UWB anchors in the room corners, at a height of approximately 1.8m, and 1 UWB tag on the

The UWB topics (4xAnchors, `/dwm1001/anchor/AN*/position` and 1xTags `/dwm1001/tag/drone/position` are recorded at approximately 10 Hz). The ground truth given by the `/vrpn_client_node/drone/pose` topic was recorded at 50Hz.

**NOTE:** The topics `/dwm1001/tag/drone/to/anchor/AN*/distance` give information about the raw distances calculated by Decawave's DRTLS.


Main details from the rosbags (for more details run `rosbag info *.bag`):

```
path:        flight01.bag
duration:    1:48s (108s)
size:        30.3 MB
topics:      /dwm1001/anchor/AN0/position                 1105 msgs    : geometry_msgs/Pose
             /dwm1001/anchor/AN1/position                 1099 msgs    : geometry_msgs/Pose              
             /dwm1001/anchor/AN2/position                 1099 msgs    : geometry_msgs/Pose              
             /dwm1001/anchor/AN3/position                 1090 msgs    : geometry_msgs/Pose              
             /dwm1001/tag/drone/position                  1107 msgs    : geometry_msgs/Pose              
             /dwm1001/tag/drone/to/anchor/AN0/distance    1103 msgs    : std_msgs/Float64                
             /dwm1001/tag/drone/to/anchor/AN1/distance    1104 msgs    : std_msgs/Float64                
             /dwm1001/tag/drone/to/anchor/AN2/distance    1099 msgs    : std_msgs/Float64                
             /dwm1001/tag/drone/to/anchor/AN3/distance    1091 msgs    : std_msgs/Float64                
             /mavlink/from                               45986 msgs    : mavros_msgs/Mavlink             
             /mavros/altitude                             1105 msgs    : mavros_msgs/Altitude            
             /mavros/battery                                55 msgs    : sensor_msgs/BatteryState        
             /mavros/estimator_status                      549 msgs    : mavros_msgs/EstimatorStatus     
             /mavros/extended_state                        217 msgs    : mavros_msgs/ExtendedState       
             /mavros/imu/data                             5498 msgs    : sensor_msgs/Imu                 
             /mavros/imu/data_raw                         5470 msgs    : sensor_msgs/Imu                 
             /mavros/imu/mag                              5438 msgs    : sensor_msgs/MagneticField       
             /mavros/imu/static_pressure                  5370 msgs    : sensor_msgs/FluidPressure       
             /mavros/imu/temperature_imu                  5338 msgs    : sensor_msgs/Temperature         
             /mavros/local_position/odom                  3260 msgs    : nav_msgs/Odometry               
             /mavros/local_position/pose                  3299 msgs    : geometry_msgs/PoseStamped       
             /mavros/local_position/velocity_body         3322 msgs    : geometry_msgs/TwistStamped      
             /mavros/local_position/velocity_local        3260 msgs    : geometry_msgs/TwistStamped      
             /mavros/manual_control/control                554 msgs    : mavros_msgs/ManualControl       
             /mavros/mission/reached                        11 msgs    : mavros_msgs/WaypointReached     
             /mavros/mission/waypoints                       1 msg     : mavros_msgs/WaypointList        
             /mavros/odometry/in                          3256 msgs    : nav_msgs/Odometry               
             /mavros/rc/in                                1093 msgs    : mavros_msgs/RCIn                
             /mavros/rc/out                               4338 msgs    : mavros_msgs/RCOut               
             /mavros/setpoint_position/local               925 msgs    : geometry_msgs/PoseStamped       
             /mavros/setpoint_raw/target_attitude          677 msgs    : mavros_msgs/AttitudeTarget      
             /mavros/setpoint_raw/target_global            428 msgs    : mavros_msgs/GlobalPositionTarget
             /mavros/state                                 111 msgs    : mavros_msgs/State               
             /mavros/statustext/recv                         6 msgs    : mavros_msgs/StatusText          
             /mavros/target_actuator_control              3302 msgs    : mavros_msgs/ActuatorControl     
             /mavros/time_reference                        108 msgs    : sensor_msgs/TimeReference       
             /mavros/timesync_status                      1103 msgs    : mavros_msgs/TimesyncStatus      
             /mavros/vfr_hud                              2181 msgs    : mavros_msgs/VFR_HUD             
             /mavros/vision_pose/pose                     1102 msgs    : geometry_msgs/PoseStamped       
             /rosout                                      6329 msgs    : rosgraph_msgs/Log                (6 connections)
             /rosout_agg                                  6303 msgs    : rosgraph_msgs/Log               
             /tf                                          6623 msgs    : tf2_msgs/TFMessage               (2 connections)
             /tf_static                                      1 msg     : tf2_msgs/TFMessage              
             /tfmini_ros_node/TFmini                     10912 msgs    : sensor_msgs/Range               
             /vrpn_client_node/drone/pose                 5374 msgs    : geometry_msgs/PoseStamped

```
