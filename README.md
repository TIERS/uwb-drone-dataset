# UWB Localization for Autonomous UAV flight

**UWB-Based System for UAV Localization in GNSS-Denied Environments: Characterization and Dataset**. Jorge Peña Queralta and Carmen Martínez Almansa and Fabrizio Schiano and Dario Floreano and Tomi Westerlund.
[The paper is available in ArXiv.](https://arxiv.org/abs/2003.04380).

If you find the code or data in this repository useful in your research, please consider citing our paper:

```
@misc{queralta2020uwbbased,
    title={UWB-based System for UAV Localization in GNSS-Denied Environments: Characterization and Dataset},
    author={Jorge Peña Queralta and Carmen Martínez Almansa and Fabrizio Schiano and Dario Floreano and Tomi Westerlund},
    year={2020},
    eprint={2003.04380},
    archivePrefix={arXiv},
    primaryClass={cs.RO}
}
```

## Introduction

This repository contains a dataset for studying the accuracy of UWB-aided autonomous flight in UAVs, as well as ROS nodes to interface with Decawave's DWM1001 nodes when they are set as __Active Tags__ or __Passive Tags__. Finally, we also provide firmware for reprogramming the DWM1001 DEV boards in order to obtain fast and accurate autopositioning of anchors.

Compared to previous datasets, this dataset contains:

- Multiple anchor configurations
- Study of UWB accuracy when the UAV is both inside and ouside the convex envelope defined by the anchor positions

## Repository structure

The repository is divided in three main parts: ROS nodes, firmware for the DWM1001 DEV, and the dataset.

### ROS NODES

Contains the **dwm1001_interface** ROS node that connected to Decawave's UART API. Two executables are available to read data form active or passive tags running Decawave's DRTLS firmware. The node `dwm1001_active.py` obtains position information of all the anchors in the DRTLS system within range, the distances to them and the tag position. The node `dwm1001_passive.py` gives the positions of all other active tags in the DRTLS system.

The active tags publishes anchor positions to `/dwm1001/anchor/AN*/position`, and its own position and distances to anchors to `/dwm1001/tag/tag_name/position` and `/dwm1001/tag/tag_name/to/anchor/AN*/distance`, respectively. The __tag_name_ is given as a parameter.

Please refer to the corresponding subfolder for more details.

### Anchor Positioning Firmware 

Contains firmware for estimating anchor positions in a fast and accurate manner (~1s and ~10cm error in LOS). The firmware itself only calculates the distances between pairs of anchors (returning the average and standard deviation of multiple measurements, by default 5 measurements are taken). The actual calculation of anchor positions must be done externally. 

The firmware contains the source code for the anchor autopositioning described in the manuscript. Each anchor takes the role of initiator once, and calculates its distance with all other anchors one at a time before the next one becomes initiator. The initiator will send a frame, wait for the response from the receiver, and calculate the distance. This is done multiple times, and the average distance and standard deviation are stored. When an anchor is in responder mode, it will be waiting to receive a frame from an initiator and send the corresponding answer. Each distance is effectively calculated twice.

Calibration is necessary in order to have an accurate measurement. It can be done by adjusting the antenna delay which is hardware dependent. 

Please refer to the corresponding subfolder for more details.

### Dataset

The dataset has been recorded using two different methods, with rosbags being recorded either at the UAV or at a ground station. The first case includes data which has been acquired using an onboard computer on a quadrotor equipped with an active UWB tag and flying autonomously. The second case refers to data from a passive tag connected to a ground station, while the quadrotor is being flown manually. In the second case, a delay exists between the ground truth data given by the motion capture system and the UWB data due to the passive nature of the tag being used for recording the positions.

The autonomous flight tests have been done with an F450 quadrotor equipped with a Pixhawk 2.4 running the PX4 firmware, an Intel Up Board as a companion computer running Ubuntu 16.04 with ROS Kinetic and MAVROS, and a TF Mini Lidar for height estimation.

Please refer to the corresponding subfolder for more details.

