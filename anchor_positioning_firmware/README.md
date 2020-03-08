# DWM1001 Anchor autopositioning firmware

This folder contains firmware for estimating anchor positions in a fast and accurate manner (~1s and ~10cm error in LOS). The firmware itself only calculates the distances between pairs of anchors (returning the average and standard deviation of multiple measurements, by default 5 measurements are taken). The actual calculation of anchor positions must be done externally.

*Note that the examples below consist in very basic application using the UWB features of the DWM1001C. These examples are not intended to be used in a commercial application and may not comply with regulation requirements.*

*Advanced firmware for DWM1001C that would comply with regulations can be found on https://www.decawave.com/product/dwm1001-module/*

## Overview

This project contains simple C firmware for DWM1001 hardware. It has been tested with the DWM1001-DEV board.

The DWM1001 module is a Ultra Wideband (UWB) and Bluetooth hardware based on DecaWave's DW1000 IC and Nordic Semiconductor nrF52832 SoC. It allows to build a scalable Two-Way-Ranging (TWR) RTLS systems with up to thousands of tags. 

The DWM1001-DEV is a development board for the DWM1001 module. It contains an integrated Jlink to facilitate development and debbuging. For more information about DWM1001, please visit www.decawave.com.

The project is built as follows : 
```
anchor_positioning_firmware/
├── boards            // DWM1001-DEV board specific definitions
├── deca_driver       // DW1000 API software package 2.04 
├── src               // C firmware for autopositioning 4 anchors (extendable for a larger number)
│   ├── ss_twr_init   // Single Sided Two Way Ranging Initiator example
│   └── twi_accel     // LIS2DH12 accelerometer example with Two Wire interface 
├── nRF5_SDK_14.2.0   // Nordic Semiconductor SDK 14.2 for nrF52832
└── README.md
```

For more information about nrF52832 and nrF SDK, please visit http://infocenter.nordicsemi.com/

### Single Sided Two Way Ranging Anchors

This folder contains the source code for the anchor autopositioning described in the manuscript. Each anchor takes the role of initiator once (the first one does so whenever a `S` command is received through the UART interface), and calculated its distance with all other anchors one at a time before the next one becomes initiator. The initiator will send a frame, wait for the response from the receiver, and calculate the distance. This is done multiple times, and the average distance and standard deviation are stored. When an anchor is in responder mode, it will be waiting to receive a frame from an initiator and send the corresponding answer. Each distance is effectively calculated twice.

```
anchor_positioning_firmware/ss_twr_init/
├── config                    // Contains sdk_config.h file for nrF SDK 14.2 customization
├── main.c                    // Initialization and main program
├── ss_init_main.c            // Initiator and responder program (each anchor takes the initiator role once)
├── UART                      // Uart 
├── SES
│   └── ss_twr_init.emProject // Segger Embedded Studio project
```
The application function is detailed in the main.c and the ss_init_main.c files. 

Calibration is necessary in order to have an accurate measurement. It can be done by adjusting the antenna delay which is hardware dependent. 


## Supported IDE

The examples are ready to use with the following IDE :
* Segger Embedded Studio (SES)
* Keil KEIL µVision

## Segger Embedded Studio

Each code contains a emproject project file for SES. The examples compile and load cleanly to the DWM1001.
The project was created with the SES version V3.34a. 

SES has a free license for nrF52832 development. Consequently, this IDE can be used without any limitation for DWM1001 development.

For more information regarding Segger Embedded Studio, please visit https://www.segger.com/products/development-tools/embedded-studio/

For more information about free license for nrF52832, please read https://www.nordicsemi.com/News/News-releases/Product-Related-News/Nordic-Semiconductor-adds-Embedded-Studio-IDE-support-for-nRF51-and-nRF52-SoC-development

### SES : Additional Package

When using SES IDE, you will need to install the following package :

Package
CMSIS 5 CMSIS-CORE Support Package (version 5.02)
CMSIS-CORE Support Package (version 4.05)
Nordic Semiconductor nRF CPU Support Package (version 1.06)

They can be install from SES itself, through the package manager in the tools menu. 







