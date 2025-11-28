.. EM1500 ADS Bridge documentation master file, created by
   sphinx-quickstart on Mon Nov 24 14:03:35 2025.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

EM1500 ADS Bridge documentation
===============================

This is the documentation for the **EM1500 ADS-ROS2_Isaac Sim Bridge**. This project connects a physical stewart platform (EM1500) to a digital twin in NVIDIA Isaac Sim using:

* **TwinCAT 3 (Beckhoff PLC)**
* **ADS communication in C++**
* **ROS2 (Humble)**
* **Isaac Sim Action Graph**
* **Isaac Sim ROS2 Bridge**
* **Inverse Kinematics in C++ using Eigen**

This documentation describes what we built, how it works, and how future students can reproduce and extend the system.

The project was developed by:

* **Elias Landro**
* **Marcus Axelsen Wold**
* **Ole-Morten Nyheim**

.. contents::
   :depth: 2
   :local:
   
Introduction
------------
   
The goal of this project is to stream the real-time motion of the EM1500 Stewart platform from a Beckhoff PLC into a virtual model inside NVIDIA Isaac Sim.

The system consists of four major components:

1. **PLC Pose Output (TwinCAT 3)**  
   A structured variable ``stEM1500Pose`` stores surge, sway, heave, roll, pitch, yaw as `LREAL`, updated every PLC cycle.

2. **ROS2 ADS Bridge (C++ node)**  
   A custom C++ node reads the structured pose through ADS, performs Stewart-platform inverse kinematics, and publishes the 6 computed leg lengths on:  
   ``/stewart/legs_em1500``

3. **Isaac Sim ROS2 Subscriber + Action Graph**  
   Isaac Sim reads the ROS2 topic and applies the leg lengths to six prismatic joint drives.

4. **Digital Twin**  
   A realistic Stewart platform with proper joints, limits, orientation, and physics constraints.
   
This documentation will cover:

* PLC ADS pose interface  
* ROS2 C++ node architecture  
* Setup of Eigen, ADS library, CMake, package.xml  
* Stewart platform inverse kinematics  
* Isaac Sim configuration (Action Graph and joint mapping)  
* Calibration and sign conventions  
* Publishing rates and sync  
* Known issues and troubleshooting  
* Demonstration videos/GIFs

.. toctree::
   :maxdepth: 2
   :caption: Contents:

   plc_interface
   ros2_ads_bridge
   isaac_integration
   running_the_system
   demos
