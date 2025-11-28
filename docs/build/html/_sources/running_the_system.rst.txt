Running the System
==================

This chapter describes how to operate the physical EM1500 Stewart platform together with the ROS2 ADS bridge and the Isaac Sim digital twin.

The components do not strictly need to be started in a particular order. However, the sequence below has proven to be the most practical and straightforward during development and testing.

System Components
-----------------

The full setup consists of the following devices:

- **Stewart Platform + Power Electronics**  
  The physical EM1500 system.

- **Motion PC 2**  
  Handles low-level communication with the platform hardware.

- **CX2043**  
  Runs the MotionLab HMI, the ADS server, and EtherCAT communication.

- **C6150**  
  Runs the PLC program that exposes ``stEM1500Pose`` to ADS.

- **Ubuntu VM (ROS2)**  
  Runs the ADS bridge node that publishes target leg lengths.

- **Isaac Sim**  
  Receives ROS2 data and actuates the virtual Stewart platform.

  Recommended Startup Workflow
----------------------------

1. **Power on the Stewart platform and Motion PC 2**  
   - The EM1500 must be energized and Motion PC 2 must be running so that the
     EtherCAT chain can initialize.  
   - In the MotionLab HMI (running on the CX2043), the **EM1500 status**
     indicator should turn **green**.

2. **Verify that the CX2043 is running and the HMI is active**  
   - Open the MotionLab HMI.  
   - Check the **ADS communication indicator**:  
    - **Green** → ADS communication is OK  
    - **Gray** → ADS not ready (ROS2 will not receive pose data)

3. **Start the PLC program on the C6150**  
   - Put the program in *RUN* mode in TwinCAT.  
   - Confirm that the structure ``stEM1500Pose`` updates continuously.  
   - These pose values represent the physical Stewart platform.

4. **Start the ROS2 ADS bridge on Ubuntu**  
   Run:

   .. code-block:: bash

       ros2 run em1500_ads_bridge ads_node

   - The terminal should print surge, sway, heave, roll, pitch, and yaw.  
   - These values should match the pose shown on the C6150 PLC.

5. **Start Isaac Sim**  
   - Load the scene file ``UiAMotionLab.usd``.  
   - Press **Play** to start simulation.  
   - The Action Graph subscriber begins receiving ROS2 messages.

6. **Check real–virtual synchronization**  
   - Move the physical platform (e.g., via the HMI).  
   - The virtual Stewart platform in Isaac Sim should move identically.

Troubleshooting
---------------

If the simulation does not move:

- Make sure ROS2 receives valid leg-length data:

  .. code-block:: bash

      ros2 topic echo stewart/legs_em1500

- Verify the ADS indicator in the HMI is **green**
- Check that ``stEM1500Pose`` updates on the C6150
- Confirm the Action Graph paths for the WritePrim nodes are correct
- Ensure Isaac Sim is running (simulation must be playing)