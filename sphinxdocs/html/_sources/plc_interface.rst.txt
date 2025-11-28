PLC Interface
=============

Overview
--------

The PLC runs on a Beckhoff TwinCAT 3 system and provides the **6-DOF pose** of the EM1500 Stewart platform to the ROS2 side via ADS.

The pose is represented as:

- Surge, sway, heave (translation, in meters)
- Roll, pitch, yaw (rotation, in degrees)

This data is:

1. Read from an EtherCAT feedback structure ``rxEtherCatBridge``  
2. Mapped into a dedicated pose struct ``ST_Pose``  
3. Exposed as a global symbol (``stEM1500Pose``) that the ADS C++ client reads

Pose Data Type
--------------

The pose is defined as a separate structured type, using ``LREAL`` (64-bit floating point) as this is the datatype provided by the EtherCAT Bridge and is compatible with C++ ``double``:

.. code-block:: text

    TYPE ST_Pose :
    STRUCT
        surge : LREAL;
        sway  : LREAL;
        heave : LREAL;
        roll  : LREAL;
        pitch : LREAL;
        yaw   : LREAL;
    END_STRUCT
    END_TYPE

This type is used both for clarity in the PLC project and to mirror the structure used in the ROS2 C++ node (``struct Pose { double surge, sway, heave, roll, pitch, yaw; };``).

Main Program and EtherCAT Bridge
--------------------------------

The main PLC program is ``PROGRAM MAIN``. It declares:

- The EtherCAT bridge structures for feedback and control
- A simple time integration variable ``fTime``
- A timeout monitor for the EtherCAT link
- The exported pose ``stEM1500Pose``

Relevant excerpt:

.. code-block:: text

    PROGRAM MAIN
    VAR
        // EtherCAT bridge
        rxEtherCatBridge AT %I* : ST_Feedback;
        txEtherCatBridge AT %Q* : ST_Control;
        nRxCounter AT %I* : DINT;
        nTxCounter AT %Q* : DINT;
        nRxCounterOld : DINT;
        
        fTime    : LREAL;
        fDt      : LREAL := 0.01;
        bEnable  : BOOL := TRUE;
        timer    : TON;
        bTimout  : BOOL;
        
        stEM1500Pose : ST_Pose;
    END_VAR

System Architecture
-------------------

The variables ``rxEtherCatBridge`` and ``txEtherCatBridge`` do **not** originate inside this PLC program. Instead, they represent an EtherCAT communication channel to a **second PLC**:

- **C6150**  
  Runs the control code shown in this document.  
  Reads the pose data and exposes it over ADS for the ROS2 node to retrieve.

- **CX2043**  
  Runs the MotionLab HMI.  
  Hosts the ADS server.  
  Communicates with the physical Stewart platform through EtherCAT.

The EtherCAT bridge works as follows:

1. The CX2043 collects real Stewart platform feedback  
   (positions, actuator states, pose, etc.).
2. This feedback is sent through EtherCAT to the C6150.
3. On the C6150, the data arrives inside  
   ``rxEtherCatBridge.em1500.eta[...]``.
4. The C6150 maps these values into ``stEM1500Pose`` for the ADS client.

The CX2043 was pre-programmed by UiA Motion Lab and is not modified in any way. 

Task and Cycle Time
-------------------

The ``MAIN`` program runs in a cyclic task with:

- **Task cycle time:** 10 ms (100 Hz)
- **Time step:** ``fDt := 0.01``

The time variable is updated each cycle:

.. code-block:: text

    fTime := fTime + fDt;
    nTxCounter := nTxCounter + 1;

EtherCAT Timeout Monitoring
---------------------------

To detect communication issues on the EtherCAT bridge, a simple timeout mechanism is implemented:

.. code-block:: text

    // ****** Check for timeout ***********
    timer.PT := T#500MS;
    IF nRxCounter <> nRxCounterOld THEN
        nRxCounterOld := nRxCounter;
        timer(IN := FALSE);
    ELSE
        timer(IN := TRUE);
    END_IF
    bTimout := timer.Q;

Logic:

- ``nRxCounter`` is updated by the EtherCAT bridge on new data
- If ``nRxCounter`` changes, the timer is reset
- If it does not change for 500 ms, ``bTimeout`` becomes TRUE

This can be used later to stop motion or flag an error.

Mapping EtherCAT Feedback to Pose
---------------------------------

The actual Stewart platform pose is provided through the EtherCAT feedback bridge:

- ``rxEtherCatBridge.em1500.eta[0..5]``

These values are mapped directly into the ``stEM1500Pose`` struct:

.. code-block:: text

    stEm1500Pose.surge := rxEtherCatBridge.em1500.eta[0];
    stEm1500Pose.sway  := rxEtherCatBridge.em1500.eta[1];
    stEm1500Pose.heave := rxEtherCatBridge.em1500.eta[2];
    stEm1500Pose.roll  := rxEtherCatBridge.em1500.eta[3];
    stEm1500Pose.pitch := rxEtherCatBridge.em1500.eta[4];
    stEm1500Pose.yaw   := rxEtherCatBridge.em1500.eta[5];

This step:

1. Decouples the EtherCAT-specific data structure from the public pose interface
2. Provides a clean, named interface for the ADS/ROS2 side
3. Ensures consistent ordering of the DOFs between PLC and ROS

Example: Legacy Test Code
-------------------------

The following block in the PLC code shows how a position command could be written directly to the Stewart platform:

.. code-block:: text

    txEtherCatBridge.em1500.u[2] := 0.2 * SIN(2.0 * 3.14 * 0.1 * fTime);

Here, ``u[2]`` corresponds to the **heave** of the EM1500 Stewart platform. This test code was used only during early validation and is not part of the final ADS–ROS2–Isaac Sim bridge. It remains as a minimal example of how actuator values can be written to the EtherCAT output structure.