ROS2–ADS Bridge
===============

This chapter describes the ROS2 node that connects the Beckhoff PLC (via ADS)
to the Isaac Sim digital twin of the EM1500 Stewart platform.  
The bridge continuously reads the real platform pose, computes inverse kinematics,
and publishes six leg-length commands to Isaac Sim in real time.

Overview
--------

The ROS2–ADS bridge performs three main tasks:

1. **Read the real Stewart platform pose from TwinCAT (via ADS)**  
   The Beckhoff PLC exposes a structured variable ``ST_Pose`` containing
   surge, sway, heave, roll, pitch, and yaw of the platform.

2. **Compute the corresponding actuator lengths using inverse kinematics**  
   A C++ implementation of the EM1500 IK — matching the UiA Motion Lab
   MATLAB formulation — converts the 6-DOF pose into six leg lengths.

3. **Publish leg-length commands to Isaac Sim**  
   The lengths are published as a ``Float64MultiArray`` on  
   ``/stewart/legs_em1500``  
   where they are consumed by an Action Graph that drives six prismatic joints.


ADS Communication
-----------------

Communication with the PLC is implemented using Beckhoff’s ``AdsLib``.  
A helper struct maps the PLC variable ``MAIN.stEM1500Pose`` directly into
a C++ data structure:

.. code-block:: cpp

    struct Pose {
        double surge;
        double sway;
        double heave;
        double roll;
        double pitch;
        double yaw;
    };

An ``AdsVariable<Pose>`` instance provides live access to this structure:

.. code-block:: cpp

    struct AdsVariables {
        explicit AdsVariables(AdsDevice& route)
            : pose{route, "MAIN.stEM1500Pose"} {}
        AdsVariable<Pose> pose;
    };

Reading the pose from TwinCAT is as simple as:

.. code-block:: cpp

    Pose p = adsHandler.getPose();


Inverse Kinematics
------------------

The inverse kinematics implementation is based on the official UiA Motion Lab
EM1500 geometry. Two point sets define the actuator attachment locations:

* ``p_AibC`` — base joints  
* ``p_BitC`` — top-plate joints  

Given a platform position ``(surge, sway, heave)`` and orientation  
``(roll, pitch, yaw)``, the top-plate joint position is:

.. code-block:: cpp

    Eigen::Vector3d Pi = p_tb + R * p_BitC[i];

The leg length is then computed as:

.. code-block:: cpp

    outL[i] = (Pi - p_AibC[i]).norm();


Heave Offset Correction
-----------------------

The PLC and the IK formulation use **different vertical reference frames**.

* The PLC reports **0.0 m** at the *neutral* height.
* MATLAB’s EM1500 IK expects **≈ 1.205 m** at neutral height.

By comparing real measurements and the MATLAB model,  
the required correction was found to be:

.. code-block:: cpp

    double heave = p.heave + 1.205;

This aligns the PLC coordinate system with the IK coordinate system.

Coordinate Frame Alignment
--------------------------

To achieve correct motion in Isaac Sim, the PLC pose must be transformed into
the coordinate frame expected by the UiA EM1500 inverse kinematics. The PLC
uses a different axis convention and reports rotations in radians.

Coordinate Frame Corrections
----------------------------

The following transformations are applied to convert the PLC pose into the
IK frame:

.. code-block:: cpp
    
    surge = -p.surge;
    sway  =  p.sway;
    heave =  p.heave + 1.205;   // aligns PLC zero with IK geometric zero
    roll  = -p.roll;
    pitch =  p.pitch;
    yaw   =  p.yaw;

Dynamic Reference (L_ref)
-------------------------

Instead of using a fixed reference length, the bridge computes a dynamic
reference the first time the node runs:

.. code-block:: cpp

    if (!ref_initialized) {
        for (int i = 0; i < 6; i++)
            L_ref[i] = L[i];
        ref_initialized = true;
    }

The commanded actuator displacement is then:

.. code-block:: cpp

    qi[i] = L[i] - L_ref[i];

This ensures that the digital twin exactly matches the physical platforms
starting position, regardless of where the real EM1500 begins.


Publisher Interface
-------------------

The node publishes all six actuator commands on:

.. code-block:: text

    /stewart/legs_em1500
    (std_msgs/msg/Float64MultiArray)

The array length is always six, indexed from leg 1 → leg 6.


Main Loop Structure
-------------------

The main loop performs:

1. Read new PLC pose  
2. Apply heave correction  
3. Compute IK and leg lengths  
4. Apply dynamic reference  
5. Publish updated actuator commands  

The final, working implementation is:

.. code-block:: cpp

    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);

        Pose p = adsHandler.getPose();

        double surge = -p.surge;
        double sway  = p.sway;
        double heave = p.heave + 1.205;   // IK heave correction
        double roll  = -p.roll;
        double pitch = p.pitch;
        double yaw   = p.yaw;

        double L[6];
        ComputeLengths(surge, sway, heave, roll, pitch, yaw, L);

        if (!ref_initialized)
        {
            for (int i = 0; i < 6; i++)
                L_ref[i] = L[i];
            ref_initialized = true;
        }

        double qi[6];
        for (int i = 0; i < 6; i++)
            qi[i] = L[i] - L_ref[i];

        std_msgs::msg::Float64MultiArray msg;
        msg.data.resize(6);
        for (int i = 0; i < 6; i++)
            msg.data[i] = qi[i];

        std::cout << "----- ROS2 ADS Bridge -----\n";
        std::cout << "Surge:  " << surge << "\n";
        std::cout << "Sway:   " << sway  << "\n";
        std::cout << "Heave:  " << heave << "   (raw PLC: " << p.heave << ")\n";
        std::cout << "Roll:   " << roll  << "\n";
        std::cout << "Pitch:  " << pitch << "\n";
        std::cout << "Yaw:    " << yaw   << "\n";

        std::cout << "Cylinder Stroke:\n";
        for (int i = 0; i < 6; i++)
            std::cout << "  qi" << i+1 << " = " << qi[i] << "\n";

        publisher->publish(msg);

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }


Building and Running the Node
-----------------------------

Inside the ROS2 workspace:

.. code-block:: bash

    colcon build
    source install/setup.bash
    ros2 run em1500_ads_bridge ads_em1500_node

Once running, the node streams real actuator displacements to Isaac Sim,
allowing the digital twin to mirror the real platform motion in real time.
