Isaac Sim Integration
=====================

This chapter explains how the ROS2 ADS bridge connects to NVIDIA Isaac Sim and drives the virtual EM1500 Stewart platform in real time. The goal is for Isaac Sim to mirror the motion of the physical platform by applying the leg-length commands published by the ROS2 node.


Overview
--------

Isaac Sim is used to visualize and validate Stewart platform motion. The ROS2 node publishes a ``Float64MultiArray`` containing six leg lengths on:

``stewart/legs_em1500``

Inside Isaac Sim, an Action Graph subscribes to this topic, extracts the array, and writes each value to the corresponding prismatic drive of each leg.


Importing the Stewart Platform
------------------------------

The Stewart platform model is created in Onshape and imported into Isaac Sim using the *Onshape for Isaac Sim* extension. This extension converts an entire Onshape assembly into a USD scene.

Importing Directly from Onshape
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

1. Open the **Extension Manager** in Isaac Sim.
2. Enable the extension:

   ``omni.isaac.onshape``

3. A new menu appears:

   **File → Import from Onshape**
4. Log into your Onshape account.
5. Select the Stewart platform assembly.
6. Click **Import**.

The importer assigns generic names (e.g., ``imported_1``). To maintain clarity, all model parts were manually renamed to meaningful identifiers and organized into folders within the Stage tree. This greatly improves readability and makes editing the USD easier.

Adding Prismatic Drives to the Actuator Joints
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The imported Onshape model does *not* include actuator drives, so each prismatic joint (e.g., ``Rod1Prismatic``) must be manually configured.

Under ``Physics → Drive``:

.. code-block::

    Drive Type: Linear
    stiffness = 2_000_000      # N/m
    damping   = 50_000         # N·s/m
    maxForce  = 500_000        # N

These values provide strong, stable actuation of each cylinder.

Adding the Stewart Platform to the Lab Environment
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To insert the Stewart platform into the main simulation file:

1. Open ``UiAMotionLab.usd``.
2. Right-click in the Stage tree → **Add → Reference**.
3. Select ``SmallStewart.usd``.
4. The platform appears at:

   ``/World/SmallStewart``

Referencing the platform ensures updates propagate automatically if the Onshape model is re-imported later.

Fixing the Platform Base
~~~~~~~~~~~~~~~~~~~~~~~~

After import, the platform is not anchored and will fall under gravity. To fix it, a **Fixed Joint** was added between the world and the lower base plate.

Joint connection:

``/World`` → ``/World/SmallStewart/BaseFrame``

This constrains the base completely and ensures only the prismatic legs control platform motion.


Action Graph Integration
------------------------

The last step is connecting ROS2 commands to the prismatic joint drives. This is handled entirely via an **Action Graph**, which performs:

1. Subscribing to the ROS2 leg-length topic  
2. Extracting six scalar values from the Float64MultiArray  
3. Writing each value to the matching prismatic joint drive  


ROS2 Subscriber Node
~~~~~~~~~~~~~~~~~~~~

The graph includes a *ROS2 Subscriber* node:

Type: ``std_msgs/msg/Float64MultiArray``

Configuration:

- **Topic Name:** ``stewart/legs_em1500``
- **Domain ID:** ``0`` (default)
- **QoS:** default

The node outputs the leg-length array on its ``data`` pin.


Splitting the Float64MultiArray Into Six Values
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Six **Array Index** nodes are used to extract individual values:

- Index ``0`` → Leg 1  
- Index ``1`` → Leg 2  
- Index ``2`` → Leg 3  
- Index ``3`` → Leg 4  
- Index ``4`` → Leg 5  
- Index ``5`` → Leg 6  

Each output is a scalar representing the target stroke length.

Because ``WritePrimAttribute`` expects arrays, each scalar is wrapped using a **Make Array** node.


Writing Values to Prismatic Joint Drives
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Each leg actuator has a corresponding **Write Prim Attribute** node.

Example for Leg 1:

::

    /World/SmallStewart/Joints/Leg1/Rod1Prismatic.drive:linear:physics:targetPosition

Configuration of each WritePrimAttribute node:

* ``prims``: RodXPrismatic joint
* ``attribute``: ``drive:linear:physics:targetPosition``
* ``values``: from MakeArray node
* ``attributeType``: left empty for auto-detection

All six WritePrimAttribute nodes update each frame, allowing Isaac Sim to follow the real Stewart platform in real time.

Leg Index Mapping in Isaac Sim
------------------------------

The Stewart platform model in Isaac Sim does not use the same leg numbering
convention as the UiA EM1500 inverse kinematics model. The IK assumes that:

- Leg indices increase **clockwise**
- Leg 1 starts at the **front-right actuator**

However, the USD model inside Isaac Sim numbers its prismatic joints
**anti-clockwise**, starting from the same physical location.

This mismatch caused incorrect behavior such as:
- Pure surge producing an unwanted rotation  
- Roll/pitch causing lateral translation  
- Motions appearing mirrored or inconsistent  

To correct this, the IK output must be reordered before sending leg commands
to Isaac Sim.

Correct Mapping
~~~~~~~~~~~~~~~

The following mapping aligns the IK legs with the Isaac Sim joint order:

=================  ===================
**IK leg index**   **Isaac Sim joint**
=================  ===================
1                  1
2                  6
3                  5
4                  4
5                  3
6                  2
=================  ===================

This re-indexing ensures that each leg length computed by the inverse
kinematics is applied to the correct prismatic joint in Isaac Sim.
