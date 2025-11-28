.. _demos:

Demos
=====

This section presents a set of demonstration trajectories used to validate
the Stewart platform motion control and visualization. All demos are
executed in open-loop using predefined references and recorded as GIFs
for quick inspection.

The following demos are included:

* Pure surge motion (0.2 m amplitude, 0.05 Hz)
* Pure roll motion (10° amplitude, 0.05 Hz)
* Motion Lab wave simulation (0.4 m amplitude, 0.05 Hz)


Surge Motion: 0.2 m, 0.05 Hz
----------------------------

In this demo the platform performs a **pure surge motion** along the 
forward–backward axis. The reference trajectory is a sinusoid with

* **Amplitude:** 0.2 m
* **Frequency:** 0.05 Hz
* **Other DOFs:** held at zero

This test is useful to verify:

* Basic correctness of the inverse kinematics in a single translational DOF
* Smoothness of actuator motion for low-frequency commands
* Absence of noticeable coupling into the other DOFs

.. figure:: _static/Sway_GIF720.gif
   :alt: Stewart platform surge motion, 0.2 m amplitude at 0.05 Hz.
   :align: center

   Surge motion demo: 0.2 m amplitude, 0.05 Hz.


Roll Motion: 10°, 0.05 Hz
-------------------------

In this demo the platform performs a **pure roll motion** about the
longitudinal axis. The reference trajectory is again sinusoidal, with

* **Amplitude:** 10°
* **Frequency:** 0.05 Hz
* **Other DOFs:** held at zero

This demo illustrates:

* Rotational behavior and leg coordination for a single rotational DOF
* Symmetry of the motion about the platform’s center
* Visual confirmation that translations remain close to zero during roll

.. figure:: _static/Roll_GIF720.gif
   :alt: Stewart platform roll motion, 10 degrees at 0.05 Hz.
   :align: center

   Roll motion demo: 10° amplitude, 0.05 Hz.


Motion Lab Wave Simulation: 0.4 m, 0.05 Hz
------------------------------------------

The Motion Lab wave simulation demo emulates a simplified **wave-induced
motion** along a chosen axis (e.g. surge, heave or a combination, depending
on the test setup). The reference is a low-frequency wave signal with

* **Amplitude:** 0.4 m
* **Frequency:** 0.05 Hz
* **Profile:** sinusoidal (wave-like), representing regular sea state

This scenario demonstrates:

* The platform’s ability to reproduce smooth, continuous wave motions
* Suitability of the system for perception and motion cueing experiments
* How well the motion envelopes and actuator limits match the intended use

.. figure:: _static/Heave_GIF720.gif
   :alt: Stewart platform wave simulation, 0.4 m amplitude at 0.05 Hz.
   :align: center

   Motion Lab wave simulation demo: 0.4 m amplitude, 0.05 Hz.
