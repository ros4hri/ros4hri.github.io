.. _robots:

Compatible Robots
=================

**Any robot running ROS 2 can be easily made compatible with ROS4HRI** by
installing existing :ref:`open-source packages <packages>`. However, some robots
come with native support for the standard.

Commercial Platforms
--------------------

.. list-table::
   :widths: 25 35 40
   :header-rows: 1
   :width: 100%
   :class: robots-table

   * - Image
     - Name
     - Description
   * - .. image:: images/robots/tiago.png
          :width: 150px
          :align: center
     - **PAL TIAGo**
       (`website <https://pal-robotics.com/robot/tiago/>`__)
     - A versatile mobile manipulator used for research in manipulation, perception, and HRI. It features a 7-DoF arm and a lifting torso.
   * - .. image:: images/robots/tiago_pro.png
          :width: 150px
          :align: center
     - **PAL TIAGo Pro**
       (`website <https://pal-robotics.com/robot/tiago-pro/>`__)
     - A next-generation mobile manipulator with dual 7-DoF arms, high-torque motors, and advanced interaction capabilities (screen, LEDs).
   * - .. image:: images/robots/tiago_pro_head.png
          :width: 150px
          :align: center
     - **PAL TIAGo Pro Head**
       (`website <https://pal-robotics.com/robot/tiago-head/>`__)
     - The expressive head of the TIAGo Pro, available as a standalone unit for HRI research focusing on social signals and gaze.
   * - .. image:: images/robots/ari.png
          :width: 150px
          :align: center
     - **PAL ARI**
       (`website <https://pal-robotics.com/robot/ari/>`__)
     - A high-performance social robot designed for reception, guidance, and entertainment. It features an expressive face and gestures.

Open-Source & Research Platforms
--------------------------------

.. list-table::
   :widths: 20 20 45 15
   :header-rows: 1
   :width: 100%
   :class: robots-table

   * - Image
     - Name
     - Description
     - Link
   * - .. image:: images/robots/emorobcare.png
          :width: 150px
          :align: center
     - **EMOROBCARE**
     - A low-cost, expressive social robot developed by ICMAT/IIIA (CSIC) for autism therapy. It features a spherical base and a 3-DoF head.
     - `Website <https://www.csic.es/>`__
   * - .. image:: images/robots/reachymini.png
          :width: 150px
          :align: center
     - **Pollen Robotics Reachy Mini**
       (`website <https://www.pollen-robotics.com/reachy-mini/>`__)
     - A small, open-source desktop robot with an expressive 6-DoF head,
       a rotating body and two animated antennas. The ROS 2 driver, developed
       by IIIA (CSIC), runs directly on the robot and exposes its camera,
       microphones (with on-board speech recognition, published on
       ``/humans/voices/``) and speaker, as well as the following ROS4HRI
       skills: :ref:`skill-replay_motion`, :ref:`skill-list_motions`,
       :ref:`skill-look_at`, :ref:`skill-execute_cartesian_trajectory`,
       :ref:`skill-execute_joint_trajectory`, plus ``wake_up`` and
       ``go_to_sleep``.
     - `Documentation & installation <https://gitlab.iiia.csic.es/socialminds/robots/reachymini/reachy-mini-driver>`__

.. note::

    If you wish to see your robot listed here, please open a Pull Request on the
    `ROS4HRI website <https://github.com/ros4hri/ros4hri.github.io>`__. We will be
    happy to add your robot to the list.
