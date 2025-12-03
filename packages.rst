.. _packages:

Open-source ROS4HRI packages
============================

.. image:: images/packages.webp
   :alt: ROS4HRI packages
   :align: center
   :width: 20%


The ROS4HRI ecosystem consists of several open-source packages available on GitHub, organized by feature area.

.. admonition:: ROS version support
    
    ROS4HRI is primarily developed and tested on **ROS 2 Humble**.
    
    - **ROS 1 Noetic** is supported but in **maintenance mode** (no regular tests; no new developments; pull requests are however welcome).
    
    - Support for **ROS 2 Jazzy and Rolling** is planned but not yet generally available.

.. hint::

    You are the author of a ROS4HRI-compliant node? We would love to feature it in this list! Open `a
    discussion on the ROS4HRI GitHub organization
    <https://github.com/orgs/ros4hri/discussions/new?category=ideas>`_, or
    better yet, **directly create a Pull Request** `against this file
    <https://github.com/ros4hri/ros4hri.github.io/blob/main/packages.rst>`_.


Core Libraries & Messages
--------------------------

.. list-table::
   :header-rows: 1
   :widths: 30 40 30
   :width: 100%

   * - Package
     - Description
     - Installation
   * - :ref:`hri_msgs_interfaces`
     - Standard ROS messages for HRI (faces, bodies, voices, persons)
     - ``apt install ros-<distro>-hri-msgs``
   * - :ref:`hri_actions_msgs_interfaces`
     - Standard ROS actions for HRI :ref:`intents`
     - ``apt install ros-<distro>-hri-actions-msgs``
   * - `human_description <https://github.com/ros4hri/human_description>`_
     - Standard URDF models for humans
     - ``apt install ros-<distro>-human-description``
   * - :ref:`libhri`
     - C++ library to ease interaction with ROS4HRI topics
     - ``apt install ros-<distro>-hri``
   * - :ref:`pyhri`
     - Python library to ease interaction with ROS4HRI topics
     - ``apt install ros-<distro>-pyhri``


Perception
----------

Face Perception
+++++++++++++++

Packages implementing face-related perception capabilities as defined in
`REP-155 -- Faces <https://www.ros.org/reps/rep-0155.html#faces>`_.

.. list-table::
   :header-rows: 1
   :widths: 25 10 10 10 10 10 25
   :width: 100%

   * - Package
     - Detection
     - Landmarks
     - Gaze
     - Expression
     - Recognition
     - Installation
   * - `hri_face_detect <https://github.com/ros4hri/hri_face_detect>`_
     - ✓
     - ✓
     - ✓
     - 
     - 
     - *from sources*
   * - `hri_emotion_recognizer <https://github.com/ros4hri/hri_emotion_recognizer>`_
     - 
     - 
     - 
     - ✓
     - 
     - *from sources*
   * - `hri_face_identification <https://github.com/ros4hri/hri_face_identification>`_
     - 
     - 
     - 
     - 
     - ✓
     - *from sources*


Body Perception
+++++++++++++++

Packages implementing body-related perception capabilities as defined in
`REP-155 -- Bodies <https://www.ros.org/reps/rep-0155.html#bodies>`_.

.. list-table::
   :header-rows: 1
   :widths: 30 10 10 10 10 10 25
   :width: 100%

   * - Package
     - 2D Skeleton
     - 3D Skeleton
     - Hands
     - TF frames
     - Posture/Gestures
     - Installation
   * - `hri_body_detect <https://github.com/ros4hri/hri_body_detect>`_
     - ✓
     - ✓
     - 
     - ✓
     - (basic) 
     - *from sources*


Voice & Audio Perception
++++++++++++++++++++++++

Packages implementing voice-related perception capabilities as defined in
`REP-155 -- Voices <https://www.ros.org/reps/rep-0155.html#voices>`_.

.. list-table::
   :header-rows: 1
   :widths: 30 10 10 10 10 10 25
   :width: 100%

   * - Package
     - Audio Features
     - Voice Activity Detection
     - Speech Recognition
     - Direction of speech
     - Speaker ID
     - Installation
   * - `asr_vosk <https://github.com/ros4hri/asr_vosk>`_
     - 
     -
     - ✓ (``vosk``)
     - 
     -
     - *from sources*

Person Tracking & Fusion
++++++++++++++++++++++++

Packages implementing person-level tracking and multi-modal fusion as defined in
`REP-155 -- Persons <https://www.ros.org/reps/rep-0155.html#persons>`_.

.. list-table::
   :header-rows: 1
   :widths: 30 10 10 10 10 25
   :width: 100%

   * - Package
     - Face/Body Matching
     - Face/Voice Matching
     - Person Fusion
     - Engagement
     - Installation
   * - `hri_face_body_matcher <https://github.com/ros4hri/hri_face_body_matcher>`_
     - ✓
     -
     - 
     - 
     - ``apt install ros-<distro>-hri-face-body-matcher``
   * - `hri_person_manager <https://github.com/ros4hri/hri_person_manager>`_
     - 
     -
     - ✓
     - 
     - *from sources*
   * - `hri_engagement <https://github.com/ros4hri/hri_engagement>`_
     - 
     -
     - 
     - ✓
     - *from sources*


Interaction Skills
------------------

Standard skill definitions
++++++++++++++++++++++++++

Standard skill definitions for robot actions and behaviors. See :ref:`skills`
for the complete list.

.. list-table::
   :header-rows: 1
   :widths: 30 40 30
   :width: 100%

   * - Package
     - Skills Defined
     - Installation
   * - :ref:`communication_skills`
     - Verbal communication (Say, Chat, Ask)
     - *from sources*
   * - :ref:`interaction_skills`
     - Social interaction (LookAt, LookFor, LED effects, Expressions)
     - *from sources*
   * - :ref:`navigation_skills`
     - Navigation (Navigate, Waypoints, Zones)
     - *from sources*
   * - :ref:`motions_skills`
     - Motion execution (Joint/Cartesian trajectories)
     - *from sources*
   * - :ref:`manipulation_skills`
     - Manipulation (Grasp, Place, Hand control)
     - *from sources*

.. note::
    
    These packages are skill *definitions*, not implementations. They define standard ROS action interfaces to enable interoperability. Use :ref:`rpk <rpk>` to generate skill implementation skeletons.


Development Tools
+++++++++++++++++

.. list-table::
   :header-rows: 1
   :widths: 30 40 30
   :width: 100%

   * - Package
     - Description
     - Installation
   * - :ref:`rpk <rpk>`
     - ROS4HRI package generator for skills, tasks, and applications
     - ``pip install rpk``
   * - `architecture_schemas <https://github.com/ros4hri/architecture_schemas>`_
     - JSON schemas for skill definitions and validation
     - *from sources*
   * - `architecture_tools <https://github.com/ros4hri/architecture_tools>`_
     - Tools to validate and discover skills (``ament_archlint``)
     - *from sources*
   * - `std_skills <https://github.com/ros4hri/std_skills>`_
     - Standard/common skills messages
     - *from sources*


Visualization & Debugging
--------------------------

.. list-table::
   :header-rows: 1
   :widths: 30 40 30
   :width: 100%

   * - Package
     - Description
     - Installation
   * - `hri_rviz <https://github.com/ros4hri/hri_rviz>`_
     - RViz plugins for visualizing HRI data (faces, bodies, persons)
     - ``apt install ros-<distro>-hri-rviz``
   * - `hri_visualization <https://github.com/ros4hri/hri_visualization>`_
     - Camera image overlay with faces, bodies, emotions
     - *from sources*
   * - `rqt_human_radar <https://github.com/ros4hri/rqt_human_radar>`_
     - rqt plugin for radar-like human visualization
     - *from sources*
   * - `rqt_chat <https://github.com/ros4hri/rqt_chat>`_
     - Chat interface for debugging dialogue systems
     - *from sources*
   * - `interaction_sim <https://github.com/ros4hri/interaction_sim>`_
     - ROS 2 simulation environment for HRI research
     - *from sources*


Building User Interface
-----------------------

.. list-table::
   :header-rows: 1
   :widths: 30 40 30
   :width: 100%

   * - Package
     - Description
     - Installation
   * - `ui_server <https://github.com/ros4hri/ui_server>`_
     - ROS 2 display server for interactive QML content
     - *from sources*
   * - `ui_msgs <https://github.com/ros4hri/ui_msgs>`_
     - ROS 2 messages for UI server
     - *from sources*
   * - `ros-qml-plugin <https://github.com/ros4hri/ros-qml-plugin>`_
     - QML plugin for ROS integration
     - *from sources*
