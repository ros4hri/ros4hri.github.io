.. _packages:

Open-source ROS4HRI packages
============================

The ROS4HRI ecosystem consists of several open-source packages available on GitHub.

.. admonition:: ROS version support
    
    ROS4HRI is primarily developed and tested on **ROS 2 Humble**.
    
    - **ROS 1 Noetic** is supported but in **maintenance mode** (no regular tests; no new developments; pull requests are however welcome).
    
    - Support for **ROS 2 Jazzy and Rolling** is planned but not yet available.


Core & Standards
----------------

-  ``hri_msgs``: Standard ROS messages for HRI (faces, bodies, voices, persons).

    - List of :ref:`hri_msgs_interfaces`
    - Source repository: `ros4hri/hri_msgs <https://github.com/ros4hri/hri_msgs>`_
    - Install: ``apt install ros-<distro>-hri-msgs``

-  ``hri_actions_msgs``: Standard ROS actions for HRI :ref:`intents`.

    - List of :ref:`hri_actions_msgs_interfaces`
    - Source repository: `ros4hri/hri_actions_msgs <https://github.com/ros4hri/hri_actions_msgs>`_
    - Install: ``apt install ros-<distro>-hri-actions-msgs``

-  ``human_description``: Standard URDF models for humans.

    - Source repository: `ros4hri/human_description <https://github.com/ros4hri/human_description>`_
    - Install: ``apt install ros-<distro>-human-description``

-  :ref:`libhri` and :ref:`pyhri`: libraries to ease interaction with ROS4HRI topics.

    - Source repository: `ros4hri/libhri <https://github.com/ros4hri/libhri>`_
    - Install (C++): ``apt install ros-<distro>-hri``
    - Install (Python): ``apt install ros-<distro>-pyhri``

Perception
----------

**Face**

-   ``hri_face_detect``: Detects and tracks faces, extracts facial landmarks, and estimates head pose.
    
    - Source repository: `ros4hri/hri_face_detect <https://github.com/ros4hri/hri_face_detect>`_
    - Install: *from sources*

-   ``hri_face_identification``: Face identification/recognition.

    - Source repository: `ros4hri/hri_face_identification <https://github.com/ros4hri/hri_face_identification>`_
    - Install: *from sources*

-   ``hri_emotion_recognizer``: Emotion recognition.

    - Source repository: `ros4hri/hri_emotion_recognizer <https://github.com/ros4hri/hri_emotion_recognizer>`_
    - Source repository (models): `ros4hri/hri_emotion_models <https://github.com/ros4hri/hri_emotion_models>`_
    - Install: *from sources*

**Body**

-   ``hri_body_detect``: 2D/3D body detection and tracking (supports both monocular and RGBD cameras).
    
    - Source repository: `ros4hri/hri_body_detect <https://github.com/ros4hri/hri_body_detect>`_
    - Install: *from sources*

**Audio/Voice**

-   ``asr_vosk``: A ROS4HRI-compatible wrapper around the VOSK speech recognition library.

    - Source repository: `ros4hri/asr_vosk <https://github.com/ros4hri/asr_vosk>`_
    - Source repository (models): `ros4hri/asr_vosk_language_models <https://github.com/ros4hri/asr_vosk_language_models>`_
    - Install: *from sources*

**Fusion & High-Level**

-   ``hri_person_manager``: Manages the list of tracked persons, fusing face, body, and voice IDs.

    - Source repository: `ros4hri/hri_person_manager <https://github.com/ros4hri/hri_person_manager>`_
    - Install: *from sources*

-   ``hri_face_body_matcher``: Matches detected faces to detected bodies.

    - Source repository: `ros4hri/hri_face_body_matcher <https://github.com/ros4hri/hri_face_body_matcher>`_
    - Install: ``apt install ros-<distro>-hri-face-body-matcher``

-   ``hri_engagement``: Engagement estimation.

    - Source repository: `ros4hri/hri_engagement <https://github.com/ros4hri/hri_engagement>`_
    - Install: *from sources*

Interaction & Skills (Action)
-----------------------------

**Core**

-   :ref:`rpk <rpk>`: ROS4HRI package generator.
    Generates ROS 2 skills, tasks, chatbots, or even full applications from
    templates.

    - Source repository: `ros4hri/rpk <https://github.com/ros4hri/rpk>`_
    - Install: ``pip install rpk``

-  ``hri_actions_msgs``: Standard ROS actions for HRI :ref:`intents`.

    - List of :ref:`hri_actions_msgs_interfaces`
    - Source repository: `ros4hri/hri_actions_msgs <https://github.com/ros4hri/hri_actions_msgs>`_
    - Install: ``apt install ros-<distro>-hri-actions-msgs``

-   ``std_skills``: Standard/Common skills messages.

    - Source repository: `ros4hri/std_skills <https://github.com/ros4hri/std_skills>`_
    - Install: *from sources*

-   ``architecture_schemas``: JSON schemas for skill definitions, etc.

    - Source repository: `ros4hri/architecture_schemas <https://github.com/ros4hri/architecture_schemas>`_
    - Install: *from sources*

-   ``architecture_tools``: Tools to validate skills (eg ``ament_archlint``) or easily retrieve the list of installed skills.

    - Source repository: `ros4hri/architecture_tools <https://github.com/ros4hri/architecture_tools>`_
    - Install: *from sources*

**Skills**

These packages define standard interfaces (mostly ROS Actions) for robot skills.

.. note::
    
    These packages are skill *definitions*, not skill *implementations*. They
    are meant to enable interoperability between different skill
    implementations. You can use the `rpk <https://github.com/ros4hri/rpk>`_
    package to quickly generate skill skeletons.

-   :ref:`Verbal communication skills <communication_skills>` (TTS, etc.)

    - Source repository: `ros4hri/communication_skills <https://github.com/ros4hri/communication_skills>`_
    - Install: *from sources*

-   :ref:`Skills for social interaction <interaction_skills>`

    - Source repository: `ros4hri/interaction_skills <https://github.com/ros4hri/interaction_skills>`_
    - Install: *from sources*

-   :ref:`Navigation skills <navigation_skills>`

    - Source repository: `ros4hri/navigation_skills <https://github.com/ros4hri/navigation_skills>`_
    - Install: *from sources*

-   :ref:`Motions/gesture skills <motions_skills>`

    - Source repository: `ros4hri/motions_skills <https://github.com/ros4hri/motions_skills>`_
    - Install: *from sources*

-   :ref:`Manipulation skills <manipulation_skills>`

    - Source repository: `ros4hri/manipulation_skills <https://github.com/ros4hri/manipulation_skills>`_
    - Install: *from sources*


**User Interface**

-   ``ui_server``: A ROS 2 display server that renders interactive QML content.
    
    - Source repository: `ros4hri/ui_server <https://github.com/ros4hri/ui_server>`_
    - Install: *from sources*

-   ``ui_msgs``: ROS 2 messages used by ``ui_server``.
    
    - Source repository: `ros4hri/ui_msgs <https://github.com/ros4hri/ui_msgs>`_
    - Install: *from sources*

-   ``ros-qml-plugin``: QML plugin for ROS.
    
    - Source repository: `ros4hri/ros-qml-plugin <https://github.com/ros4hri/ros-qml-plugin>`_
    - Install: *from sources*

Visualization & Tools
---------------------

- ``hri_visualization``: Generates a camera image overlay with faces, bodies, emotions, etc.

    - Source repository: `ros4hri/hri_visualization <https://github.com/ros4hri/hri_visualization>`_
    - Install: *from sources*

- ``hri_rviz``: RViz plugins for visualizing HRI data (Faces, Bodies, etc.).

    - Source repository: `ros4hri/hri_rviz <https://github.com/ros4hri/hri_rviz>`_
    - Install: ``apt install ros-<distro>-hri-rviz``

- ``rqt_human_radar``: An rqt plugin to visualize humans in a radar-like view.

    - Source repository: `ros4hri/rqt_human_radar <https://github.com/ros4hri/rqt_human_radar>`_
    - Install: *from sources*

- ``rqt_chat``: Chat interface for debugging.

    - Source repository: `ros4hri/rqt_chat <https://github.com/ros4hri/rqt_chat>`_
    - Install: *from sources*

- ``interaction_sim``: ROS 2 simulation environment for HRI research.

    - Source repository: `ros4hri/interaction_sim <https://github.com/ros4hri/interaction_sim>`_
    - Install: *from sources*
