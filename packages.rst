.. _packages:

Packages
========

The ROS4HRI ecosystem consists of several open-source packages available on GitHub.

Core & Standards
----------------

-   `hri_msgs <https://github.com/ros4hri/hri_msgs>`_: Standard ROS messages for HRI (faces, bodies, voices, persons).
-   `hri_actions_msgs <https://github.com/ros4hri/hri_actions_msgs>`_: Standard ROS actions for HRI intents.
-   `human_description <https://github.com/ros4hri/human_description>`_: Standard URDF models for humans.
-   `libhri <https://github.com/ros4hri/libhri>`_: C++ & Python library to ease interaction with ROS4HRI topics.

Perception
----------

**Face**

-   `hri_face_detect <https://github.com/ros4hri/hri_face_detect>`_: Detects and tracks faces, extracts facial landmarks, and estimates head pose.
-   `hri_face_identification <https://github.com/ros4hri/hri_face_identification>`_: Face identification/recognition.
-   `hri_emotion_recognizer <https://github.com/ros4hri/hri_emotion_recognizer>`_: Emotion recognition.

**Body**

-   `hri_body_detect <https://github.com/ros4hri/hri_body_detect>`_: 2D/3D body
    detection and tracking (supports both monocular and RGBD cameras).

**Audio/Voice**

-   `asr_vosk <https://github.com/ros4hri/asr_vosk>`_: A ROS4HRI-compatible wrapper around the VOSK speech recognition library.
-   `asr_vosk_language_models <https://github.com/ros4hri/asr_vosk_language_models>`_: Language models for ``asr_vosk``.

**Fusion & High-Level**

-   `hri_person_manager <https://github.com/ros4hri/hri_person_manager>`_: Manages the list of tracked persons, fusing face, body, and voice IDs.
-   `hri_face_body_matcher <https://github.com/ros4hri/hri_face_body_matcher>`_: Matches detected faces to detected bodies.
-   `hri_engagement <https://github.com/ros4hri/hri_engagement>`_: Engagement estimation.

Interaction & Skills (Action)
-----------------------------

** Core **

-   `hri_actions_msgs <https://github.com/ros4hri/hri_actions_msgs>`_: Standard ROS actions for HRI intents.

** Skills **

These packages define standard interfaces (mostly ROS Actions) for robot skills.

.. note::
    
    These packages are skill *definitions*, not skill *implementations*. They
    are meant to enable interoperability between different skill
    implementations.

-   `std_skills <https://github.com/ros4hri/std_skills>`_: Standard/Common skills messages.
-   `communication_skills <https://github.com/ros4hri/communication_skills>`_: Skills related to communication (TTS, gestures).
-   `interaction_skills <https://github.com/ros4hri/interaction_skills>`_: Skills for social interaction.
-   `navigation_skills <https://github.com/ros4hri/navigation_skills>`_: Navigation skills.
-   `motions_skills <https://github.com/ros4hri/motions_skills>`_: Motion/Gesture skills.
-   `manipulation_skills <https://github.com/ros4hri/manipulation_skills>`_: Manipulation skills.

-   `architecture_tools <https://github.com/ros4hri/architecture_tools>`_: Tools
    to validate skills (eg ``ament_archlint``) or easily retrieve the list of installed skills.

**User Interface**

-   `ui_server <https://github.com/ros4hri/ui_server>`_: A ROS 2 display server that renders interactive QML content.
-   `ui_msgs <https://github.com/ros4hri/ui_msgs>`_: ROS 2 messages used by ``ui_server``.
-   `ros-qml-plugin <https://github.com/ros4hri/ros-qml-plugin>`_: QML plugin for ROS.

Visualization & Tools
---------------------

-   `hri_visualization <https://github.com/ros4hri/hri_visualization>`_: Generates a camera image overlay with faces, bodies, emotions, etc.
-   `hri_rviz <https://github.com/ros4hri/hri_rviz>`_: RViz plugins for visualizing HRI data (Faces, Bodies, etc.).
-   `rqt_human_radar <https://github.com/ros4hri/rqt_human_radar>`_: An rqt plugin to visualize humans in a radar-like view.
-   `rqt_chat <https://github.com/ros4hri/rqt_chat>`_: Chat interface for debugging.
