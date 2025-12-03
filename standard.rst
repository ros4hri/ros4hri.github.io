.. _standard:

The ROS4HRI Standard
====================

The ROS4HRI standard is defined in `REP-155
<https://www.ros.org/reps/rep-0155.html>`_. It specifies conventions and
interfaces for Human-Robot Interaction (HRI) perception in ROS.

The standard covers:

1.  :ref:`human_representation`: Modeling humans as a combination of permanent
    identities (Persons) and transient parts (Faces, Bodies, Voices).
2.  :ref:`ros4hri_topics_structure` : Naming conventions under the ``/humans/`` namespace.
3.  :ref:`kinematic_model`: TF frames and URDF models for humans.
4.  **Group Interactions**: Representation of social groups and gaze.

.. important::

    Want to suggest a change to ROS4HRI? propose an extension? `open a
    discussion on the ROS4HRI GitHub organization
    <https://github.com/orgs/ros4hri/discussions/new?category=ideas>`_ to get
    the ball rolling!

.. _human_representation:

Human Representation
--------------------

A person is represented by four unique identifiers (UUIDs):

.. image:: images/ros4hri_ids.png
   :alt: ROS4HRI IDs
   :align: center
   :width: 400px

-   **Person ID**: A permanent identifier for a unique person.
-   **Face ID**: A transient identifier for a detected face.
-   **Body ID**: A transient identifier for a detected body/skeleton.
-   **Voice ID**: A transient identifier for a detected voice.


These identifiers are linked together. For example, a face and a body might be
associated with the same person.

.. figure:: images/people_graph.png
   :alt: Example of a feature graph
   :align: center
   :width: 80%

The :ref:`hri_person_manager <hri_person_manager>` node provides tools to manage these identities and publish the relations between identifiers.

.. seealso::

    For more details on the ROS4HRI human model, refer to the `Human Representation
    section of REP-155 <https://www.ros.org/reps/rep-0155.html#human-representation>`_.


.. _ros4hri_topics_structure:

Topics Structure
----------------

All HRI-related topics are grouped under ``/humans/``.

-   ``/humans/faces/``: Face detection and recognition.

    -   ``/humans/faces/tracked``: List of currently tracked faces.
    -   ``/humans/faces/<faceID>/``: Topics for a specific face (e.g., cropped image, landmarks).

-   ``/humans/bodies/``: Body tracking.

    -   ``/humans/bodies/tracked``: List of currently tracked bodies.
    -   ``/humans/bodies/<bodyID>/``: Topics for a specific body (e.g., joint states).

-   ``/humans/voices/``: Voice activity and localization.

    -   ``/humans/voices/tracked``: List of currently tracked voices.

-   ``/humans/persons/``: High-level person tracking.

    -   ``/humans/persons/tracked``: List of currently tracked persons.
    -   ``/humans/persons/known``: List of all known persons.
    -   ``/humans/persons/<personID>/``: Topics for a specific person.

-   ``/humans/interactions/``: Group interactions and social signals.

.. seealso::

    For more details on the ROS4HRI topic structure, refer to the `Topics Structure
    section of REP-155 <https://www.ros.org/reps/rep-0155.html#topics-structure>`_.

.. _kinematic_model:

Kinematic Model and Coordinate Frames
-------------------------------------

ROS4HRI defines standard TF frames:

-   ``face_<faceID>``: Head pose (origin at sellion, x-forward, z-up).
-   ``gaze_<faceID>``: Gaze direction (z-aligned with gaze vector).
-   ``body_<bodyID>``: Body root (mid-point of hips).
-   ``voice_<voiceID>``: Sound source location.

.. image:: images/frames.png
   :alt: ROS4HRI Frames
   :align: center
   :width: 400px

A standard URDF model for humans is provided by the `human_description
<https://github.com/ros4hri/human_description>`_ package.

.. seealso::

    For more details on the ROS4HRI kinematic model, refer to the `Kinematic Model
    section of REP-155 <https://www.ros.org/reps/rep-0155.html#kinematic-model-and-coordinate-frames>`_.

See also
--------

For more details, please refer to the full `REP-155 specification <https://www.ros.org/reps/rep-0155.html>`_.
