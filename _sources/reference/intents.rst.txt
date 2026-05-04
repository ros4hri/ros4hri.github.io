.. index:: Intents

.. role:: json(code)
   :language: json

.. _intents:

Intents
=======

**Intents** are the general mechanism used on the robot to *aggregate* user
commands, and present them to the robot's *mission controller*.

They are published on the :topic:`/intents` (:msg:`hri_actions_msgs/msg/Intent`).

What are intents?
-----------------

An :term:`intent` is an abstract description of an operation to be performed by the
robot. Intents are represented as ROS messages of type :msg:`hri_actions_msgs/msg/Intent`
and published on the :topic:`/intents` topic.

While inspired by the Android intents [android-intents]_, ROS intents
are primarily designed to capture user-initiated intents. For instance, a
button click on a touchscreen, the result of a chatbot-based verbal
interaction, a command started by a remote user interface.

Intents are emitted (*published*) by nodes that track the user's activities (eg,
the touchscreen, the dialogue manager), and are meant to be consumed by a
:term:`mission controller` (ie. the top-level controller of the robot).

Structure of an intent
----------------------

Intents comprise of four mandatory fields:

- the ``intent``, which should be one of the available predefined intents, 
- the ``data`` which must be a JSON object containing the data required to fully instantiate the intent.
- the ``source`` of the intent (for instance, an user)
- and the ``modality`` by which the intent was conveyed to the robot.

Optionally, you can also specify a ``priority`` and a level of ``confidence``.

Intent name and data
++++++++++++++++++++++

Intents are primarily composed of an *intent name* and *data* to parametrise the intent.

The ``intent`` field is a string describing the action intended by this intent.

Where suitable, the intent name SHOULD be one of the constant defined in the table below.
However, we recognise that the list of intents is possibly large. Therefore,
custom strings are also permissible.

.. attention:: 

    **Possible terminology confusion**

    Even though an intent describes a desired action, the ``intent`` field is
    unrelated to ROS actions. Here, the *intent* is the intended action to be
    performed (going somewhere, picking something...), while *ROS actions* are a
    low-level asynchronous remote procedure call (RPC) technique.

    They are not directly related (even though ROS actions can be used to
    implement intents).


The intent's ``data`` is a JSON object containing the data required to fully
specify the intent. The keys of the object should be one of the following
*thematic role*, or the generic ``other_data``:


- ``agent``: the agent expected to perform the intent (if omitted, the robot
  itself is assumed)
- ``object`` (also named *theme* or *patient* in the linguistics literature):
  entity undergoing the effect of the intent
- ``goal``: entity towards which the intent is directed or moves
- ``recipient``: entity that receives the object

Examples:

- *"I want you to go to the kitchen"*:

  - intent: ``MOVE_TO``
  - data: :json:`{"goal":"kitchen_1"}`

- *"Can you take the groceries to Luke in the kitchen?"*:

  - intent: ``BRING_OBJECT``
  - data: :json:`{"object": "groceries", "goal":"kitchen_1", "recipient": "person_luke"}`

.. note::

   Additional complete examples of intents are provided below: `Examples of intents`_.

Each intent defines a specific set of required and optional thematic roles,
listed in the following table (note that the ``agent`` role can be optionally
added to all intents, and is omitted from the table for clarity):


+--------------------+-----------------------------------------------------------------------------+---------------------------------------------------------------+-----------------------------------------------------------------------------+
| **Intent**         | **Description**                                                             | **Required thematic roles**                                   | **Optional thematic roles**                                                 |
+====================+=============================================================================+===============================================================+=============================================================================+
| ``ENGAGE_WITH``    | an agent wants to engage with another one                                   | - ``recipient``                                               |                                                                             |
+--------------------+-----------------------------------------------------------------------------+---------------------------------------------------------------+-----------------------------------------------------------------------------+
| ``MOVE_TO``        | navigates to a specific location                                            | - ``goal``                                                    |                                                                             |
+--------------------+-----------------------------------------------------------------------------+---------------------------------------------------------------+-----------------------------------------------------------------------------+
| ``GUIDE``          | guides someone somewhere                                                    | - ``goal``                                                    |                                                                             |
|                    |                                                                             | - ``recipient``                                               |                                                                             |
+--------------------+-----------------------------------------------------------------------------+---------------------------------------------------------------+-----------------------------------------------------------------------------+
| ``GRAB_OBJECT``    | pick-up a specific object                                                   | - ``object``                                                  |                                                                             |
+--------------------+-----------------------------------------------------------------------------+---------------------------------------------------------------+-----------------------------------------------------------------------------+
| ``BRING_OBJECT``   | bring a specific object to a specific place                                 | - ``object``                                                  |                                                                             |
|                    |                                                                             | - ``recipient``                                               |                                                                             |
+--------------------+-----------------------------------------------------------------------------+---------------------------------------------------------------+-----------------------------------------------------------------------------+
| ``PLACE_OBJECT``   | put an object on a support (eg a table)                                     | - ``recipient``                                               | - ``object`` (only required if more that one object could be placed)        |
+--------------------+-----------------------------------------------------------------------------+---------------------------------------------------------------+-----------------------------------------------------------------------------+
| ``GREET``          | greet an agent                                                              | - ``recipient``                                               |                                                                             |
+--------------------+-----------------------------------------------------------------------------+---------------------------------------------------------------+-----------------------------------------------------------------------------+
| ``SAY``            | says some text, optionally annotated with gestures or expressions           | - ``object`` (the text to say)                                | - ``recipient``                                                             |
+--------------------+-----------------------------------------------------------------------------+---------------------------------------------------------------+-----------------------------------------------------------------------------+
| ``PRESENT_CONTENT``| present (via a screen, pre-recorded text...) predefined content             | - ``object`` (the content identifier)                         | - ``recipient``                                                             |
+--------------------+-----------------------------------------------------------------------------+---------------------------------------------------------------+-----------------------------------------------------------------------------+
| ``PERFORM_MOTION`` | performs a motion (eg, a dance or a specific gesture like pointing, waving) | - ``object`` (the system-specific name of the motion/gesture) | - ``recipient``                                                             |
+--------------------+-----------------------------------------------------------------------------+---------------------------------------------------------------+-----------------------------------------------------------------------------+
| ``START_ACTIVITY`` | start a scripted behaviour/activity                                         | - ``object`` (the name of the activity)                       | - any additional parameter required to start the activity                   |
+--------------------+-----------------------------------------------------------------------------+---------------------------------------------------------------+-----------------------------------------------------------------------------+
| ``CANCEL_ACTIVITY``| request cancellation of an activity                                         | - ``object`` (the name of the activity)                       | - ``object`` (the name of the activity. If unset, current main activity)    |
+--------------------+-----------------------------------------------------------------------------+---------------------------------------------------------------+-----------------------------------------------------------------------------+

.. note:: 

   If you believe your intent should be standardised and added to the list of
   pre-defined intents, fill the corresponding entry in the "thematic roles"
   table below and submit a pull request on the `hri_actions_msg
   <https://github.com/ros4hri/hri_actions_msgs>`_ repository.


Source of the intent
++++++++++++++++++++

The *source* of the intent is a string describing who created this intent. This
is *not* the node which published the intent, but instead the actual agent who
expressed the intent/command/desire. ``source`` can be either one of the
constant below, or the specific id of the person/agent expressing the intent. In
a `REP-155 <https://www.ros.org/reps/rep-0155.html>`_ compliant system, this ID
must be the person ID of the agent.

.. code-block::

    # for intents originating from the robot itself
    string ROBOT_ITSELF = "__myself__"
    # for intents originating from a external robot control system (for instance, a remote control tablet)
    string REMOTE_SUPERVISOR = "__remote_supervisor__"
    # for intents coming from an agent interacting with the robot, but not uniquely
    # identified
    string UNKNOWN_AGENT = "__unknown_agent__"
    # for unknown sources
    string UNKNOWN = "__unknown__"


Modality of the intent
++++++++++++++++++++++

The intent's *modality* conveys how  the intent was expressed: verbally, via the
touchscreen, via a gesture, etc.

The special modality ``MODALITY_INTERNAL`` must be used for intents coming for the
robot's internal processes, when applicable.

The ``modality`` field MUST be one of the ``MODALITY_`` constant below.

.. code-block::

    string MODALITY_SPEECH = "speech"
    string MODALITY_MOTION = "motion"
    string MODALITY_TOUCHSCREEN = "touchscreen"
    string MODALITY_INTERNAL = "internal"
    string MODALITY_OTHER = "other"


Intent priority
+++++++++++++++

The priority of this intent. This MIGHT be used as a hint by the robot's
application controller to prioritise appropriately the intent. The application
controller is however *not* forced to respect this priority level.

0 is the lowest priority, 128 is the default priority, 255 is the highest
priority.

Intent confidence
+++++++++++++++++

The intent's *confidence* is a value between 0.0 (no confidence) and 1.0 (full
confidence) that the intent was correctly perceived and interpreted.

For instance, a 'waving' gesture could be interpreted as an implicit request
from a user for the robot to greet back or engage. As this interpretation is
not certain, the confidence of the intent may be below 1.0.

Examples of intents
-------------------

User approaches the robot
+++++++++++++++++++++++++

- **Possible intent trigger**: user less than 2 meters away, looking at the robot 
- **Possible published intent**:

.. code-block::

   intent: ENGAGE_WITH
   data: {"recipient": "anonymous_person_a2f5"}
   source: "anonymous_person_a2f5"
   modality: MODALITY_MOTION
   priority: 128
   confidence: 0.6


User presses a button on the touchscreen to navigate
++++++++++++++++++++++++++++++++++++++++++++++++++++

- **Possible intent trigger**: button press on "Go to room X" 
- **Possible published intent**:

.. code-block::

   intent: MOVE_TO
   data: {"goal": "room_X"}
   source: UNKNOWN_USER
   modality: MODALITY_TOUCHSCREEN
   priority: 200
   confidence: 1.0

User presses a button on the touchscreen to play game
+++++++++++++++++++++++++++++++++++++++++++++++++++++

- **Possible intent trigger**: button press: "Play memory game"
- **Possible published intent**:

.. code-block::

   intent: START_ACTIVITY
   data: {"object": "games_memory_game"}
   source: UNKNOWN_USER
   modality: MODALITY_TOUCHSCREEN
   priority: 100
   confidence: 1.0

User presses a button on the touchscreen to display page
++++++++++++++++++++++++++++++++++++++++++++++++++++++++

- **Possible intent trigger**: button press: "Go to page_X"

The result depends on what ``page_X`` is about:

- if ``page_X`` is a purely informational page, that does not require any additional
  robot capability (eg, does not requires the robot to speak or to move), **no
  intent** needs to be generated. As this action is 'read-only' with no impact
  on the robot, it can be handled directly by the touchscreen.

- ``page_X`` requires additional robot resources. In this case, an intent needs
  to be published:

- **Possible published intent**:

.. code-block::

   intent: PRESENT_CONTENT
   data: {"object": "page_X"}
   source: UNKNOWN_USER
   modality: MODALITY_TOUCHSCREEN
   priority: 100
   confidence: 1.0


User asks the robot to display a specific page
++++++++++++++++++++++++++++++++++++++++++++++

- **Possible intent trigger**: the chatbot recognises the command 'display page_X' 
- **Possible published intent**:

.. code-block::

   intent: PRESENT_CONTENT
   data: {"object": "page_X",
          "recipient": "anonymous_person_e4da"}
   source: "anonymous_person_e4da"
   modality: MODALITY_SPEECH
   priority: 128
   confidence: 0.4

User asks the robot to go somewhere
+++++++++++++++++++++++++++++++++++

- **Possible intent trigger**: the chatbot recognises the command 'take me to place_X' 
- **Possible published intent**:

.. code-block::

   intent: GUIDE
   data: {"goal": "place_X",
          "recipient": "person_55dc"}
   source: "person_55dc"
   modality: MODALITY_SPEECH
   priority: 128
   confidence: 0.8


User wants to cancel a task
+++++++++++++++++++++++++++

- **Possible intent trigger**: the user presses a 'cancel' button on the
  touchscreen
- **Possible published intent**:

.. code-block::

   intent: STOP_ACTIVITY
   data: {"object": "<current activity>"}
   source: UNKONOWN_USER
   modality: MODALITY_TOUCHSCREEN
   priority: 255
   confidence: 1.0

Supervisor sends command for the robot to dock via tablet
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++

- **Possible intent trigger**: a button press on a remote control tablet 
- **Possible published intent**:

.. code-block::

   intent: MOVE_TO
   data: {"goal": "poi_docking"}
   source: REMOTE_SUPERVISOR
   modality: MODALITY_TOUCHSCREEN
   priority: 255
   confidence: 1.0


How are intents used by the robot?
----------------------------------

Intents published on the ``/intents`` topic represent each of the user's desires
or commands understood by the robot.

These intents need to be acted upon by a dedicated node (or a group of nodes,
depending on the architecture design) that is called the **robot's application
controller**. The general role of the :term:`mission controller` is to
schedule and run the different :term:`capabilities<capability>` based on received
intents, and allocate robot's resources (to ensure no two actions are using eg
the arms or navigation, at the same time).

You can use any supervision technique to implement your own application
controller: simple python scripts, finite state machines, behaviour trees,
symbolic task planner: the PAL SDK does not enforce any particular
paradigm.

You can learn more about how to program applications for the robot here:
:ref:`intro-development`.

To get started, the :ref:`basic-interaction` tutorial explains how to create
your own simple Python controller. 

PAL robots normally come with a default application controller (the one
underpinning the landing demo page, see eg :ref:`ari-first-startup` for ARI)
that reacts to different types of intents. You can have a look at its source
code and use it as a reference.

When *not* to use intents?
++++++++++++++++++++++++++

There are two interaction situations where intents should not be used:
interactions with no side-effect on the robot (instead, use directly the chatbot
actions), and short confirmation

.. TODO

 (use the :ref:`UserInput <user-input>` API
 instead).

Interactions with no side-effects
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

User interactions do not always have to generate intents. In particular, during
a chatbot interaction, the chatbot engine might need perform simple *actions* to
answer the user's questions which do not impact the robot state. In this case,
it is unnecessary to generate an intent, as no complex action scheduling is
necessary.

For instance, if the user asks the robot about the weather, the chatbot can
generate an answer by querying an online weather forecast API. This does not
require any specific robot resources. Similarly, checking the battery level of
the robot has no impact on the robot state or resources.

In these cases, instead of publishing an intent, the chatbot engine can directly
perform the API requests or ROS service calls to answer the user's question.

.. note::

  :ref:`dialogue_management` explains how to create and customise your own chatbots.
  You can also specifically refer to :ref:`rasa_chatbot` or :ref:`llm_chatbot`.

.. TODO: not yet implemented!
  Short user input
  ~~~~~~~~~~~~~~~~
  
  For short user input (eg, a yes/no confirmation dialogue, a simple
  multiple-choice question, a simple text entry), you can use the robot
  multi-modal user-input mechanism :ref:`user-input`.


.. [android-intents] `Android Intents <https://developer.android.com/guide/components/intents-filters>`_
