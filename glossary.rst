.. _glossary:

Glossary
========

.. auto-generated. If you manually modify this file, please *delete this comment line*

.. glossary::

  **Action**
    *Action* is a general term, and might refer to different concept depending on the context. In this documentation, it might either refer to a `ROS action <wiki.ros.org/actionlib>`_ (ie, a low-level asynchronous communication mechanism between ROS node) or a *chatbot action*, triggered in response to a verbal command (see :ref:`communication/rasa_chatbot:Custom actions`).

  **Activity**
    See :term:`Task`

  **Application**
    An application is what creates and manage the behaviour robot. Only one application can be running at a given time. It includes at least a :term:`mission controller`, and possibly many additional software components, depending on its needs.

  **ASR**
    *ASR* stands for *automatic speech recognition* (also sometimes called *speech-to-text*): the process of converting speech (recorded by the robot's microphone) into written text.

  **Capability**
    See :term:`skill`

  **Fact**
    See :term:`statement`

  **Intent**
    An abstract description (a ROS message) representing *something* that *someone* wants the robot to perform. Intents are usually published by the user-perception nodes (like the dialogue manager, or the touchscreen manager), and are processed by the :term:`mission controller`.

    Not to be confused with the chatbot *intents*, which are internal to the chatbot, and not visible outside of it. See :ref:`dialogue_management` for more about chatbots.

    *See also:* :ref:`intents`

  **Knowledge base**
    The robot's *knowledge base* is a semantic database where you can store and query *facts* (also called *statements*), and perform first-order logic reasoning.

  **Mission**
    See :term:`Mission controller`

  **Mission controller**
    The *mission controller* is the software that controls the main behaviour of the robot: it is the core software component of your application. It schedules and execute :term:`skills <skill>` based on user :term:`intents <intent>` and other application-specific needs.

  **Ontology**
    In computer science, an *ontology* is a network of semantic concepts related to each others. It is also refered as *semantic graph*. The knowledge is stored in ontologies using formalisms like RDF/OWL. On our robot, we using ontologies to store the semantic knowledge of the robot.

  **OWL**
    OWL stands for *Web Ontology Language* and it the format in which we store symbolic knowledge in our :term:`ontologies <ontology>` and knowledge base.

  **rpk**
    See :ref:`rpk`

  **Skill**
    *Skills* are 'unit' operations executed by the robot: navigating to a location, looking at a specific target, grasping an object, etc. The |sdkname| offers several skills out of the box (the *system skills*), and you can implement your own as well (*user skills*).

    *See also:* :ref:`skills`

  **Statement**
    A *statement* refers to a piece of information (formally, a <subject, predicate, object> triple), stored (or retrieved) from the robot's :term:`knowledge base`.

  **Task**
    An *task* represents a finite set of actions performed by the robot to achieve a specific goal (i.e. “Doing an Inventory”, “Getting a glass of water”, “Going to the kitchen”). In other systems, it is sometimes refered as an `activity`.

  **Triples**
    See :term:`statement`

