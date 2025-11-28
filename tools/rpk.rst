.. index:: Automatic code generation with ``rpk``

.. _rpk:

Automatic code generation with ``rpk``
======================================

The ``rpk`` tool is a command-line utility designed to streamline the
development of robotic applications in the ROS 2 ecosystem. It follows the ROS4HRI
approach to building composable and reusable robotic applications.

``rpk`` facilitates the creation and management of standardized application
skeletons for robots, enabling developers to quickly bootstrap projects
and focus on implementation-specific details. Its flexibility and
modularity support a range of robot types, making it a valuable asset
for robotics software developers.

This section provides an overview of the tool's functionality, usage,
and the templates it offers for different components of robotic
applications.

Role and purpose
----------------

``rpk`` is a quick way to start a new, clean and complete ROS 2 package.

It helps with:

-  **Standardization**: Ensures consistency across robot projects by
   using pre-defined templates adhering to ROS 2 conventions.
-  **Modularity**: Promotes the reuse of code and functionality across
   different projects and components.
-  **Ease of Use**: Simplifies the setup process for robot applications,
   reducing the learning curve for new developers.
-  **Flexibility**: Supports multiple robot types and use cases, making
   it adaptable to diverse projects.
-  **Rapid Prototyping**: Enables developers to quickly bootstrap
   complex systems, focusing on custom functionality rather than
   boilerplate code.

Main functions
--------------

The ``rpk`` tool offers two primary commands:

-  ``create``: This command generates application skeletons for
   various components of a robot system. Developers can specify the type
   of component (e.g., skill, task, mission) and select from available
   templates tailored to specific robot types or use cases.

-  ``list``: This command displays a catalog of available templates
   for intents, skills, tasks, missions, and applications. It provides
   brief descriptions of each template, including its programming
   language and example use cases.

``create`` command
~~~~~~~~~~~~~~~~~~

The ``create`` command is central to ``rpk``'s functionality, enabling
developers to generate skeletons for the following components:

1. **Intent**: Modules for extracting user intentions from inputs, such
   as chatbot systems or interfaces leveraging Large Language Models
   (LLMs) (:ref:`read more about intent recognition <def_intent>`).

2. **Skill**: Reusable, atomic robot actions. These are basic building
   blocks for tasks and missions, such as "go to" or "say" actions
   (:ref:`see the list of ROS4HRI standard skills <skills>`).

3. **Task**: Time-limited activities that combine multiple skills. Tasks
   represent intermediate levels of robot behavior, such as greeting a
   person or fetching an object.

4. **Mission**: High-level controllers managing the robot’s overall
   behavior. Missions combine tasks to define the robot’s operational
   goals, such as acting as a receptionist or waiter.

5. **Application**: Comprehensive frameworks combining mission
   controllers, tasks, and skills, along with necessary resources, to
   create complete robotic applications.

The ``create`` command supports the following options:

-  ``-r`` or ``--robot``: Specifies the target robot type. Available
   options include generic robots and predefined configurations for
   robots like PAL ARI and PAL TIAGo.

-  ``-p`` or ``--path``: Defines the directory path where the
   generated skeleton will be saved. By default, it uses the current
   directory.

``list`` command
~~~~~~~~~~~~~~~~

The ``list`` command displays available templates, categorized by
component type. For each template, it provides:

-  A unique identifier (e.g., ``base_python``).
-  A description of the template's purpose and features.
-  The programming language used (e.g., Python).

Available Templates
-------------------

``rpk`` comes preloaded with a set of templates for intents, skills,
tasks, missions, and complete applications.

As of the current version, the following templates are available:

-  **Intent Extractors**:

   -  ``basic_chatbot``: A simple chatbot skeleton.
   -  ``llm_bridge_python``: An intent extraction module utilizing LLMs
      via APIs like OpenAI's ChatGPT.

-  **Skills**:

   -  ``base_python``: A generic Python skill template.
   - ``base_cpp``: A generic C++ skill template.
   -  ``say_python``: Implements a “say” action as an example.
   -  ``db_connector_python``: A mock-up for database interaction.
   - ``locate_cpp``: Example implementation of 'locate' skill (C++).

-  **Tasks**:

   -  ``base_python``: A generic task template (Python).
   - ``simple_ui``: Simple task template with a graphical user interface (Python).
   -  ``greet_task_python``: Demonstrates a “greet” task implementation (Python).

-  **Mission Controllers**:

   -  ``base_python``: A generic mission controller template.
   -  ``base_intents_python``: A supervisor with pre-filled intent
      handlers, as well as advanced features like internationalization
      support.
   - ``base_intents_ui_python``: A robot supervisor template, with a GUI and pre-filled intent handlers.
   -  ``chatbot_supervisor_python``: A complete supervisor example, using a 
      basic chatbot to manage interactions with users.
   -  ``llm_supervisor_python``: An advanced supervisor leveraging LLMs
      for user interaction.

-  **Applications**:

   -  ``basic_chatbot_python``: complete sample app, using a basic chatbot to 
      interact with users. It includes a supervisor and sample tasks and skills 
   -  ``llm_chatbot_python``: A complete sample application featuring
      LLM-driven user interaction, with integrated mission controllers,
      tasks, and skills.

Some of these templates (the ``base_*`` ones) were designed to be
generic and extensible, showcasing best practices in ROS 2 development
(including the use of lifecycle nodes, parameter-based configuration,
etc.). They are meant to be used as starting points for FSTP users to
create their own components.

Other templates are more specific and are meant as examples of how to
integrate complete stacks.

Usage
~~~~~

You can install ``rpk`` from its `GitHub repository <https://github.com/ros4hri/rpk.git>`_, like any ROS 2 package.

Alternatively, you can install it from PyPI:

.. code:: bash

   pip install rpk

Once installed, the tool can be accessed from the command line by running
``rpk`` followed by the desired command (e.g., ``create`` or ``list``).

.. warning::

   When using a ``pip``-installed ``rpk``, you can generate templates, but you
   will not be have to build and run the generated code unless you have
   a full ROS 2 environment set up.


See also
--------

- :ref:`zero-to-llm`: this tutorial uses ``rpk`` to create a complete ROS 2 application from scratch.
