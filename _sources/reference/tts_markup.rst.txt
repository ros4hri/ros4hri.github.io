.. index:: TTS markup, multi-modal expression

.. _tts:

.. _tts-markup:

Multi-modal expression markup
=============================

The text passed to the :ref:`skill-say` skill (and, more generally, any text
that the robot speaks out, including chatbot responses) can contain **markup
actions**. Markup actions are embedded in the sentence, and let you
**synchronise the speech with other robot capabilities**: facial expressions,
gestures, LEDs, gaze, pauses...

For instance:

.. code-block:: text

    <set expression(happy)> <start motion(wave)> Hello! <wait motion timeout=1> <set expression(neutral)>

makes the robot say *"Hello!"* with a happy face while waving, then wait until
the waving motion is finished (or at most 1 second), and finally return to a
neutral expression.

We call such a string a **multi-modal expression**.

The reference implementation of the markup language is the SocialMinds
`dialogue_manager
<https://gitlab.iiia.csic.es/socialminds/ros4hri/dialogue_manager/-/tree/main/dialogue_manager>`__.
The default values given below are the ones of this implementation.

Syntax
------

A markup action is enclosed in angle brackets. Its full form is:

.. code-block:: text

    <verb name(arguments) timeout=seconds>

The arguments and the timeout are optional. The minimal form is therefore
``<verb name>``.

Everything outside of angle brackets is regular text, spoken by the TTS engine.
Consecutive pieces of text are spoken as one utterance; speech is *blocking*,
meaning that the markup actions following a piece of text are only executed
once that text has been spoken.

Verbs
~~~~~

The verb must be one of:

.. list-table::
   :header-rows: 1
   :widths: 15 85

   * - Verb
     - Meaning
   * - ``set``
     - *Start and forget* the action. Useful when you do not need to know if
       or when the action completes.
   * - ``start``
     - Start the action, without waiting for it to complete. The action is
       tracked, and can later be waited for (``wait``) or cancelled
       (``stop``).
   * - ``wait``
     - Wait for a previously started action to complete (the most recent one
       with the same name).
   * - ``stop``
     - Stop an ongoing action (the most recent one with the same name).
   * - ``do``
     - Equivalent to ``start`` immediately followed by ``wait``: blocks until
       the action is completed.

``wait`` and ``stop`` only take the action name (and optionally a timeout), not
its arguments: ``<wait motion>``, ``<stop look_at>``.

Actions that are started (``start``, not ``set``) and neither waited for nor
stopped explicitly are **implicitly waited for at the end of the expression**.

.. note::

   Actions implemented as a ROS topic (for instance, ``expression``) cannot be
   tracked: ``start`` and ``do`` behave as ``set`` for those.

Arguments
~~~~~~~~~

Arguments are written between parentheses, like a function call. They can be
**positional** or **named** (``key=value``). Positional arguments must come
before named ones:

.. code-block:: text

    <set expression(happy)>
    <set expression(name=happy)>
    <do leds(blue, groups=[ear_leds], effect=blink)>

Supported values are:

- numbers: ``1``, ``-0.5``, ``1e-3``, ``0x1F``
- strings, either quoted (``"hello world"``, ``'hello'``) or as a bare word
  (``happy``)
- booleans and null: ``true``, ``false``, ``null``
- arrays: ``[ear_leds, back_leds]``

Timeout
~~~~~~~

The optional ``timeout=<seconds>`` specifies the maximum time to wait for the
action to complete. If omitted, an implementation-defined default is used (10
seconds in ``dialogue_manager``).

.. code-block:: text

    <do motion(bow) timeout=3>
    <wait motion timeout=1>

In addition, the complete multi-modal expression has a global timeout (60
seconds by default in ``dialogue_manager``), after which all ongoing
actions are cancelled.

Built-in actions
----------------

Built-in actions resemble regular markup actions, but do not take a verb.

The only built-in action currently is:

- ``<pause(seconds)>``: stay silent for the given duration.

For instance:

.. code-block:: text

    <start motion(wave)> Hello! <pause(2)> Anything new going on? <wait motion>

makes the robot stay silent for 2 seconds after saying *"Hello!"*.

Available actions
-----------------

The available markup actions are defined by the component executing the
multi-modal expression (typically, the dialogue manager), and map to standard
:ref:`skills <skills>`. The default ones are:

.. list-table::
   :header-rows: 1
   :widths: 40 30 30

   * - Markup action
     - Arguments
     - Underlying skill
   * - ``expression(name="neutral")``
     - ``name``: the expression to display (eg ``happy``, ``sad``,
       ``amazed``...)
     - :ref:`skill-set_expression`
   * - ``motion(name)``
     - ``name``: a pre-recorded motion (see :ref:`skill-list_motions`)
     - :ref:`skill-replay_motion`
   * - ``leds(color, groups, effect="solid_color", duration, alpha=1.0, secondary_color, secondary_alpha=1.0, cycle=1.0, partition=1.0)``
     - see :ref:`skill-do_led_effect`
     - :ref:`skill-do_led_effect`
   * - ``look_at(x=1.0, y, z, frame="base_link", policy="glance")``
     - target point, in the given ``frame``
     - :ref:`skill-look_at`

Default values are shown in the signatures above. When unsure of the argument
order, prefer named arguments: ``<set look_at(x=1.0, y=0.5, z=1.2)>``.

.. note::

   Implementations can disable specific markup actions (for instance,
   ``dialogue_manager`` exposes a ``disabled_markup_actions`` parameter;
   ``motion`` is disabled by default). Disabled actions are silently skipped.

Variables
---------

Both the spoken text and the argument values can refer to **variables**, using
the syntax ``@name|default`` (or ``@name.field|default``). If the variable is
not provided by the executing component, the default value is used:

.. code-block:: text

    Hello @user.name|"there"! <set expression(@mood|neutral)>

In the spoken text, the default value must be a quoted string.

Error handling
--------------

If a multi-modal expression cannot be parsed (for instance, because of an
unterminated ``<``), the whole string is spoken as-is, without executing any
action.

Examples
--------

Change expression while speaking:

.. code-block:: text

    <set expression(happy)> I'm the famous emotion mirroring robot! My holiday was great <set expression(amazed)>, thank you for asking! <set expression(neutral)>

Look somewhere, then speak once the head movement is completed:

.. code-block:: text

    <do look_at(x=1.0, y=-0.5, z=1.0, policy="glance")> Over there is the kitchen.

Blink the LEDs while talking, and stop them afterwards:

.. code-block:: text

    <start leds(blue, groups=[ear_leds], effect=blink)> Let me think about it... <stop leds> I've got it!

See also
--------

- :ref:`skill-say`
- :ref:`communication_skills`
