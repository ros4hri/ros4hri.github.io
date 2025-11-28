import os
from pathlib import Path
from jinja2 import Template
from src.architecture_tools.pal_arch_tools.pal_arch_tools import *

SKILL_TEMPLATE = """.. index:: {{id}}

.. _{{ component_type }}-{{ id }}:

{{ component_type | capitalize }} ``{{ id }}``
-----------------------------------------------------------

- Interface: :{{ interface }}:`{{ default_interface_path }}`
- Message type: :msg:`{{ datatype }}`

{{ (description | wordwrap(80,break_on_hyphens=False)) if description }}


{% if "parameters" in s %}
{% if "in" in parameters %}

Input parameters
~~~~~~~~~~~~~~~~

{% for p in parameters["in"] %}

- **{{ p.name }}** {{ p.type if p.type.startswith(':msg:') else "``" + p.type + "``" }}{{ ", default: ``" + (p.default|string if (p.type != "string" or (p.type == "string" and p.default != "")) else "\"\"") + "``" if "default" in p }}{{ ", *required*" if "required" in p and p.required }}

{% filter indent(width=2, first=True) %}
{{ p.description | wordwrap(80, break_on_hyphens=False) }}

{% endfilter %}

{% endfor %}
{% endif %}

{% if "out" in parameters %}

Output fields
~~~~~~~~~~~~~

{% for p in parameters["out"] %}

- **{{ p.name }}** {{ p.type if p.type.startswith(':msg:') else "``" + p.type + "``" }}

{% filter indent(width=2, first=True) %}
{{ p.description | wordwrap(80, break_on_hyphens=False) }}

{% endfilter %}

{% endfor %}
{% endif %}

{% if "feedback" in parameters %}

Feedback fields
~~~~~~~~~~~~~~~


{% for p in parameters["feedback"] %}

- **{{ p.name }}** {{ p.type if p.type.startswith(':msg:') else "``" + p.type + "``" }}

{% filter indent(width=2, first=True) %}
{{ p.description | wordwrap(80, break_on_hyphens=False) }}

{% endfilter %}

{% endfor %}
{% endif %}

{% else %}
*Parameters not available*
{% endif %}

Quick snippets
~~~~~~~~~~~~~~

{% if interface == 'action' %}
.. code-block:: sh
    :caption: Call the skill from the command-line

    $ ros2 action send_goal {{ default_interface_path }} {{ datatype }} # then press Tab to complete the message prototype
{% elif interface == 'service' %}
.. code-block:: sh
    :caption: Call the skill from the command-line

    $ ros2 service call {{ default_interface_path }} {{ datatype }} # then press Tab to complete the message prototype
{% elif interface == 'topic' %}
.. code-block:: sh
    :caption: Trigger the skill from the command-line

    $ ros2 topic pub {{ default_interface_path }} {{ datatype }} # then press Tab to complete the message prototype
{% endif %}

{% if primary_domain == section %}
.. _skill_{{ id }}_examples:
{% endif %}

How to use in your code
~~~~~~~~~~~~~~~~~~~~~~~

.. tabs:: 

  .. tab:: ROS 2 action (C++)

    See code samples for the corresponding :{{ interface }}:`{{default_interface_path}}`: :ref:`code samples <{{ interface }}_{{ default_interface_id }}_examples>`

  .. tab:: ROS 2 action (Python)

    See code samples for the corresponding :{{ interface }}:`{{default_interface_path}}`: :ref:`code samples <{{ interface }}_{{ default_interface_id }}_examples>`

  .. tab:: QML

    You can call this skill from QML using the following code snippet. See :ref:`qml-intro` to learn more.

    .. code-block:: qml

        import Ros 2.0
        
        // ...
        {{ id.split('_') | map('capitalize') | join }}Skill {
            id: mySkill

            {% if interface == 'action' or interface == "service" -%}
            onResult: {
                console.log("Skill result: " + result);
            }
            {% endif -%}
            {% if interface == 'action' -%}
            onFeedback: {
                console.log("Skill feedback: " + feedback);
            }{% endif %}

        }

        // Call the skill
        mySkill.{{id}}({{ parameters["in"] | selectattr("required") | map(attribute="name") | join(", ") if ("parameters" in s and "in" in parameters) else "" }});




"""

INDEX_TEMPLATE = """Skills
========

This is the list of all skills available in ROS4HRI. Each skill is documented in its own page.

.. toctree::
   :maxdepth: 1

   {% for c in components %}
   {{ c.component_type }}-{{ c.id }}{% endfor %}


"""

if __name__ == "__main__":
    base_dir = Path(__file__).parent
    skills_dir = base_dir / "src"
    os.chdir(skills_dir)

    print("Components to document:")
    for c in get_components():
        print(f"- {c.component_type} '{c.id}' (from package '{c.from_package}'):")

    # generate the documentation for each component as rst files in the skills/ directory
    for c in get_components():
        with open(base_dir / f"skills/{c.component_type}-{c.id}.rst", "w") as f:
            f.write(Template(SKILL_TEMPLATE).render(**c))

    # generate the index of components, sorted by component type
    with open(base_dir / "skills/index.rst", "w") as f:
        f.write(Template(INDEX_TEMPLATE).render(components=get_components()))

    print("Documentation generated successfully.")
