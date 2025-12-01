import os
import yaml
import datetime
from pathlib import Path
from jinja2 import Template
from src.architecture_tools.pal_arch_tools.pal_arch_tools import *

SKILL_TEMPLATE = """.. index:: {{id}}

.. _{{ component_type }}-{{ id }}:

{{ component_type | capitalize }} ``{{ id }}``
-----------------------------------------------------------

{% if version %}
- **Version**: {{ version }}
{% endif %}
- **Default path**: ``{{ default_interface_path }}``
- **Datatype**: :{{ interface }}:`{{ datatype.fqn }}`
- **Definition source**: ``package.xml`` in `ros4hri/{{from_package}} <https://github.com/ros4hri/{{from_package}}>`_

{{ (description | wordwrap(80,break_on_hyphens=False)) if description }}


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

Quick snippets
~~~~~~~~~~~~~~


.. tabs:: 

  .. tab:: Command-line

    Call the skill from the command-line
    
{% if interface == 'action' %}
    .. code-block:: sh
    
        $ ros2 action send_goal {{ default_interface_path }} {{ datatype.fqn }} # then press Tab to complete the message prototype
{% elif interface == 'service' %}
    .. code-block:: sh
    
        $ ros2 service call {{ default_interface_path }} {{ datatype.fqn }} # then press Tab to complete the message prototype
{% elif interface == 'topic' %}
    .. code-block:: sh
    
        $ ros2 topic pub {{ default_interface_path }} {{ datatype.fqn }} # then press Tab to complete the message prototype
{% endif %}

  .. tab:: ROS 2 action (Python)

    Call the action from a Python script:

{% if interface == 'action' %}
    .. code-block:: python
    
        #!/usr/bin/env python

        import rclpy
        from rclpy.action import ActionClient
        from rclpy.node import Node

        from {{ datatype.pkg }}.action import {{ datatype.class }}

        class {{ classname }}ActionClient(Node):

            def __init__(self):
                super().__init__('{{ snakename }}_client')
                self._action_client = ActionClient(self, {{ datatype.class }}, '{{ default_interface_path }}')

            def send_goal(self, a, b):
                goal_msg = {{ datatype.class }}.Goal()

                # TODO: adapt to the action's parameters
{% if datatype.url %}
                # check {{ datatype.url }}
                # for the possible goal parameters
{% else %}
                # check the {{ datatype.name }}Goal message
                # definition for the possible goal parameters
{% endif %}
                # goal_msg.a = a
                # goal_msg.b = b

                self._action_client.wait_for_server()

                return self._action_client.send_goal_async(goal_msg)

        if __name__ == '__main__':
            rclpy.init(args=args)

            action_client = {{ datatype.classname }}ActionClient()

            # TODO: adapt to your action's parameters
            future = action_client.send_goal(a, b)

            rclpy.spin_until_future_complete(action_client, future)

            rclpy.shutdown()


{% elif interface == 'service' %}
    .. code-block:: python
    
{% elif interface == 'topic' %}
    .. code-block:: python
    
{% endif %}
  
  .. tab:: ROS 2 action (C++)

    Call the action from a C++ program:

    .. code-block:: cpp

        #include <functional>
        #include <future>
        #include <memory>
        #include <string>
        #include <sstream>
        #include <chrono>

        #include "{{ datatype.pkg }}/action/{{ datatype.snake_class }}.hpp"

        #include "rclcpp/rclcpp.hpp"
        #include "rclcpp_action/rclcpp_hpp"
        #include "rclcpp_components/register_node_macro.hpp"

        using namespace std::chrono_literals;
        using namespace std;

        class {{ classname }}ActionClient : public rclcpp::Node
        {
        public:
          using {{ datatype.class }} = {{ datatype.pkg }}::action::{{ datatype.class }};
          using GoalHandle{{ datatype.class }} = rclcpp_action::ClientGoalHandle<{{ datatype.class }}>;

          explicit {{ classname }}ActionClient(const rclcpp::NodeOptions & options)
          : Node("{{ snakename }}_action_client", options)
          {
            this->client_ptr_ = rclcpp_action::create_client<{{ datatype.class }}>(
                    this,
                    "{{ name }}");

            this->timer_ = this->create_wall_timer(
                    500ms,
                    bind(&{{ classname }}ActionClient::send_goal, this));
          }

          void send_goal()
          {
            using namespace std::placeholders;

            this->timer_->cancel();

            if (!this->client_ptr_->wait_for_action_server()) {
              RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
              rclcpp::shutdown();
            }

            auto goal_msg = {{ datatype.class }}::Goal();
  
{% if datatype.url %}
            // check {{ datatype.url }}
            // for the possible goal parameters
{% else %}
            // check the {{ datatype.name }}Goal message
            // definition for the possible goal parameters
{% endif %}
            // goal_msg.... = ...;

            RCLCPP_INFO(this->get_logger(), "Sending goal");

            auto send_goal_options = rclcpp_action::Client<{{ datatype.class }}>::SendGoalOptions();

            send_goal_options.goal_response_callback =
                bind(&{{ classname }}ActionClient::goal_response_callback, this, _1);

            send_goal_options.feedback_callback =
                bind(&{{ classname }}ActionClient::feedback_callback, this, _1, _2);

            send_goal_options.result_callback =
                bind(&{{ classname }}ActionClient::result_callback, this, _1);


            this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
          }

        private:
          rclcpp_action::Client<{{ datatype.class }}>::SharedPtr client_ptr_;
          rclcpp::TimerBase::SharedPtr timer_;

          void goal_response_callback(const GoalHandle{{ datatype.class }}::SharedPtr & goal_handle)
          {
            if (!goal_handle) {
              RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            } else {
              RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
            }
          }

          void feedback_callback(
              GoalHandle{{ datatype.class }}::SharedPtr,
              const shared_ptr<const {{ datatype.class }}::Feedback> feedback)
          {
            stringstream ss;
            ss << "Next number in sequence received: ";
            for (auto number : feedback->partial_sequence) {
              ss << number << " ";
            }
            RCLCPP_INFO(this->get_logger(), ss.str().c_str());
          }

          void result_callback(const GoalHandle{{ datatype.class }}::WrappedResult & result)
          {
            switch (result.code) {
              case rclcpp_action::ResultCode::SUCCEEDED:
                  break;
              case rclcpp_action::ResultCode::ABORTED:
                  RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                  return;
              case rclcpp_action::ResultCode::CANCELED:
                  RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                  return;
              default:
                  RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                  return;
            }
            stringstream ss;
            ss << "Result received: ";
            for (auto number : result.result->sequence) {
              ss << number << " ";
            }
            RCLCPP_INFO(this->get_logger(), ss.str().c_str());
            rclcpp::shutdown();
          }
        };  // class {{ classname }}ActionClient


        RCLCPP_COMPONENTS_REGISTER_NODE({{ classname }}ActionClient)

  .. tab:: QML

    You can call this skill from QML using the following code snippet. See :ref:`ros_qml_plugin` to learn more.

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

INDEX_TEMPLATE = """.. _skills:

ROS4HRI Standard Skills
=======================

This is the list of all skill definitions available in ROS4HRI, organised by
domain (note that some skills can be found in multiple domains).

.. note::
    You can use the :ref:`rpk <rpk>` tool to generate ROS4HRI-compliant skill skeletons.

.. note::
    You can download the complete list of ROS4HRI skills in YAML format from https://ros4hri.github.io/skills.yaml.
    

.. toctree::
   :maxdepth: 1
   :hidden:

   {% for c in components %}
   {{ c.component_type }}-{{ c.id }}{% endfor %}

{% for domain, skills in skills_by_domain.items() %}

.. _{{domain}}_skills:

{{ domain | capitalize }}
{{ "-" * domain|length }}

{% for c in skills %}
* :ref:`{{ c.component_type }}-{{ c.id }}`: {{ c.description.splitlines()[0] }}
{% endfor %}

.. note::

    - You would like to suggest a new {{domain}} skill? `Follow this link <https://github.com/ros4hri/{{domain}}_skills/issues/new?template=new-skill.md>`_.

    - You want to suggest a change to an existing {{domain}} skill? (like
      additional parameters, etc) `Follow this link <https://github.com/ros4hri/{{domain}}_skills/issues/new?template=amend-skill.md>`_.

{% endfor %}

"""


def snake_name(name):
    if name[0] == "/":
        name = name[1:]

    replacements = {
        "/*": "",
        "/": "_",
    }

    res = name.strip()
    for src, dst in replacements.items():
        res = res.replace(src, dst)

    return res


def camel_to_snake(name):

    # taken from https://stackoverflow.com/a/1176023
    name = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', name)
    name = re.sub('__([A-Z])', r'_\1', name)
    name = re.sub('([a-z0-9])([A-Z])', r'\1_\2', name)

    return name.lower()


if __name__ == "__main__":
    base_dir = Path(__file__).parent
    skills_dir = base_dir / "src"
    os.chdir(skills_dir)

    print("Components to document:")
    for c in get_components():
        print(f"- {c.component_type} '{c.id}' (from package '{c.from_package}'):")

    # create the skills directory if it doesn't exist
    os.makedirs(base_dir / "skills", exist_ok=True)

    # generate the documentation for each component as rst files in the skills/ directory
    components = get_components()
    
    # Build consolidated skills data for YAML export
    skills_data = []
    
    for c in components:
        with open(base_dir / f"skills/{c.component_type}-{c.id}.rst", "w") as f:
            # for instance 'communication_skills/action/Ask'
            datatype = c["datatype"]
            c["datatype"] = {
                "fqn": datatype,
                "pkg": datatype.split('/')[0],
                "type": datatype.split('/')[1],
                "name": datatype.split('/')[2],
                "snakename": snake_name(datatype.split('/')[2]),
            }
            c["snakename"] = snake_name(c["id"])
            c["classname"] = datatype.split('/')[2].capitalize()
            f.write(Template(SKILL_TEMPLATE).render(**c))
        
        # Build skill data for YAML export
        skill_data = {
            'id': c['id'],
            'version': c.get('version', '0.0.0'),
            'type': c['component_type'],
            'package': c['from_package'],
            'interface': c['interface'],
            'datatype': c['datatype']['fqn'],
            'default_path': c['default_interface_path'],
            'description': c['description'],
            'functional_domains': c['functional_domains'],
            'parameters': c.get('parameters', [])    
        }
        
        skills_data.append(skill_data)

    # group components by domain
    skills_by_domain = {}
    for c in components:
        for domain in c.functional_domains:
            if domain not in skills_by_domain:
                skills_by_domain[domain] = []
            skills_by_domain[domain].append(c)

    # sort domains and skills within domains
    skills_by_domain = {k: sorted(v, key=lambda x: x.id)
                        for k, v in sorted(skills_by_domain.items())}

    # generate the index of components, sorted by component type
    with open(base_dir / "skills/index.rst", "w") as f:
        f.write(Template(INDEX_TEMPLATE).render(
            components=components, skills_by_domain=skills_by_domain))
    
    # Export consolidated skills to YAML
    skills_yaml_path = base_dir / "skills.yaml"
    with open(skills_yaml_path, 'w') as f:
        yaml.dump({
            'metadata': {
                'generated_on': datetime.datetime.now().isoformat(),
                'total_skills': len(skills_data),
            },
            'skills': skills_data,
        }, f, default_flow_style=False, sort_keys=False, allow_unicode=True)
    
    print(f"Exported {len(skills_data)} skills to {skills_yaml_path}")
    print("Documentation generated successfully.")

