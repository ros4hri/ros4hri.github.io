from docutils import nodes
from sphinx.roles import XRefRole


def make_role(type, text, lineno, inliner, options={}, content=[]):

    text = text.strip()

    # currently, only skills are supported. We create a reference to the skill's page
    if type == "skill":
        if text[0] == '/':
            text = text[1:]
        text = text.replace('*/', '').replace('/', '-')
        
        role = XRefRole(lowercase=True, innernodeclass=nodes.inline,
                        warn_dangling=True)
        modified_target = f"{type}-" + text
        refnodes, messages = role(
            'std:ref', f':ref:`{modified_target}`', f'{modified_target}', lineno, inliner, options, content)
        refnodes[0]['classes'].append('ros-interface')
        refnodes[0]['classes'].append(f'ros-{type}')
    else:
        # just display the text using a pre-formatted style, and add the class
        
        refnodes = [nodes.literal(text, text, classes=['ros-interface', f'ros-{type}'])]
        messages = []

    return refnodes, messages


def topic_role(name, rawtext, text, lineno, inliner, options={}, content=[]):
    return make_role('topic', text, lineno, inliner, options, content)


def action_role(name, rawtext, text, lineno, inliner, options={}, content=[]):
    return make_role('action', text, lineno, inliner, options, content)


def service_role(name, rawtext, text, lineno, inliner, options={}, content=[]):
    return make_role('service', text, lineno, inliner, options, content)


def param_role(name, rawtext, text, lineno, inliner, options={}, content=[]):
    return make_role('param', text, lineno, inliner, options, content)


def node_role(name, rawtext, text, lineno, inliner, options={}, content=[]):
    return make_role('node', text, lineno, inliner, options, content)


def msg_role(name, rawtext, text, lineno, inliner, options={}, content=[]):
    return make_role('msg', text, lineno, inliner, options, content)


def skill_role(name, rawtext, text, lineno, inliner, options={}, content=[]):
    return make_role('skill', text, lineno, inliner, options, content)


def setup(app):
    app.add_role_to_domain('std', 'topic', topic_role)
    app.add_role_to_domain('std', 'action', action_role)
    app.add_role_to_domain('std', 'service', service_role)
    app.add_role_to_domain('std', 'param', param_role)
    app.add_role_to_domain('std', 'node', node_role)
    app.add_role_to_domain('std', 'msg', msg_role)
    app.add_role_to_domain('std', 'skill', skill_role)

    return {
        'version': '0.1',
        'parallel_read_safe': True,
        'parallel_write_safe': True,
    }
