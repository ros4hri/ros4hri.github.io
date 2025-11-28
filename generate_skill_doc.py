import os
from pathlib import Path
from jinja2 import Template
from src.architecture_tools.pal_arch_tools.pal_arch_tools import *
    
TEMPLATE = """.. _{{ component_type }}-{{ id }}:

{{ component_type|capitalize }} ``{{ id }}``
===============================================

Functional domains: {{ functional_domains }}

Description
-----------

{{ description }}

Details
-------

Interface: {{ interface }}
Default interface path: {{ default_interface_path }}
Datatype: {{ datatype }}
Parameters: {{ parameters }}
From package: {{ from_package }}
Component type: {{ component_type }}
Primary domain: {{ primary_domain }}
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
            f.write(Template(TEMPLATE).render(**c))
            f.write("\n")

    # generate the index of components, sorted by component type
    with open(base_dir / "skills/index.rst", "w") as f:
        f.write("Skills\n")
        f.write("========\n")
        f.write("\n")
        f.write(".. toctree::\n")
        f.write("   :maxdepth: 1\n")
        f.write("\n")
        for c in sorted(get_components(), key=lambda x: x.component_type):
            f.write(f"   {c.component_type}-{c.id}\n")

    print("Documentation generated successfully.")
