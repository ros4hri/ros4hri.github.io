import os
import json
import datetime
from pathlib import Path
from jinja2 import Template
from jsonschema import validate, ValidationError
from src.architecture_tools.pal_arch_tools.pal_arch_tools import *


def load_template(template_name):
    """Load a Jinja2 template from the tpl/ directory."""
    template_path = Path(__file__).parent / "tpl" / template_name
    with open(template_path, 'r') as f:
        return f.read()


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
    import re
    name = re.sub('(.)([A-Z][a-z]+)', r'\\1_\\2', name)
    name = re.sub('__([A-Z])', r'_\\1', name)
    name = re.sub('([a-z0-9])([A-Z])', r'\\1_\\2', name)

    return name.lower()


if __name__ == "__main__":
    base_dir = Path(__file__).parent
    skills_dir = base_dir / "src"
    os.chdir(skills_dir)

    print("\n\nComponents to document:")
    for c in get_components():
        print(f"- {c.component_type} '{c.id}' (from package '{c.from_package}'):")

    # create the skills directory if it doesn't exist
    os.makedirs(base_dir / "skills", exist_ok=True)

    # Load templates
    skill_template_str = load_template("skill.rst.j2")
    index_template_str = load_template("skills_index.rst.j2")

    # generate the documentation for each component as rst files in the skills/ directory
    components = get_components()

    # Build consolidated skills data for JSON export
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
            c["primary_domain"] = c["functional_domains"][0]
            c["snakename"] = snake_name(c["id"])
            c["classname"] = datatype.split('/')[2].capitalize()
            f.write(Template(skill_template_str).render(**c))

        # Build skill data for JSON export
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
        f.write(Template(index_template_str).render(
            components=components, skills_by_domain=skills_by_domain))

    # Load the skill schema
    schema_path = base_dir / "src/architecture_schemas/schemas/skill.schema.json"
    with open(schema_path, 'r') as f:
        skill_schema = json.load(f)

    # Validate each skill against the schema
    print("\nValidating skills against schema...")
    validation_errors = []
    for skill in skills_data:
        try:
            # Create a skill object with only the fields defined in the schema
            skill_for_validation = {
                'id': skill['id'],
                'description': skill['description'],
                'interface': skill['interface'],
                'datatype': skill['datatype'],
            }

            # Add optional fields if they exist
            if 'default_path' in skill:
                skill_for_validation['default_interface_path'] = skill['default_path']
            if 'parameters' in skill and skill['parameters']:
                skill_for_validation['parameters'] = skill['parameters']
            if 'functional_domains' in skill:
                skill_for_validation['functional_domains'] = skill['functional_domains']

            validate(instance=skill_for_validation, schema=skill_schema)
            print(f"  ✓ {skill['id']}")
        except ValidationError as e:
            validation_errors.append((skill['id'], str(e)))
            print(f"  ✗ {skill['id']}: {e.message}")

    if validation_errors:
        print(
            f"\n⚠ Warning: {len(validation_errors)} skill(s) failed schema validation")
        print("Proceeding with export anyway, but please review the errors above.")
    else:
        print(f"\n✓ All {len(skills_data)} skills validated successfully!")

    # Export consolidated skills to JSON
    skills_json_path = base_dir / "skills.json"
    with open(skills_json_path, 'w') as f:
        json.dump({
            'metadata': {
                'generated_on': datetime.datetime.now().isoformat(),
                'total_skills': len(skills_data),
            },
            'skills': skills_data,
        }, f, indent=2, ensure_ascii=False)

    print(f"Exported {len(skills_data)} skills to {skills_json_path}")
    print("Documentation generated successfully.")
