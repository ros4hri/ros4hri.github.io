#!/usr/bin/env python3
"""
Generate ROS message interface documentation.

This script finds all ROS packages with message definitions (msg, srv, action)
in the src/ directory and generates reStructuredText documentation for each.
"""

import os
from pathlib import Path
from collections import defaultdict
from jinja2 import Template

def load_template(template_name):
    """Load a template from the tpl/ directory."""
    template_path = Path(__file__).parent / "tpl" / template_name
    with open(template_path, 'r') as f:
        return f.read()

def find_ros_packages_with_interfaces(src_dir):
    """
    Find all ROS packages in src/ that contain message definitions.
    
    Returns:
        dict: {package_name: {'msg': [...], 'srv': [...], 'action': [...]}}
    """
    packages = defaultdict(lambda: {'msg': [], 'srv': [], 'action': []})
    
    src_path = Path(src_dir)
    
    # Look for msg, srv, and action directories
    for msg_type in ['msg', 'srv', 'action']:
        for msg_file in src_path.rglob(f'*/{msg_type}/*.{msg_type}'):
            # Get package name (parent of msg/srv/action directory)
            package_name = msg_file.parent.parent.name
            msg_name = msg_file.stem
            
            packages[package_name][msg_type].append(msg_name)
    
    return dict(packages)

def generate_message_docs(base_dir, packages):
    """
    Generate .rst files for each message definition.
    
    Args:
        base_dir: Base directory of the documentation
        packages: Dictionary of packages and their interfaces
    """
    interfaces_dir = base_dir / "interfaces"
    interfaces_dir.mkdir(exist_ok=True)
    
    # Load templates
    message_template_str = load_template("interface_message.rst.j2")
    package_index_template_str = load_template("interface_package_index.rst.j2")
    
    for package, interfaces in packages.items():
        # Create package directory
        package_dir = interfaces_dir / package
        package_dir.mkdir(exist_ok=True)
        
        toctree_entries = []
        
        # Generate docs for each message type
        for msg_type in ['msg', 'srv', 'action']:
            for msg_name in sorted(interfaces[msg_type]):
                # Create individual message file
                msg_file = package_dir / f"{msg_name}.rst"
                
                title = f"``{package}/{msg_type}/{msg_name}``"
                underline = "=" * len(title)
                
                content = Template(message_template_str).render(
                    package=package,
                    msg_type=msg_type,
                    name=msg_name,
                    title=title,
                    underline=underline,
                    msg_dir=msg_type,
                    ext=msg_type
                )
                
                with open(msg_file, 'w') as f:
                    f.write(content)
                
                toctree_entries.append(f"   {package}/{msg_name}")
        
        # Generate package index
        if toctree_entries:
            package_index = package_dir.parent / f"{package}.rst"
            
            underline = "=" * len(package)
            toctree_str = "\n".join(sorted(list(set(toctree_entries))))
            
            content = Template(package_index_template_str).render(
                package=package,
                underline=underline,
                toctree_entries=toctree_str
            )
            
            with open(package_index, 'w') as f:
                f.write(content)

if __name__ == "__main__":
    base_dir = Path(__file__).parent
    src_dir = base_dir / "src"
    
    print("\n\nSearching for ROS packages with message definitions...")
    packages = find_ros_packages_with_interfaces(src_dir)
    
    if not packages:
        print("No packages with message definitions found.")
        exit(0)
    
    print(f"\nFound {len(packages)} package(s) with interfaces:")
    for package, interfaces in sorted(packages.items()):
        msg_count = len(interfaces['msg'])
        srv_count = len(interfaces['srv'])
        action_count = len(interfaces['action'])
        print(f"  - {package}: {msg_count} msg, {srv_count} srv, {action_count} action")
    
    print("\nGenerating documentation...")
    generate_message_docs(base_dir, packages)
    
    print("\nDocumentation generated successfully!")
