#!/usr/bin/env python3
"""
Generate ROS message interface documentation.

This script finds all ROS packages with message definitions (msg, srv, action)
in the src/ directory and generates reStructuredText documentation for each.
"""

import os
from pathlib import Path
from collections import defaultdict

# Template for individual message/service/action files
MESSAGE_TEMPLATE = """.. _{package}_{msg_type}_{name}:

{title}
{underline}

.. literalinclude:: ../../src/{package}/{msg_dir}/{name}.{ext}
   :language: python
   :linenos:

"""

# Template for package index
PACKAGE_INDEX_TEMPLATE = """.. _{package}_interfaces:

``{package}`` interfaces
=========================================================

This is the list of all ROS interfaces (messages, services, actions) defined in the ROS package ``{package}``.

**Source repository**:  `ros4hri/{package} <https://github.com/ros4hri/{package}>`_.

.. toctree::
   :maxdepth: 1

{toctree_entries}
"""

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
                
                content = MESSAGE_TEMPLATE.format(
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
            
            content = PACKAGE_INDEX_TEMPLATE.format(
                package=package,
                underline=underline,
                toctree_entries=toctree_str
            )
            
            with open(package_index, 'w') as f:
                f.write(content)

if __name__ == "__main__":
    base_dir = Path(__file__).parent
    src_dir = base_dir / "src"
    
    print("Searching for ROS packages with message definitions...")
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
