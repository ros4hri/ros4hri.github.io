import sys
import os

project = 'ROS4HRI: ROS for Human-Robot Interaction'
copyright = 'PAL Robotics S.L.'
author = 'Séverin Lemaignan'
release = '0.1'


# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

sys.path.append(os.path.abspath("_ext"))
sys.path.append(os.path.abspath("src"))

extensions = ['myst_parser',
              'sphinx_tabs.tabs',
              'ros-roles',
              'sphinx.ext.autodoc',
              'breathe',
              'sphinxcontrib.mermaid']

breathe_projects = { "libhri": "_build/doxygen/xml" }
breathe_default_project = "libhri"

autodoc_mock_imports = ["geometry_msgs", "numpy"]

templates_path = ['_templates']
exclude_patterns = ['README.md', '_build', 'Thumbs.db', '.DS_Store', 'src', '.venv']


# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
html_theme_options = {
    'includehidden': False,
}
html_static_path = ['_static', 'images']
html_extra_path = ['skills.json']

html_css_files = ["style/main.css"]
html_logo = "images/logo_small.png"
html_favicon = "images/icon.png"
