project = 'ROS4HRI: ROS for Human-Robot Interaction'
copyright = 'PAL Robotics S.L.'
author = 'Séverin Lemaignan'
release = '0.1'


# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = ['myst_parser']

templates_path = ['_templates']
exclude_patterns = ['README.md', '_build', 'Thumbs.db', '.DS_Store']


# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static', 'images']

html_css_files = ["style/main.css"]
html_logo = "images/logo_small.png"
html_favicon = "images/icon.png"
