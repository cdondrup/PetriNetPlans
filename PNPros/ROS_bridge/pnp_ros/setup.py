## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['pnp_plugin_server', 'pnp_knowledgebase', 'pnp_planning_system'],
    package_dir={'': 'src'})

setup(**setup_args)
