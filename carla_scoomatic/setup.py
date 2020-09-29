"""
Setup for carla_scoomatic
"""

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['carla_scoomatic'],
    package_dir={'': 'src'}
)

setup(**d)
