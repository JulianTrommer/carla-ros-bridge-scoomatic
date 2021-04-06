"""
Setup for dynamic_obstacle_visualizer
"""

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['dynamic_obstacle_visualizer'],
    package_dir={'': 'src'}
)

setup(**d)
