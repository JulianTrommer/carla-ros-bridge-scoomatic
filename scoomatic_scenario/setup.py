"""
Setup for scoomatic_scenario
"""

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['scoomatic_scenario'],
    package_dir={'': 'src'}
)

setup(**d)
