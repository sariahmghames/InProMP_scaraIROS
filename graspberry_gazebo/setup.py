#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['graspberry_gazebo', 'graspberry_planning'],
    package_dir={'': 'src'},
    requires=['numpy', 'PyYaml', 'rospkg', 'tf', 'Pillow', 'SimpleDataTransport']
)

setup(**setup_args)


