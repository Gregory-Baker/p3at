#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
     packages=['icm20948_imu'],
     package_dir={'': 'src'},
     install_requires=['icm20948']
)

setup(**setup_args)
