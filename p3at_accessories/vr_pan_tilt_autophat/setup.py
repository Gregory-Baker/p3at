## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['vr_pan_tilt_autophat'],
    package_dir={'': 'src'},
    install_requires=['pi_servo_hat']
)

setup(**d)
