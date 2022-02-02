"""
DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

This package allows python modules from the src path to be used in the
executables in the scripts path. The argument scripts is deliberately
not used here, because in ROS executables are executed with rosrun or by
launch file via roslaunch. This file is called by catkin from
CMakeLists.txt via catkin_python_setup() and should not be executed
directly.
"""

from catkin_pkg.python_setup import generate_distutils_setup

from setuptools import setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['zalmani_commander'],
    package_dir={'': 'src'},
)

setup(**setup_args)
