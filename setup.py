#!/usr/bin/env python3
from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup

setup(**generate_distutils_setup(packages=["swarm_uav_executor", "swarm_uav_executor.drivers"], package_dir={"": "src"}))
