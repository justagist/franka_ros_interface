#!/bin/sh
''':'
if [ "$ROS_PYTHON_VERSION" = "3" ]; then
  exec python3 "$0" "$@"
else
  exec python2 "$0" "$@"
fi
'''
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup()
d['packages'] = ['franka_moveit']
d['package_dir'] = {'': 'src'}

setup(**d)
