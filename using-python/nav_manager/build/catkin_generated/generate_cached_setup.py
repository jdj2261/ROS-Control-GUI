# -*- coding: utf-8 -*-
from __future__ import print_function
import argparse
import os
import stat
import sys

# find the import for catkin's python package - either from source space or from an installed underlay
if os.path.exists(os.path.join('/opt/ros/kinetic/share/catkin/cmake', 'catkinConfig.cmake.in')):
    sys.path.insert(0, os.path.join('/opt/ros/kinetic/share/catkin/cmake', '..', 'python'))
try:
    from catkin.environment_cache import generate_environment_script
except ImportError:
    # search for catkin package in all workspaces and prepend to path
    for workspace in "/home/djjin/Work/silbot3_ws/devel;/home/djjin/Work/test_ws/devel;/home/djjin/Work/autism_ws/devel;/home/djjin/Work/CarND-Capstone/ros/devel;/home/djjin/Work/moving_bot_ws/devel;/home/djjin/Work/social_ws/devel;/home/djjin/Work/fourth_robot_ws/devel;/home/djjin/Work/autonomous_ws/devel;/opt/ros/kinetic".split(';'):
        python_path = os.path.join(workspace, 'lib/python2.7/dist-packages')
        if os.path.isdir(os.path.join(python_path, 'catkin')):
            sys.path.insert(0, python_path)
            break
    from catkin.environment_cache import generate_environment_script

code = generate_environment_script('/home/djjin/Work/test_ws/src/nav_manager/build/devel/env.sh')

output_filename = '/home/djjin/Work/test_ws/src/nav_manager/build/catkin_generated/setup_cached.sh'
with open(output_filename, 'w') as f:
    #print('Generate script for cached setup "%s"' % output_filename)
    f.write('\n'.join(code))

mode = os.stat(output_filename).st_mode
os.chmod(output_filename, mode | stat.S_IXUSR)
