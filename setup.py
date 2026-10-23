from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# ROS 1 only; ROS 2 installs this module through CMake.
setup(**generate_distutils_setup(
    packages=['maritime_ins'],
    package_dir={'': 'src'},
))
