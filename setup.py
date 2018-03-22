#! /usr/bin/env python3
## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from setuptools import setup, find_packages
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=find_packages("src"),
    scripts=["scripts/analyseBagMotion.py"],
    package_dir={'': 'src'})

setup(**setup_args)


# @TODO FIND OUT HOW TO AUTOMATICALLY BUILD PACKAGE INSTEAD OF IMPORTING THROUGH toolbelt.PY
#from setuptools import find_packages
#packages=find_packages(),
