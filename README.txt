This ROS package is designed to act as a repository of common python tools and
scripts useful to all SoftWear Automation implementers.

The Layout is as follows:

src/toolbelt:
  Top level module wrapper. Imports all submodule classes.
src/*:
  All other submodules should be defined here, with toolbelt importing
  them explicitly. Use namespaces!!!
