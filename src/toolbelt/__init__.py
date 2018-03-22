#!/usr/bin/env python3
import pkgutil

# This loop simply imports all the subpackages and modules one level below the current package.
for _, name, _ in pkgutil.walk_packages(__path__, __package__.split(".")[-1]+"."):
    # Only load from one level below!:
    if name.split(".")[-2] == __name__:
        __import__(name.split(".")[-1], globals=globals(), locals=locals(), level=1)

del(pkgutil, name)
