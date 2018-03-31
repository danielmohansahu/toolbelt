#! /usr/bin/env python3
"""@package docstring
 # file excepts.py
 # copyright SoftWear Automation, Inc.
 # author Daniel Sahu <d.sahu@softwearinc.com>
 # date February, 2018
"""

import traceback

# Try to import from relative path; if we're calling as main import
if __package__:
    from .logger import SoftwearLogger as sLogger
else:
    from logger import SoftwearLogger as sLogger

class SoftwearExceptionHandler(object):
    """
    ExceptHook override; intended to be used in conjunction with a TryExceptDecorator

    Hopefully we don't cause any infinite loops here...
    """
    def __init__(self, shutdown):
        self._logger = sLogger("ExceptionHandler")
        self._logger.debug("Initialization")

        self.shutdown = shutdown

    def softwearExceptHook(self, exceptionType, exceptionValue, exceptionTraceback):
        """
        @TODO Log the exception here and then choose what to do based on the exception type.
        """
        self._logger.error(str(exceptionType) + ": " + str(exceptionValue))

        print("TRACEBACK ERROR : ")
        traceback.print_tb(exceptionTraceback)

        if isinstance(exceptionType, Exception):
            self._handle_general(exceptionValue, exceptionTraceback)

    def _handle_general(self, exceptionValue, exceptionTraceback):
        self.shutdown()
