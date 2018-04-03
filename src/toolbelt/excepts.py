#! /usr/bin/env python3
"""@package docstring
 # file excepts.py
 # copyright SoftWear Automation, Inc.
 # author Daniel Sahu <d.sahu@softwearinc.com>
 # date February, 2018
"""

import traceback
import logging

class SoftwearExceptionHandler(object):
    """
    ExceptHook override; intended to be used in conjunction with a TryExceptDecorator

    Hopefully we don't cause any infinite loops here...
    """
    def __init__(self, shutdown):
        self._logger = logging.getLogger("SoftwearExceptionHandler")
        self._logger.debug("Initialization")

        self.shutdown = shutdown

    def softwearExceptHook(self, exceptionType, exceptionValue, exceptionTraceback):
        """
        @TODO Log the exception here and then choose what to do based on the exception type.
        """
        #@TODO add some other handling for printing traceback info
        self._logger.error(str(exceptionType.__name__)
                           + ": "
                           + str(exceptionValue))

        if isinstance(exceptionType, Exception):
            self._handle_general(exceptionValue, exceptionTraceback)

    def _handle_general(self, exceptionValue, exceptionTraceback):
        self.shutdown()
