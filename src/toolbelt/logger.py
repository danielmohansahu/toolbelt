#! /usr/bin/env python3
"""@package docstring
 # file logger.py
 # copyright SoftWear Automation, Inc.
 # author Daniel Sahu <d.sahu@softwearinc.com>
 # date April, 2018
"""

import os
import logging
import rospy

class SoftwearLogger(object):
    """
    Logging configuration for use in all projects. Basically a wrapper
    around python logging module.

    Instantiation:
    my_logger = SoftwearLogger("my_logger")
    my_logger.debug("This is a debug message!")
    """

    log_file = None
    FORMAT = '%(asctime)-15s; %(name)-8s; %(message)s'
    logger = None
    instances = []

    @classmethod
    def getLogger(cls, name, log_file=None):
        """
        Return a softwear logger
        """

        if log_file:
            cls.log_file = log_file

        if not cls.log_file:
            # First time initialization:
            cls.log_file = str(os.path.abspath(os.curdir)) + "/default_softwear_log.log"
            logging.basicConfig(format=cls.FORMAT, filename=cls.log_file, level=logging.DEBUG)

        new_logger = cls.SubLogger(name, cls.FORMAT)
        cls.instances.append(new_logger)

        return new_logger

    class SubLogger:
        def __init__(self, name, FORMAT):
            self.logger = logging.getLogger(name)
            self.FORMAT = FORMAT

        def debug(self, message):
            """
            Log with flag = DEBUG
            """
            # self.FORMAT["log_level"] = "DEBUG"
            self.logger.debug(message)#, extra=self.FORMAT)
            rospy.logdebug(message)

        def info(self, message):
            """
            Log with flag = INFO
            """
            # self.FORMAT['log_level'] = "INFO"
            self.logger.info(message)#, extra=self.FORMAT)
            rospy.loginfo(message)

        def warning(self, message):
            """
            Log with flag = WARNING
            """
            # self.FORMAT['log_level'] = "WARNING"
            self.logger.warning(message)#, extra=self.FORMAT)
            rospy.logwarn(message)

        def error(self, message):
            """
            Log with flag = ERROR
            """
            # self.FORMAT['log_level'] = "ERROR"
            self.logger.error(message)#, extra=self.FORMAT)
            rospy.logerr(message)

        def critical(self, message):
            """
            Log with flag = CRITICAL
            """
            # self.FORMAT['log_level'] = "CRITICAL"
            self.logger.critical(message)#, extra=self.FORMAT)
            rospy.logfatal(message)

if __name__ == "__main__":
    import code


    l1 = SoftwearLogger.getLogger('name1')
    l2 = SoftwearLogger.getLogger('name2')

    code.interact(local=locals())
