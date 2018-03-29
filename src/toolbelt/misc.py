#! /usr/bin/env python3
"""@package docstring
 # file misc.py
 # copyright SoftWear Automation, Inc.
 # author Daniel Sahu <d.sahu@softwearinc.com>
 # date February, 2018
"""

import sys
import subprocess
from threading import Thread
import os
import logging
import rospy

class SoftwearExceptionHandler(object):
    """
    ExceptHook override; intended to be used in conjunction with a TryExceptDecorator
    """
    def __init__(self):
        self._logger = SoftwearLogger("ExceptionHandler")
        self._logger.debug("Initialization")

    def softwearExceptHook(exceptionType, exceptionValue, traceback):
        """
        @TODO Log the exception here and then choose what to do based on the exception type.
        """
        self._logger.error(exceptionType + ": " + exceptionValue)
        print(tracback.print_tb())

class SoftwearLogger(object):
    """
    Logging configuration for use in all projects. Basically a wrapper
    around python logging module.

    Instantiation:
    my_logger = SoftwearLogger("my_logger")
    my_logger.debug("This is a debug message!")
    """

    log_file = None
    FORMAT = None
    logger = None
    instances = []

    @classmethod
    def initLoggingClass(cls, log_file_loc=None):
        """
        Method to modify log file location.
        This should be done before any loggers are instantiated to make sure
        everything is logged to the same place.
        """
        if log_file_loc:
            cls.log_file = log_file_loc
        else:
            cls.log_file = str(os.path.abspath(os.curdir)) + "/default_softwear_log.log"

        cls.FORMAT = '%(asctime)-15s; %(name)-8s; %(message)s'
        logging.basicConfig(format=cls.FORMAT, filename=cls.log_file, level=logging.DEBUG)

    def __init__(self, name):
        self.name = name
        self.logger = logging.getLogger(name)

        self.instances.append(self)

    def debug(self, message):
        """
        Log with flag = DEBUG
        """
        # self.d["log_level"] = "DEBUG"
        self.logger.debug(message)
        #self.logger.debug(message, extra=self.d)
        rospy.logdebug(message)

    def info(self, message):
        """
        Log with flag = INFO
        """
        # self.d['log_level'] = "INFO"
        # self.logger.info(message, extra=self.d)
        self.logger.info(message)
        rospy.loginfo(message)

    def warning(self, message):
        """
        Log with flag = WARNING
        """
        # self.d['log_level'] = "WARNING"
        # self.logger.warning(message, extra=self.d)
        self.logger.warning(message)
        rospy.logwarn(message)

    def error(self, message):
        """
        Log with flag = ERROR
        """
        # self.d['log_level'] = "ERROR"
        # self.logger.error(message, extra=self.d)
        self.logger.error(message)
        rospy.logerr(message)

    def critical(self, message):
        """
        Log with flag = CRITICAL
        """
        # self.d['log_level'] = "CRITICAL"
        # self.logger.critical(message, extra=self.d)
        self.logger.critical(message)
        rospy.logfatal(message)

class SoftwearThread(Thread):
    """
    This class is a carbon copy of the threading.Thread class, except the run
    method is overridden to pass any exceptions to sys.excepthook. This is
    because native threads in Python do not raise their exceptions to the
    calling thread.

    http://spyced.blogspot.com/2007/06/workaround-for-sysexcepthook-bug.html
    https://sourceforge.net/tracker/?func=detail&atid=105470&aid=1230540&group_id=5470

    Ideally this would be performed by using the decorator class TryExceptDecorator,
    but that would cause an infinite import loop because of the logging class.
    """
    def __init__(self, *args, **kwargs):
        Thread.__init__(self, *args, **kwargs)

    def __run__(self, *args, **kwargs):
        try:
            Thread.__run__(self, *args, **kwargs)
        except (KeyboardInterrupt, SystemExit):
            raise
        except:
            sys.excepthook(*sys.exc_info())

class JobCounter(object):
    """
    Simple structure for maintaining a count of the number of runs during
    continuous cycling.

    Methods:
     - ++           : Increment counters
     - clear        : Reset instance counter

    Parameters:
     - cumulative: Number incremented since instantiation
     - instance: NUmber incremented since previous clear

    """

    def __init__(self):
        self.instance = 0
        self.cumulative = 0

    def __iadd__(self, increment):
        self.instance += int(increment)
        self.cumulative += int(increment)

    def clear(self):
        """
        Clear instance count (cumulative is unchanged.)
        """
        self.instance = 0

class AWSInterface(object):
    """
    This class acts as an interface between our file system and Amazon
    Web Services.
    """
    def __init__(self, location, clientName, machineName):
        self.clientName = clientName
        self.machineName = machineName
        self.location = location

        self.awsBucket = "s3://"
        self.awsBucketAppended = self.awsBucket + self.clientName + "/" + self.machineName
        self.awsCommand = ["aws", 's3', 'mv']

    def uploadBag(self, filename):
        """
        Deprecated use; use uploadFile instead.
        """
        self.uploadFile(filename)

    def uploadFile(self, fileName):
        """
        Upload a target file to AWS
        """
        commandOut = []

        targetFilePath = fileName
        targetFilePath = targetFilePath.replace(self.location, "")
        targetFilePath = targetFilePath.replace(".archive", "")

        for item in self.awsCommand:
            commandOut.append(item)
        commandOut.append(fileName)
        commandOut.append(self.awsBucketAppended + targetFilePath)

        with open(os.devnull, 'w') as FNULL:
            subprocess.call(commandOut, stdout=FNULL, stderr=subprocess.STDOUT)

if __name__ == "__main__":
    import code

    code.interact(local=locals())
