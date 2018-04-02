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
