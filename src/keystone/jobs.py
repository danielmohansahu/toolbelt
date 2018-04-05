#! /usr/bin/env python3
"""@package docstring
 # file jobs.py
 # copyright SoftWear Automation, Inc.
 # author Daniel Sahu <d.sahu@softwearinc.com>
 # date April, 2018
"""

import time
import random
import os
import pathlib
import glob
import logging
from toolbelt.decorators import LoggingDecorator

class JobCommonTemplate(object):
    """
    Subclass for jobs.

    DO NOT INSTANTIATE THIS CLASS DIRECTLY

    """

    def __init__(self, sc):
        self._logger = logging.getLogger("JobCommonTemplate")
        self._logger.info("Initializing...")
        self.sc = sc

        # The following parameters need to be loaded before this class is initted:
        # @TODO do this better
        if not (self.name and self.palletNames and self.basePath):
            raise RuntimeError("Inheriting class needs to be initted and defined properly before JobCommon!")

        # Parameters used for loading other stuff
        self.shapeID = None
        self.fabricContour = None
        self.sewFileNames = [self.name + ".sew"]

        # Initialize fabric outline:
        self._logger.info("Initializing job.")
        self.initializeJob()

        self._logger.info("Initialized.")

    def processCurrentProduct(self, continueEvent, *nextJobArgs):
        """
        processCurrentProduct
        """
        self._logger.info("Entering: processCurrentProduct")
        self._logger.info("Args: " + str(*nextJobArgs))

        time.sleep(random.gauss(3.0, 0.5))
        continueEvent.set()
        self._logger.info("continueEvent set.")
        time.sleep(random.gauss(3.0, 0.5))

        # This is just to add some complication
        self.randomError()

        print("Ran processCurrentProduct for job: " + self.name)

        self._logger.info("Exiting: processCurrentProduct")

    ######################## PROTECTED FUNCTIONS ##############################

    @LoggingDecorator
    def initializeJob(self):
        """
        Fully instantiate a job:

        - Set and Plan job in pather
        - Get fabric path and save
        - Add shape to vision

        This function should called on initialization and after any updates are
        made to vision or the sew file.

        """

        self.setJob()
        self.addShape(setJob=False)

    @LoggingDecorator
    def setJob(self, sewFileName=None):
        """
        Sets sew file in given directory.

        sewFileName : Optional argument for multiple sew files. Defaults to None
                      which
        """

        # Check if the user specified a specific file:
        filesToPlan = None
        if sewFileName:
            # If a particular sewFile is specified set that:
            self._logger.info("Setting user specified files: " + str(sewFileName))
            filesToPlan = [sewFileName]
        else:
            # If no sew file is specified set all default sew files:
            self._logger.debug("Setting default sew files: " + str(self.sewFileNames))
            filesToPlan = self.sewFileNames

        # Go through and plan all specified files:
        for filename in filesToPlan:

            filePaths = glob.glob("**/" + filename, recursive=True)
            if len(filePaths) < 1:
                raise RuntimeError("No sew file found.")
            elif len(filePaths) < 1:
                raise RuntimeError("More than one sew file found.")
            else:
                sewFilePath = pathlib.Path(filePaths[0])

            # Make sure the file exists:
            if sewFilePath.is_file():
                self._logger.info("Setting: " + str(sewFilePath))

                with sewFilePath.open() as f:
                    fcontent = f.read()

                self.sc.path.setJob(fcontent)
                self.sc.path.planJob()

                self._logger.info("Done setting: " + str(sewFilePath))
            else:
                raise RuntimeError("Sewfile: " + str(sewFilePath) + " does not exist.")

    @LoggingDecorator
    def addShape(self, setJob=True, xOffset=0, yOffset=0, tOffset=0):
        """
        Add current shape to vision. Assumes that we need to set the job first,
        which can be changed via the optional flag "setJob"

        """

        if setJob:
            self.setJob()

        # Get shape outline:
        #@TODO define this SewBotVisionApiPoint stuff elsewhere!
        self.shapeList = self.sc.swa.VisionPointList()
        shape = self.sc.path.getFabricOutline(0.003)

        for row in shape:
            #@TODO define this SewBotVisionApiPoint stuff elsewhere!
            point = self.sc.swa.SewBotVisionApiPoint()
            point.x = float(row.x)
            point.y = float(row.y)

            self.shapeList.append(point)

        self.shapeID = self.sc.v.addShape(self.shapeList, xOffset, yOffset, tOffset)
