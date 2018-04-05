#! /usr/bin/env python3
"""@package docstring
 # file pallet.py
 # copyright SoftWear Automation, Inc.
 # author Daniel Sahu <d.sahu@softwearinc.com>
 # date February, 2018
"""

import logging
from toolbelt.decorators import LoggingDecorator

class PalletTemplate(object):
    """
    The pallets class just maintains all possible pallet variations and the
    currently selected one.

    Right now it's basically just a structure, but I want the option to do more
    complex stuff in here if necessary. The idea is that any subsystem that
    has pallet specific movements will check the current pallet in this class
    in order to decide what to do. This is to get the functionality of jobSets
    without needing the heirarchy.

    API:
        - addPallet : Add a new pallet and associated jobs to class.
        - getCurrentPallet : Get the currently selected pallet.
        - setCurrentPallet : Set the currently selected pallet.

    Example Instantiation:
    >> P = Pallets()
    >> Extra1 = { 'destack' : 'method1' }
    >> Extra2 = { 'destack' : 'method2' }
    >> P.addPallet('Pallet1', ['job1', 'job2'], extraFields=Extra1)
    >> P.addPallet('Pallet2', ['job2', 'job3'], extraFields=Extra2)
    >> P.setCurrentPallet('Pallet2')


    #@TODO
    pass pallets initialization from jobcontrol
    define initialization sequence (plan all jobs and reset vision)
    define select and plan a job (via OHVS)

    basically handles all jobs / pallets / vision / planning stuff.
    """

    def __init__(self, sc):
        self._logger = logging.getLogger("Pallet")
        self.name = "Pallet"
        self.currentPallet = None
        self.sc = sc

        # These are defined by the user:
        self.jobs = {}            # Dictionary of jobnames (keys) vs. jobInstances (values)
        self.pallets = {}           # Dictionary of pallets (keys) vs. jobNames (list)

        self._logger.info("Initialized")

    ######################### Helper Functions #################################

    @LoggingDecorator
    def getShapeIndex(self):
        """
        Returns a dictionary of format {"shapeID" : "jobName"} for aid in piece
        matching.

        Only returns the shapes in the currently selected pallet
        """
        shapeIndex = {}
        for jobName in self.pallets[self.getCurrentPallet()]:
            shapeIndex[self.jobs[jobName].shapeID] = jobName

        return shapeIndex

    @LoggingDecorator
    def getJobFromCurrentPallet(self, jobName):
        """
        Returns the Job object corresponding to the given jobName.

        Returns None and warns if the object does not exist or isn't in the
        currently selected pallet.
        """

        # Check that pallet exists
        currentPallet = self.getCurrentPallet()
        if not currentPallet:
            self._logger.warning("No current pallet selected. Returning No jobs.")
            return None

        # Check that job exists
        currentJobs = self.pallets[currentPallet]
        if jobName not in currentJobs:
            self._logger.warning("jobName: " + str(jobName) + " does not exist \
                                 in current pallet: " + str(currentPallet))
            return None

        # Return job instance:
        return self.jobs[jobName]

    def setCurrentPallet(self, currentPallet):
        """
        Set the currently running pallet. This should be referenced by all
        subsystems and actuators that have pallet specific operations.

        """
        if not self.pallets:
            raise ValueError("Pallets not initialized yet.")
        elif currentPallet in self.pallets:
            self._logger.info("Setting currentPallet from: " + str(self.currentPallet) \
                              + " to: " + str(currentPallet))
            self.currentPallet = currentPallet
        else:
            raise ValueError("Invalid pallet choice selected.")

    def getCurrentPallet(self):
        """
        Get the currently running pallet. This should be referenced by all
        subsystems and actuators that have pallet specific operations.

        """
        if not self.currentPallet:
            self._logger.warning("Tried to access currentPallet before it was defined.")
            return None
        else:
            return self.currentPallet

    ############################ API Functions #################################

    def getNextProduct(self):
        """
        Template API function to be defined by the implementer. This should
        perform all necessary operations to define the next job. Returns the
        next job's name as a string.

        Examples:
         - For a system with a destacker, this would destack and use vision
            to find the next job.
         - For a system that only uses a single job this could just return that
            as a string.

        """
        self._logger.warning("getNextProduct not defined! Override with your code.")
