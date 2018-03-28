#! /usr/bin/env python3
"""@package docstring
 # file control.py
 # copyright SoftWear Automation, Inc.
 # author Daniel Sahu <d.sahu@softwearinc.com>
 # date March, 2018
"""

import pkgutil
import importlib
import threading
from collections import defaultdict

# Try to import from relative path; if we're calling as main import
if __package__:
    from .misc import SoftwearLogger as sLogger
    from .misc import SoftwearThread as sThread
else:
    from misc import SoftwearLogger as sLogger
    from misc import SoftwearThread as sThread

class SingleOperation(object):
    """
    Creates a single task for use by the CycleManager.

    Note: currently it returns any output of the operation unless run in blocking
    mode. This can be done if necessary via futures so please let the maintainer
    know if this is a necessary feature.
    """

    def __init__(self, operation, keepRunning, blocking=True):
        self._logger = sLogger("SingleOperation: " + str(operation))
        self.operation = operation

        self.keepRunning = keepRunning
        self.defaultTimeout = 120
        self.blocking = blocking

    def run(self, objectToRun, inputArguments=()):
        """
        Run the predefined operation on the given objectToRun. Can be either
        blocking or non-blocking.

        Optional argument "args" allows arguments to be passed through to the
        operation being called.
        """

        if self.keepRunning.is_set():

            # Run method and then wait for next method to be defined.
            self._logger.debug("Running: " + str(objectToRun) + "." + str(self.operation))

            try:
                operationToRun = getattr(objectToRun, self.operation)
            except AttributeError:
                raise RuntimeError("Failed to run. " + str(objectToRun) + \
                                   " has no method: " + str(self.operation))

            if self.blocking:
                return operationToRun(*inputArguments)
            else:
                t = sThread(target=operationToRun,
                            args=inputArguments,
                            name=self.operation,
                            daemon=True)
                t.start()

class MasterOperation(object):
    """
    The master operation for continuous cycling. Depending on the complexity of
    your project this could be the only operation or

    This operation defaults to non-blocking. Depending on your project it might
    make more sense to have it block.
    """

    def __init__(self, operation, keepRunning, pallet, blocking=False):
        self._logger = sLogger("MasterOperation: " + str(operation))

        self.keepRunning = keepRunning
        self.continueEvent = threading.Event()
        self.defaultTimeout = 120
        self.blocking = blocking
        self.result = None

        # Get operation to run:
        try:
            self.operationToRun = getattr(pallet, operation)
        except AttributeError:
            raise RuntimeError("Failed to run. current pallet has no method: " + str(operation))

    def getGenerator(self):
        """
        Get a generator object that calls the user-defined operation and yields
        its results. After yielding it waits to continue until the continueEvent
        is set.
        """
        return self._jobGenerator()

    def getContinueEvent(self):
        """
        Get the continueEvent that triggers the generator to continue.
        See "getGenerator" for more information.
        """
        return self.continueEvent

    def _preloadNextOperation(self):
        """
        Internal method that handles calling the next operation and waiting
        for the continueEvent to be set.
        """
        # Wait until continue event is set:
        self._logger.info("Waiting for continueEvent to be set...")
        self.continueEvent.wait(self.defaultTimeout)
        if not self.continueEvent.is_set():
            self.result = None
            self._logger.warning("Timed out waiting for continueEvent to be set.")
            return
        else:
            self.continueEvent.clear()

        # Otherwise call operation:
        self.result = self.operationToRun()

    def _jobGenerator(self):
        """
        This is the main object to be called. Calling next() on this generator
        object will return the next job to run and spin off a thread that waits
        until an event is set before "getting" the next job to run.
        """

        # The first time this is called we just run the operation:
        self.result = self.operationToRun()

        # If the event is stopped during the first operation yield the result here:
        if not self.keepRunning.is_set():
            yield self.result

        # Continuously generate next job to run:
        while self.keepRunning.is_set():

            # Spin off thread that waits until an event is set before getting
            # the next object to run:
            t = sThread(target=self._preloadNextOperation,
                        name="preloadNextOperation",
                        daemon=True)
            t.start()

            self._logger.info("Returning result: " + str(self.result))
            yield self.result

            # Make sure we rejoin the previous operation before continuing:
            t.join()

class JobControlTemplate(object):
    """
    Class to handle initial importing of jobs.

    In theory this should never be touched by the implementer.
    """
    def __init__(self, sc, pallet, jobCollectionsModule, CycleManager):
        # Instantiate Logger:
        self._logger = sLogger("JobControl")
        self._logger.debug("Initialized.")

        # Subsystems:
        self.sc = sc
        self.pallet = pallet

        # Define constants:
        self.jobCollectionsModule = jobCollectionsModule
        self.jobClassName = "Job"

        # Define class variables
        self.keepRunning = threading.Event()   # Event to trigger continuous cycling on / off
        self.cyclerThreadHandle = None
        self.Jobs = {}

        # Load all defined jobs.
        self.loadJobs()

        # Load all defined pallets and add them to the pallets class
        self.loadPallets()

        # Instantiate continuous cycling:
        self.cycler = CycleManager(self.pallet, self.keepRunning)

    ####################### API PASSTHROUGHS ##################################

    def startContinuous(self):
        """
        Start continuous cycling of the robot.
        """
        # If we exited the other cycle without clearing the keepRunning:
        if self.keepRunning.is_set() and self.cyclerThreadHandle and self.cyclerThreadHandle.is_alive():
            self.keepRunning.clear()
            raise RuntimeError("Attempted to start continuous cycling while still running.")

        self.cyclerThreadHandle = sThread(target=self.cycler.startContinuous,
                                          name="ContinuousCycler",
                                          daemon=True)
        self.cyclerThreadHandle.start()

    def stopContinuous(self):
        """
        Stop continuous cycling of the robot.
        """
        self.cycler.stopContinuous()

        # Rejoin thread (if it was actually running in the first place.)
        if self.cyclerThreadHandle:
            self.cyclerThreadHandle.join()

    def testActuator(self, inputArguments):
        """
        Call the 'testActuator' method.
        """
        if self.keepRunning.is_set():
            raise RuntimeError("Attempted to call testActuator while running.")
        self.cycler.testActuator(inputArguments)

    def debugAction(self, inputArguments):
        """
        Call the 'debugAction' method.
        """
        self.cycler.debugAction(inputArguments)

    def setCurrentPallet(self, palletName):
        """
        Set the current pallet that we're running.
        This API method is called whenever the user selects a new pallet.
        """
        self.pallet.setCurrentPallet(palletName)

    ############################ LOADING STUFF #################################

    def loadJobs(self):
        """
        Walk through all subpackages in the jobCollections package.

        """
        self._logger.info("Loading all Jobs")

        # Empty any previously loaded jobSets
        self.Jobs = {}

        for _, pkgname, ispkg in pkgutil.walk_packages(self.jobCollectionsModule.__path__,
                                                       self.jobCollectionsModule.__name__+"."):
            # Split out the different packages and package names.
            # Ignoring the first index since that's just the top level package.
            pkgname_split = pkgname.split(".")

            # If we come across a package don't do anything (all the work is
            # done on modules)
            if ispkg and pkgname_split[-1]:
                continue

            # If we come across a module assume it's a job or jobset:
            if len(pkgname_split) != 3:
                self._logger.warning("Found a module in jobCollections with an "
                                     + "invalid path length. Ignoring " + str(pkgname_split[-1]))
                continue

            # It's a jobSet (i.e. subpackage)
            instance = self.loadOneJob(".".join(pkgname_split))

            # Append the instance:
            if not instance:
                # If we didn't import anything assume it's just an extra file
                # and ignore it.
                continue
            else:
                if instance.name in self.Jobs:
                    self.jobControlError(
                        "Multiple jobs found with the same name : " + \
                        str(instance.name) + ". This behavior is not supported.")
                # Finally if we satisfy all of the above, append the job:
                self.Jobs[instance.name] = instance

        self._logger.info("Finished loading all Jobs and JobSets")

    def loadOneJob(self, packageName):
        """
        Try to import the module; if it fails we assume it's some other
        python file in there and don't error out. @TODO decide if we
        should actually error out here...
        """

        moduleName = self.jobClassName

        self._logger.info("Loading " + str(moduleName) + " from " + str(packageName))

        try:
            # Import the actual JobSet class and instantiate it here:
            mod_instance = importlib.import_module(packageName)
            instance = getattr(mod_instance, moduleName)(self.sc)
            return instance
        except ImportError as e:
            msg = "Failed to import module. Got an error : " + str(e)
            self._logger.warning(msg)
            print(msg)

        return None

    def loadPallets(self):
        """
        Go through each loaded job and autogenerate the pallet class.

        This function should only be called on initialization. It basically goes
        through all loaded jobs and adds their pallet(s) to the global Pallet
        class.
        """

        # Instantiate a default dictionary to add the job and pallet names:
        tempPalletNames = defaultdict(list)

        # Cycle through all loaded jobs and check what pallet(s) they belong to.
        for jobName, jobInstance in self.Jobs.items():
            # Check the jobs defined pallets. Can either be a string or a list:
            jobPallets = jobInstance.palletNames
            jobPallets = jobPallets if isinstance(jobPallets, list) else [jobPallets]

            # Go through pallets and define
            for jobPallet in jobPallets:
                tempPalletNames[jobPallet].append(jobName)

        # Add pallets to the pallet class:
        tempPallets = {}
        for palletName, jobNames in tempPalletNames.items():
            # @TODO auto define extra keys:
            tempPallets[palletName] = jobNames

        # Add all the jobs / pallets into the pallet class:
        self.pallet.jobs = self.Jobs
        self.pallet.pallets = tempPallets

    ############################ Miscellaneous ################################

    def jobControlError(self, message):
        """
        Helper method to call logger and raise an error.
        @TODO REMOVE
        """
        self._logger.error(message)
        raise RuntimeError(message)
