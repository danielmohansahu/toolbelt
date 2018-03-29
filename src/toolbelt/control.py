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
import rospy
from collections import defaultdict

# Try to import from relative path; if we're calling as main import
if __package__:
    from .misc import SoftwearLogger as sLogger
    from .misc import SoftwearThread as sThread
else:
    from misc import SoftwearLogger as sLogger
    from misc import SoftwearThread as sThread

############################ JOBCONTROL STUFF ##################################

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

######################### JOBCONTROLSERVER STUFF ###############################

class TopicListener(rospy.SubscribeListener):
    def __init__(self, callback):
        self._callback = callback
    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        msg = self._callback()
        peer_publish(msg)

class RosServiceDecorator:
    def __init__(self, serviceClass):
        self.serviceClass = serviceClass
        self.responseLength = len(serviceClass._response_class.__slots__) - 2
        self.appendHeader = "header" in serviceClass._response_class.__slots__
        if self.appendHeader:
            self.responseLength -= 1

    def __call__(self, func):
        def wrap(*args, **kwargs):
            result = None
            try:
                rv = func(*args, **kwargs)
                result = [True, ""]  # [success, return message]
                if rv:
                    result.extend(rv)
            except Exception as exp:
                result = [False, str(exp)]  # [success, return message]
                result.extend([[] for i in range(self.responseLength)])
            if self.appendHeader:
                result.append(None)
            return result
        return wrap

class JobControlServer:
    def __init__(self, jc):
        self.jc = jc

        serviceList = [
            ['getJobSetsService',      GetStrings],
            ['getJobSetJobs',          GetStrings],
            ['getSetlessJobsService',  GetStrings],
            ['getJobSetInfoService',   GetStrings],
            ['getJobInfoService',      GetStrings],
            ['getSaveSlotsService',    GetStrings],
            ['saveSlotStatusService',  GetStrings],
            ['getJobSetService',       GetStrings],
            ['resetJobSetService',     SetStrings],
            ['startJobSetService',     SetStrings],
            ['setJobSetService',       SetStrings],
            ['startSingleJobService',  SetStrings],
            ['debugActionService',     SetStrings],
            ['resetOptionService',     SetStrings],
            ['setOptionService',       SetStrings],
            ['resetOdomService',       SetStrings],
            ['setActuatorService',     SetStrings],
            ['loadOptionService',      SetStrings],
            ['saveOptionService',      SetStrings],
            ['stopSingleJobService',   Trigger],
            ['robotHomeService',       Trigger],
            ['stopJobSetService',      Trigger],
            ['robotNeedleHomeService', Trigger],
            ['robotMoveByService',     SetDoubles],
            ['setLotNumberService',    SetDoubles],
            ['robotSewingMachineSpeedSetService', SetDoubles],
            ['robotEESetService',      SetBool],
            ['robotTensionSetService', SetBool],
            ['robotDogSetService',     SetBool],
            ['getOptionsService',      GuiOptionRequest],
            ['getMetricsService',      GuiMetricsRequest]
        ]

        publisherList = [
            ['poseEmit',          Pose],
            ['signalEmit',        String],
            ['updateOptionValue', String],
            ['lockEmit',          JobControlLock],
            ['statusEmit',        JobControlStatus],
            ['displayPopup',      JobControlStatus],
            ['jobFinished',       JobControlStatus]
        ]

        self.services = {}
        for serviceName, serviceClass in serviceList:
            self.services[serviceName] = rospy.Service(
                '/jobControl/' + serviceName,
                serviceClass,
                getattr(self, serviceName))

        self.publishers = {}
        for publisherName, publisherClass in publisherList:
            self.publishers[publisherName] = rospy.Publisher(
                '/jobControl/' + publisherName,
                publisherClass,
                queue_size=10)

        self.publishers['imageTopics'] = rospy.Publisher(
            '/images/imageTopics',
            ImageTopics,
            queue_size=10,
            subscriber_listener=TopicListener(self.getImageTopics))
        self.publishers['imageTopics'].publish(self.getImageTopics())

        self.subscribers = {}
        self.subscribers['positionSubscriber'] = rospy.Subscriber(
            "/sewbot/controller/stepper/LowryStepper/pose",
            Pose,
            self.positionCallback)

    @RosServiceDecorator(SetStrings)
    def resetOptionService(self, req):
        optionName = req.strings[0]
        print "Reseting Option: " + optionName

        if optionName == 'PillowsLeft':
            name = self.jc.getCurrentJobSet() ## string name of job
            name2 = name.split('X')
            length = name2[1]
            if length == '36':
                value = self.jc.getXmlData()['LargePillows']['value']
            elif length == '26':
                value = self.jc.getXmlData()['SmallPillows']['value']
            else:
                value = self.jc.getXmlData()['MediumPillows']['value']

        if optionName == 'Side':
            value = 'Default'

        self.jc.setXmlOption(optionName, value)
        self.updateOptionValue(optionName, value)

    @RosServiceDecorator(GetStrings)
    def getJobSetsService(self, req):
        result = self.jc.getJobSetNames()
        return [result]

    # this function no longer needed
    @RosServiceDecorator(GetStrings)
    def getSetlessJobsService(self, req):
        result = self.jc.getSetlessJobs()
        return [[result]]

    @RosServiceDecorator(GetStrings)
    def getJobSetJobs(self, req):
        jobSet = self.jc.getJobSet(req.optional)
        result = jobSet.getJobNames()
        return [result]

    @RosServiceDecorator(GetStrings)
    def getJobSetInfoService(self, req):
        jobSet = self.jc.getJobSet(req.optional)
        result = jobSet.getInfo()
        return [[result]]

    @RosServiceDecorator(GetStrings)
    def getJobInfoService(self, req):
        print "getJobInfoService Called"
        return [["not set up yet"]]

    @RosServiceDecorator(SetStrings)
    def resetJobSetService(self, req):
        self.jc.getJobSet(req.strings[0]).resetJobSet()

    @RosServiceDecorator(SetDoubles)
    def setLotNumberService(self, req):
        print "setLotNumberService Called" #INSERT STUFF HERE

    @RosServiceDecorator(GetStrings)
    def getJobSetService(self, req):
        result = self.jc.getCurrentJobSet()
        return [[result]]

    @RosServiceDecorator(SetStrings)
    def setJobSetService(self, req):
        self.jc.setCurrentJobSet(req.strings[0])

    @RosServiceDecorator(SetStrings)
    def startJobSetService(self, req):
        self.currentlyRunningJobSet = self.jc.getJobSet(req.strings[0])
        self.currentlyRunningJobSet.startContinuous()

    @RosServiceDecorator(Trigger)
    def stopJobSetService(self, req):
        if self.currentlyRunningJobSet != None:
            self.currentlyRunningJobSet.stopContinuous()

    @RosServiceDecorator(SetStrings)
    def startSingleJobService(self, req):
        print "startSingleJobService Called" #INSERT STUFF HERE

    @RosServiceDecorator(Trigger)
    def stopSingleJobService(self, req):
        print "stopSingleJobService Called" #INSERT STUFF HERE

    @RosServiceDecorator(Trigger)
    def robotHomeService(self, req):
        print "robotHomeService Called"
        self.lockEmit(True, "Currently Homing")

        #sc.orangeLightOff()
        #sc.greenLightOff()

        # Jobset home, jobcontrol home
        #sc.platformDown()
        #sc.destacker.needleRelease()
        #sc.destacker.needleGripperUp()
        #sc.restacker.needleRelease()
        #sc.restacker.needleGripperUp()
        #sc.ultraSonicWelderDown()
        #sc.destacker.home()
        #sc.restacker.home()
        print('Debug 1:robotHomeService')
        sc.rb.enable()

        ## Collision avoidance
        if sc.destacker.destackerRailPort.readState() != 'Retract':
            sc.destacker.clampRetract()
        if sc.destacker.destackerTrayPort.readState() != 'Retract':
            sc.destacker.trayRetract()
        time.sleep(3)

        print('Debug 2:robotHomeService')
        sc.rb.home()
        print('Debug 3:robotHomeService')
        sc.sm.setSewScale(0, 1.0)
        print('Debug 4:robotHomeService')
        sc.sm.setSewScaleIndex(0)
        print('Debug 5:robotHomeService')
        sc.dogDown()
        print('Debug 6:robotHomeService')
        sc.rb.needleUp()
        print("Homing the needle before start")
        sc.moveSafe()
        print('Debug 7:robotHomeService')
        sc.bg()
        # print('Debug 8:robotHomeService')
        print('Debug 9:robotHomeService')
        self._updatePosition()
        print('Debug 10:robotHomeService')
        self.lockEmit(False)

    @RosServiceDecorator(SetDoubles)
    def robotMoveByService(self, req):
        sc.rb.moveBy(req.values[0], req.values[1], req.values[2])
        self._updatePosition()

    @RosServiceDecorator(SetBool)
    def robotEESetService(self, req):
        if not req.value:
            sc.eeUp()
        else:
            sc.eeDown()

    @RosServiceDecorator(SetBool)
    def robotTensionSetService(self, req):
        if req.value:
            sc.tensionOn()
        else:
            sc.tensionOff()

    @RosServiceDecorator(SetBool)
    def robotDogSetService(self, req):
        if req.value:
            sc.dogUp()
        else:
            sc.dogDown()

    @RosServiceDecorator(SetDoubles)
    def robotSewingMachineSpeedSetService(self, req):
        print "robotSewingMachineSpeedSetService Called"
        sc.setVelocity(0.0, 0.0, 0.0, req.values[0])

    @RosServiceDecorator(Trigger)
    def robotNeedleHomeService(self, req):
        print "robotNeedleHomeService Called"
        sc.rb.needleUp()

    @RosServiceDecorator(SetStrings)
    def setOptionService(self, req):
        print "setOptionService Called"
        self.jc.setXmlOption(req.optional, req.strings[0])

    @RosServiceDecorator(GuiOptionRequest)
    def getOptionsService(self, req):
        # print "getOptionService Called"
        data = self.jc.getXmlData()
        keys = []
        types = []
        descriptions = []
        defaults = []
        currents = []
        previous = []
        minValue = []
        maxValue = []
        choices = []

        for dataItem in data:
            # print dataItem
            # print data[dataItem]
            keys.append(dataItem)
            types.append(data[dataItem]['type'])
            descriptions.append(data[dataItem]['description'])
            defaults.append(str(data[dataItem]['defaultValue']))
            currents.append(str(data[dataItem]['value']))
            previous.append(str(data[dataItem]['previousValue']))
            minValue.append(str(data[dataItem]['minValue']))
            maxValue.append(str(data[dataItem]['maxValue']))
            choices.append(data[dataItem]['choices'])

        return [keys, types, descriptions, defaults,
                currents, previous, minValue, maxValue, choices]

    @RosServiceDecorator(SetStrings)
    def loadOptionService(self, req):
        self.jc.loadXmlOptions(req.strings[0])

    @RosServiceDecorator(SetStrings)
    def saveOptionService(self, req):
        self.jc.saveXmlOptions(req.strings[0])

    @RosServiceDecorator(GetStrings)
    def getSaveSlotsService(self, req):
        strings = self.jc.getSaveSlots()
        return [strings]

    @RosServiceDecorator(GetStrings)
    def saveSlotStatusService(self, req):
        strings = self.jc.getSaveSlotsStatus()
        return [strings]

    @RosServiceDecorator(GuiMetricsRequest)
    def getMetricsService(self, req):
        print "getMetricsService Called"
        data = self.jc.getMetricsData()
        days = []
        hours = []
        seconds = []
        keys = []
        attempts = []
        finished = []
        interruptions = []
        linearfeet = []

        for dataItem in data:
            # print dataItem
            # print data[dataItem]

            keys.append(dataItem)

            timeDiff = (datetime.datetime.today() - data[dataItem]['date'])
            days.append(int(timeDiff.days))
            seconds.append(int(timeDiff.seconds))

            attempts.append(int(float(data[dataItem]['attempts'])))
            finished.append(int(float(data[dataItem]['finished'])))
            interruptions.append(int(float(data[dataItem]['interruptions'])))
            linearfeet.append(float(data[dataItem]['linearfeet']))

        return [days, seconds, keys, attempts,
                finished, interruptions, linearfeet]

    @RosServiceDecorator(SetStrings)
    def resetOdomService(self, req):
        print "resetOdomService Called"
        if req.strings[0] == 'Odometer1':
            self.jc.resetOdom1()
        else:
            self.jc.resetOdom2()

    @RosServiceDecorator(SetStrings)
    def setActuatorService(self, req):
        print("setActuatorKeysService Called on: ", req.strings[0])
        self.jc.testActuator(req.strings[0])

    @RosServiceDecorator(SetStrings)
    def debugActionService(self, req):
        print "debugActionService Called"
        print req.strings[1]

        self.currentlyRunningJobSet = self.jc.getJobSet(req.strings[0])
        self.currentlyRunningJobSet.debugAction(req.strings[1])
        self._updatePosition()

    def getImageTopics(self):
        topics = [('hello', 'bye'), ('person', 'you'), ('beer', 'still'),
            ('lost', 'show'), ('bad', 'good'), ('heey', 'boo'), ('house', 'more')]
        msg = ImageTopics()
        for item in topics:
            msg.name.append(item[0])
            msg.topic.append(item[1])
        return msg

    def updateOptionValue(self, name, value):
        msg = String()
        msg.data = name + '.' + str(value)
        self.publishers['updateOptionValue'].publish(msg)

    def statusEmit(self, message, error=False, warning=False):
        assert not (error and warning), "Both warning and error can not be set at the same time"
        msg = JobControlStatus()
        msg.type = msg.INFO
        if error:
            msg.type = msg.ERROR
        if warning:
            msg.type = msg.WARNING

        msg.message = message
        self.publishers['statusEmit'].publish(msg)

    def displayPopup(self, message, error=False, warning=False):
        assert not (error and warning), "Both warning and error can not be set at the same time"
        msg = JobControlStatus()
        msg.type = msg.INFO
        if error:
            msg.type = msg.ERROR
        if warning:
            msg.type = msg.WARNING

        msg.message = message
        self.publishers['displayPopup'].publish(msg)

    def signalEmit(self, signal):
        print "signalEmit Called: " + signal
        msg = String()
        msg.data = signal

        self.publishers['signalEmit'].publish(msg)

    def lockEmit(self, value, message=None, error=False, warning=False):
        assert not (error and warning), "Both warning and error can not be set at the same time"

        print "Lock Emit Called: Value: " + str(value) + " Message: \"" + str(message) + "\""

        msg = JobControlLock()
        msg.lockValue = value

        msg.type = msg.INFO
        if error:
            msg.type = msg.ERROR
        if warning:
            msg.type = msg.WARNING

        if message != None:
            msg.message = message
        else:
            msg.message = ""
        self.publishers['lockEmit'].publish(msg)

    def jobFinished(self, message="", error=False, warning=False):
        assert not (error and warning), "Both warning and error can not be set at the same time"
        msg = JobControlStatus()
        msg.type = msg.INFO
        if error:
            msg.type = msg.ERROR
        if warning:
            msg.type = msg.WARNING

        msg.message = message
        self.publishers['jobFinished'].publish(msg)

    def positionCallback(self,data):
        self.publishers['poseEmit'].publish(data)

    def _updatePosition(self):
        position = sc.rb.getPosition()

        poseOut = Pose()
        poseOut.position.x = position[0]
        poseOut.position.y = position[1]
        poseOut.orientation.z = position[2]

        self.publishers['poseEmit'].publish(poseOut)
