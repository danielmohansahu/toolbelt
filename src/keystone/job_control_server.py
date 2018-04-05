#! /usr/bin/env python3
"""@package docstring
 # file jcs.py
 # copyright SoftWear Automation, Inc.
 # author Daniel Henderson
 # date February, 2018
"""

import os
import rospy
import logging
import xml.etree.ElementTree as ET

# Import messages and services:
from std_srvs.srv import Trigger
from geometry_msgs.msg import Pose
from std_msgs.msg import String

from sew_bot_common.srv import SetBool
from sew_bot_common.srv import SetDoubles
from sew_bot_common.srv import GetStrings
from sew_bot_common.srv import SetStrings
from sew_bot_touchscreen.srv import GuiOptionRequest
from sew_bot_touchscreen.srv import GuiMetricsRequest

from sew_bot_touchscreen.msg import JobControlStatus
from sew_bot_touchscreen.msg import JobControlLock
from sew_bot_controller.msg import ImageTopics

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
    def __init__(self, jc, sc, translationXML=None):
        self._logger = logging.getLogger("JobControlServer")
        self.jc = jc
        self.sc = sc

        # Touchscreen translationMap
        self.touchscreenMap = {}
        self._initTouchscreenMap(translationXML)

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

        self._logger.info("Initialized.")

    @RosServiceDecorator(SetStrings)
    def resetOptionService(self, req):
        optionName = req.strings[0]
        print("Reseting Option: " + optionName)

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
        print("getJobInfoService Called")
        return [["not set up yet"]]

    @RosServiceDecorator(SetStrings)
    def resetJobSetService(self, req):
        self.jc.getJobSet(req.strings[0]).resetJobSet()

    @RosServiceDecorator(SetDoubles)
    def setLotNumberService(self, req):
        print("setLotNumberService Called") #INSERT STUFF HERE

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
        print("startSingleJobService Called") #INSERT STUFF HERE

    @RosServiceDecorator(Trigger)
    def stopSingleJobService(self, req):
        print("stopSingleJobService Called") #INSERT STUFF HERE

    @RosServiceDecorator(Trigger)
    def robotHomeService(self, req):
        print("robotHomeService Called")
        self.lockEmit(True, "Currently Homing")

        #self.sc.orangeLightOff()
        #self.sc.greenLightOff()

        # Jobset home, jobcontrol home
        #self.sc.platformDown()
        #self.sc.destacker.needleRelease()
        #self.sc.destacker.needleGripperUp()
        #self.sc.restacker.needleRelease()
        #self.sc.restacker.needleGripperUp()
        #self.sc.ultraSonicWelderDown()
        #self.sc.destacker.home()
        #self.sc.restacker.home()
        print('Debug 1:robotHomeService')
        self.sc.rb.enable()

        ## Collision avoidance
        if self.sc.destacker.destackerRailPort.readState() != 'Retract':
            self.sc.destacker.clampRetract()
        if self.sc.destacker.destackerTrayPort.readState() != 'Retract':
            self.sc.destacker.trayRetract()
        time.sleep(3)

        self.sc.rb.home()
        self.sc.sm.setSewScale(0, 1.0)
        self.sc.sm.setSewScaleIndex(0)
        self.sc.dogDown()
        self.sc.rb.needleUp()
        self.sc.moveSafe()
        self.sc.bg()
        self._updatePosition()
        self.lockEmit(False)

    @RosServiceDecorator(SetDoubles)
    def robotMoveByService(self, req):
        self.sc.rb.moveBy(req.values[0], req.values[1], req.values[2])
        self._updatePosition()

    @RosServiceDecorator(SetBool)
    def robotEESetService(self, req):
        if not req.value:
            self.sc.eeUp()
        else:
            self.sc.eeDown()

    @RosServiceDecorator(SetBool)
    def robotTensionSetService(self, req):
        if req.value:
            self.sc.tensionOn()
        else:
            self.sc.tensionOff()

    @RosServiceDecorator(SetBool)
    def robotDogSetService(self, req):
        if req.value:
            self.sc.dogUp()
        else:
            self.sc.dogDown()

    @RosServiceDecorator(SetDoubles)
    def robotSewingMachineSpeedSetService(self, req):
        print("robotSewingMachineSpeedSetService Called")
        self.sc.setVelocity(0.0, 0.0, 0.0, req.values[0])

    @RosServiceDecorator(Trigger)
    def robotNeedleHomeService(self, req):
        print("robotNeedleHomeService Called")
        self.sc.rb.needleUp()

    @RosServiceDecorator(SetStrings)
    def setOptionService(self, req):
        print("setOptionService Called")
        self.jc.setXmlOption(req.optional, req.strings[0])

    @RosServiceDecorator(GuiOptionRequest)
    def getOptionsService(self, req):
        # print("getOptionService Called"
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
            # print(dataItem
            # print(data[dataItem]
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
        print("getMetricsService Called")
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
            # print(dataItem
            # print(data[dataItem]

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
        print("resetOdomService Called")
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
        print("debugActionService Called")
        print(req.strings[1])

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
        print("signalEmit Called: " + signal)
        msg = String()
        msg.data = signal

        self.publishers['signalEmit'].publish(msg)

    def lockEmit(self, value, message=None, error=False, warning=False):
        assert not (error and warning), "Both warning and error can not be set at the same time"

        print("Lock Emit Called: Value: " + str(value) + " Message: \"" + str(message) + "\"")

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

    def translateDB(self, stringToTranslate):
        """
        Translate string based on touchscreen map
        """
        if stringToTranslate not in self.touchscreenMap:
            self._logger.warning("No translation found for: " + str(stringToTranslate))
            return stringToTranslate
        else:
            self._logger.info("Translating: " + str(stringToTranslate) + " to " + str(self.touchscreenMap[stringToTranslate]))
            return self.touchscreenMap[stringToTranslate]

    ################################# Protected ################################
    def _updatePosition(self):
        position = self.sc.rb.getPosition()

        poseOut = Pose()
        poseOut.position.x = position[0]
        poseOut.position.y = position[1]
        poseOut.orientation.z = position[2]

        self.publishers['poseEmit'].publish(poseOut)

    def _initTouchscreenMap(self, xmlfile):
        """
        Initialize touchscreen map; this is used to translate touchscreen methods
        """
        # Check that file is specified
        if not xmlfile:
            self._logger.warning("No touchscreen xml file specified; not creating touchscreen map.")
        elif not os.path.isfile(xmlfile):
            raise RuntimeError("Specified xml file does not exist!")
        else:
            self._logger.info("Initializing touchscreen map.")
            tsMap = ET.parse(xmlfile).find("touchscreenMap")
            for line in tsMap:
                self.touchscreenMap[line.attrib['key']] = line.attrib['value']
            self._logger.info("touchscreenMap initialized.")
