#! /usr/bin/env python3
"""@package docstring
 # file monitor.py
 # copyright SoftWear Automation, Inc.
 # author Daniel Sahu <d.sahu@softwearinc.com>
 # date February, 2018
"""

import time
import copy
import threading
import logging

# Try to import from relative path; if we're calling as main import
if __package__:
    from .decorators import TryExceptDecorator, LoggingDecorator
else:
    from decorators import TryExceptDecorator, LoggingDecorator

class SystemMonitor(object):
    """
    Class to aid in the creation of system watchdogs.

    topics = {
        'topic1_name' : {
            'check' : check_blah,
            'trigger' : do_blah }
        'topic2_name' : {
            'check' : check_blah,
            'trigger' : do_blah,
            'latch' : True,
            'unlatch_check' : check_blegh }
    }

    sm = SystemMonitor(topics)
    sm.start()

    # Methods:
     - start            : Start continous checking.
     - stop             : Stop continuous checking.
     - setFrequency     : Set the frequency at which checks are performed.
                            Default is 10 Hz.
     - isActive         : Returns true if the monitor is active.

    """
    instances = []
    def __init__(self, topics):
        # Logging info:
        self._logger = logging.getLogger("SystemMonitor")
        self._logger.info("Initialization.")

        # Class variables:
        self._frequency = 10     # Hz
        self._active = False
        self._t = None
        self._modify_topics_lock = threading.Lock()

        # Topics:
        self._topics = self._checkTopics(topics)

        # Check and warn if we've already instantiated a watcher
        if self.instances:
            self._logger.warning("Multiple SystemMonitors active. Undefined behavior may ensue.")
        self.instances.append(self)

    @LoggingDecorator
    def isActive(self):
        """
        Returns a boolean indicating whether or not the monitor is active.
        """
        return self._active

    def getTopics(self):
        """
        Returns a list of topic names.
        """
        return list(self._topics.keys())

    def setFrequency(self, desired_frequency):
        """
        Change the frequency at which the SystemMonitor checks.
        """
        self._frequency = desired_frequency

    def start(self):
        """
        Begin continuous monitoring of input topics.
        """
        if self._t and self._t.is_alive():
            self._logger.warning("System Monitor 'start' called when running. Not restarting; call 'stop' then 'start' again to restart.")
        else:
            self._logger.info("Starting system monitor.")
            self._active = True
            self._t = threading.Thread(target=self._checkThread, name="systemMonitorCheckThread", daemon=True)
            self._t.start()

    def stop(self):
        """
        End continuous monitoring.
        """
        self._active = False
        if self._t and self._t.is_alive():
            self._logger.info("Stopping system monitor.")
            self._t.join()
        else:
            self._logger.info("System Monitor 'stop' called when not active.")

        # Unlatch for next start
        self.unlatch()

    @LoggingDecorator
    def unlatch(self, input_key=None):
        """
        Unlatch the specified method. If no key is specified all topics are unlatched.
        """
        for key, method in self._topics.items():
            if key == input_key or not input_key:
                self._set_latch(key, False)

    ########################## PRIVATE METHODS ################################

    @TryExceptDecorator
    def _checkThread(self):
        """
        Internal Method to check all input topics
        """
        #@TODO CLEAN THIS UP. it's very unclear and potentially buggy
        while self._active:
            for topic, methods in self._topics.items():
                # Check if what we're monitoring has occurred:
                if methods['check']():
                    # If we're unlatched or always triggering, trigger method:
                    if not methods["latch"]:
                        # Trigger continuously if we don't desire a latching response:
                        self._logger.warning("Non-latching topic: " + str(topic) + " detected in System Monitor. Triggering response...")
                        methods['trigger']()
                    elif not methods["latched"]:
                        # Otherwise, trigger and then latch
                        methods['trigger']()
                        self._set_latch(topic, True)
                else:
                    # If the check criterion is not met: check if we should unlatch
                    if methods["latch"] and methods["latched"]:
                        # If both lat
                        if methods['unlatch_check']():
                            self._set_latch(topic, False)

            time.sleep(1.0/self._frequency)

    def _checkTopics(self, input_topics):
        """
        Internal method to make sure we've gotten the correct inputs
        """
        def raise_failure():
            raise ValueError("Invalid input to SystemMonitor. Please see doc.")

        if not isinstance(input_topics, dict):
            raise_failure()
        for methods in input_topics.values():
            if ("check" not in methods) or ("trigger" not in methods):
                raise_failure()
            if (not callable(methods["check"])) or (not callable(methods["trigger"])):
                raise_failure()

            # Populate default arguments:
            if "unlatch_check" not in methods:
                methods["unlatch_check"] = lambda: False

            # Populate and instantiate latch:
            with self._modify_topics_lock:
                if "latch" not in methods:
                    methods["latch"] = True
                methods["latched"] = False

        return input_topics

    def _set_latch(self, key, state):
        """
        Modify the latched state of a given topic:
        """

        # Make sure key exists:
        if not key in self._topics.keys():
            raise RuntimeError("Tried to latch/unlatch non-existent state")

        # Make sure state is correct:
        if not isinstance(state, bool):
            raise RuntimeError("State must be a boolean.")

        # Modify state:
        with self._modify_topics_lock:
            self._logger.info("Setting latched state of: " + str(key) + " to " + str(state))
            self._topics[key]['latched'] = state
