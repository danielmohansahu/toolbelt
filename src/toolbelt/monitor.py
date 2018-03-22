#! /usr/bin/env python3
"""@package docstring
 # file monitor.py
 # copyright SoftWear Automation, Inc.
 # author Daniel Sahu <d.sahu@softwearinc.com>
 # date February, 2018
"""

import time
import copy

# Try to import from relative path; if we're calling as main import
if __package__:
    from .misc import SoftwearLogger as sLogger
    from .misc import SoftwearThread as sThread
else:
    from misc import SoftwearLogger as sLogger
    from misc import SoftwearThread as sThread


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
            'latch' : True }
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
        self._logger = sLogger('SystemMonitor')
        self._logger.debug("Initialization.")

        # Topics:
        self.topics = self._checkTopics(topics)

        # Class variables:
        self.frequency = 10     # Hz
        self.active = False
        self._t = None

        # Check and warn if we've already instantiated a watcher
        if self.instances:
            self._logger.warning("Multiple SystemMonitors active. Undefined behavior may ensue.")
            print("WARNING: Multiple SystemMonitors active. Undefined behavior may ensue.")
        self.instances.append(self)

    def isActive(self):
        """
        Returns a boolean indicating whether or not the
        """
        return copy.copy(self.active)

    def setFrequency(self, desired_frequency):
        """
        Change the frequency at which the SystemMonitor checks.
        """
        self.frequency = desired_frequency

    def start(self):
        """
        Begin continuous monitoring of input topics.
        """
        self.active = True
        self._t = sThread(target=self._checkThread, daemon=True)
        self._t.start()

    def stop(self):
        """
        End continuous monitoring.
        """
        self.active = False
        if self._t and self._t.is_alive():
            self._t.join()

        # Unlatch for next start
        for methods in self.topics.items():
            methods["latched"] = False

    ########################## PRIVATE METHODS ################################

    def _checkThread(self):
        """
        Internal Method to check all input topics
        """
        while self.active:
            for topic, methods in self.topics.items():
                # Check if what we're monitoring has occurred:
                if methods['check']():
                    # If we're unlatched or always triggering, trigger method:
                    if (not methods["latch"]) or (not methods["latched"]):
                        print(str(topic) + " detected in System Monitor.")
                        methods['trigger']()
                    methods["latched"] = True
                else:
                    methods["latched"] = False
            time.sleep(1.0/self.frequency)

    def _checkTopics(self, input_topics):
        """
        Internal method to make sure we've gotten the correct inputs
        """
        def raise_failure():
            self._logger.error("Invalid input to SystemMonitor. Please see doc.")
            raise ValueError("Invalid input to SystemMonitor. Please see doc.")

        if not isinstance(input_topics, dict):
            raise_failure()
        for methods in input_topics.values():
            if ("check" not in methods) or ("trigger" not in methods):
                raise_failure()
            if (not callable(methods["check"])) or (not callable(methods["trigger"])):
                raise_failure()

            # Else populate default arguments:
            if "latch" not in methods:
                methods["latch"] = True
            methods["latched"] = False

        return input_topics
