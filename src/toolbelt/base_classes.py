#! /usr/bin/env python3
"""@package docstring
 # file base_classes.py
 # copyright SoftWear Automation, Inc.
 # author Daniel Sahu <d.sahu@softwearinc.com>
 # date February, 2018
"""

import threading
import time
import copy

# Try to import from relative path; if we're calling as main import
if __package__:
    from .misc import SoftwearThread as sThread
    from .misc import SoftwearLogger as sLogger
    from .decorators import LoggingDecorator
else:
    from misc import SoftwearThread as sThread
    from misc import SoftwearLogger as sLogger
    from decorators import LoggingDecorator

# Default States defined for Actuator below:
class DefaultStateDict(object):
    """
    Class containing the default state initialization format for BaseActuator
    class.
    """
    def __init__(self):
        self.requiredFormat = {
            'sense': {'default': None, 'required': False},
            'actuate': {'default': None, 'required': True},
            'home_state': {'default': False, 'required': False},
            'previous_states' : {'default': [], 'required': False}
        }

    def get_required(self):
        """
        Return required format.
        """
        return self.requiredFormat

    def get_default(self):
        """
        Return default format.
        """
        result = {}
        for item, val in self.requiredFormat.items():
            result[item] = val['default']

# A class that creates base functions for an actuator (including sensors)
class BaseActuator(object):
    """
    This class is intended to be used to command all softwear actuators.
    The purpose is to force orthogonality between the Softwear IO API and the
    python JobControlServer. By exclusively using the actuator class for all
    calls to the IO API code refactoring in the future will not break JobControl.

    Each BaseActuator class instantiation takes a unique name and a dictionary
    outlining the sensing and actuation methods for each state (the sensing
    method is optional). The scope of this class does not extend to continuous
    state actuators. Some methods (such as Actuator().toggle()) are defined
    only for dual state systems.

    Initialization is called via the following:
    input = {
        'state1' : {
            'sense' : sensorport.readValue,
            'actuate' : lambda : state1port.writeState('state1'),
        }
        'state2' : {
            'actuate' : lambda : state2port.writeState('state2'),
            'home_state' : True,
            'previous_states' : ['state1']
        }
    }
    act = BaseActuator(input, name="test_actuator")

    Note that the above details a number of significant facts about the input.
    Input of sensor ports is not required. The 'home_state' boolean is also
    optional, although at least on given state must specify this as True.

    ### Methods:
     - go_to_state      : This method commands the actuator to go to the
                            specified state. The specified state must be one
                            that was specified during initialization. The
                            optional argument "blocking" (default: True) can be
                            set to False to spin off a movement thread and
                            return control to caller immediately. A callable
                            "start_condition" method resulting in a boolean can
                            also be specified, in which case the system will wait
                            until that condition returns True or the settable
                            safety_timeout is exceeded.
    - at_current_state  : Determines whether or not the actuator expected state
                            matches its sensor. This method is used to determine
                            whether or not the actuator has drifted or been
                            forced into an incorrect state, and therefore
                            only returns False if there's a discrepancy between
                            sensor and internal state. True is returned even if
                            we are moving or unsure of our state (i.e.
                            current_state is None ) or if there is no sensor
                            for the current_state.
    - is_moving         : Returns True if the actuator has been commanded to move
                            with blocking=False and has not returned yet. Returns
                            False otherwise.
    - home              : A wrapper around the "go_to_state" method that commands
                            a move to the actuator home state. "blocking" and
                            "start_condition" are still optional arguments.
    - toggle            : Only defined for dual state systems. This method moves
                            from the current state to the other state. If the
                            system is unsure of its state it goes to the
                            non-home state by default.
    - get_default_input : This static method returns a dictionary of the correct
                            format to use as a state input.
    - all_states_go     : This class method returns True if there are no
                            discrepencies between sensors and current states for
                            any instantiated actuator. It calls "at_current_state"
                            on every instance of the actuator class.

    ### Configuration:
    A number of settable parameters are available to the user. These include:
     - 'safety_timeout' : Time (in seconds) to wait for safety condition to be
                            satisfied before raising an exception. If no safety
                            condition is specified this field is not used.
                            Default is 30.
     - 'move_timeout'   : Time (in seconds) to wait for a sensor to trigger at
                            the end of a move before raising an exception. If
                            no sensor is specified this field is not used.
                            Default is 30.

    There are also a larger number of gettable parameters. These include:
     - 'safety_timeout' : Same as specified in settable parameters.
     - 'move_timeout'   : Same as specified in settable parameters.
     - 'name'           : Instance name (specified by user)
     - 'current_state'  : Current state the actuator is in. If unknown the
                            return value is None.
     - 'home_state'     : Returns the home state of the actuator instance.
     - 'states'         : Retruns a list of strings indicating the possible
                            states this actuator can exist in.

    """

    # List of currently instantiated Actuators.
    # Used to prevent duplicates and to check sensor status of all actuators
    instances = []

    def __init__(self, name):
        # Logging info:
        self._logger = sLogger('ACTUATOR_' + name)
        self._logger.debug("Initialization.")

        # Lists delineating settable and gettable attribute names.
        self._settable = ['safety_timeout',
                          'move_timeout',
                          'home_first']

        self._gettable = ['safety_timeout',
                          'move_timeout',
                          'name',
                          'states',
                          'home_state',
                          'current_state',
                          'home_first',
                          'initialized']

        # Miscellaneous variables
        self.name = name            # Actuator name
        self.initialized = False    # indicate if system initialized
        self.safety_timeout = 30    # Timeout waiting for safety condition
        self.move_timeout = 30      # Timeout waiting for move
        self.home_state = None      # Home state (specified)
        self.current_state = None   # Current state (determined)
        self.home_first = True      # Specify if we must home if state unknown
        self.states = []            # List of states
        self._t = None              # Thread handle for non-blocking calls.
        self._S = {}                # Dictionary of states (populated)
        self.input_states = {}      # Input states given by the user.

        # Locks to prevent simultaneous movement and state changes
        self._current_state_lock = threading.Lock()
        self._movement_lock = threading.Lock()

        # Increment Instances class variable
        # Doing this at the end so that it only happens if we don't error out before.
        if self.name not in [item.name for item in self.instances]:
            self.instances.append(self)
        else:
            self._logger.error("Attempted to create a duplicate actuator with the same name.")
            raise RuntimeError("Attempted to create a duplicate actuator with the same name.")

    def __setitem__(self, name, value):
        # Settable parameters:
        if name in self._settable:
            # Make sure we're casting it as the correct type:
            try:
                value_type = type(getattr(self, name))
                value = value_type(value)
            except ValueError as e:
                print("Unable to set parameter; type input is wrong.")
                self._logger.error('Unable to set parameter: ' + str(name) + " to " + str(value))
                raise ValueError(e)

            self._logger.info('Set parameter: ' + str(name) + " to " + str(value))
            setattr(self, name, float(value))
        else:
            self._logger.error("Unsettable parameter specified in __setitem__")
            raise ValueError("Unsettable parameter specified in __setitem__")

    def __getitem__(self, name):
        # Gettable parameters
        if name in self._gettable:
            # Deliberately not returning item itself; prevent accidental
            # modification of self._states, etc. Irrelevant for immutables.
            return copy.copy(getattr(self, name))
        else:
            self._logger.info('Requested parameter: ' + str(name) + " does not exist.")
            raise ValueError("Requested parameter does not exist.")

    ############################ SPECIAL METHODS ###############################

    @staticmethod
    def get_default_input():
        """
        Return a default dictionary (for use in initialization):
        """
        return DefaultStateDict().get_default()

    @classmethod
    def all_states_go(cls):
        """
        Performs a status check on all instantiated instances of the Actuator
        class. This is simply a call to "at_current_state" for all objects.

        Note: the default return value of each call to "at_current_state" is True;
        it only returns False if there is a noted discrepency between the
        internal current_state and the sensor value of that state. This is to
        ensure that we don't raise nuisance Exceptions if a sensor doesn't
        exist of the actuator is moving.
        """
        if cls.instances:
            return all([instance.at_current_state() for instance in cls.instances])
        else:
            # If we haven't created any instances yet this should still be true.
            return True

    ############################# BASIC METHODS ################################

    def add_state(self, state_name, actuate, sense=None, home_state=False, previous_states=None):
        """
        Append a new state to the class.
        """
        if not callable(actuate):
            raise RuntimeError("Actuate class needs to be a method.")
        if sense and not callable(sense):
            raise RuntimeError("Sense class needs to be a method.")
        if state_name in self.states:
            raise RuntimeError("State already exists!")

        self.input_states[state_name] = {
            'actuate' : actuate,
            'sense' : sense,
            'home_state' : home_state,
            'previous_states' : previous_states
        }
        self.states.append(state_name)

        # Update current list of states:
        self._set_states()

        # Check whether the user has entered states properly.
        if self['initialized']:
            self._logger.debug("System fully initialized, safe to move.")
        else:
            self._logger.debug("System not initialized; continue to add states.")

    def is_moving(self):
        """
        Check whether our system is still moving
        """
        if self._t and self._t.isAlive():
            return True

        # If we don't have an active thread we assume we're stationary
        # @TODO think this through. When could this assumption be false?
        return False

    @LoggingDecorator
    def wait(self):
        """
        Wait for move to finish.
        Returns immediately if the actuator isn't moving.
        """
        if not self.is_moving():
            self._logger.info("Wait returned immediately. Not moving.")
            return True
        self._t.join()

    ########################### COMPLEX METHODS ################################

    # Return a boolean indicating whether or not our sensor value matches the current state
    def at_current_state(self):
        """
        Safety check to see if we're at the expected state or not.
        """

        current_state = self['current_state']
        if not current_state:
            # We're moving or unhomed; don't know where the state is:
            # A bit funny to say things are ok, but we know we're in between states
            self._logger.debug("Unsure of current state.")
            return True

        # Sanity checks:
        sense = self._S[current_state]['sense']
        if not sense:
            self._logger.debug("Unable to check current state status: no sensor.")
            # Strange to say things are ok, but the default assumption here is
            # that we should only raise an error if we KNOW THINGS ARE WRONG
            return True

        # Check that our current state sensor reading is correct.
        # If our signal is noisy this should return false.
        success = sense()
        self._logger.debug("At expected state? : " + str(success))
        return success

    # Basic Move function:
    @LoggingDecorator
    def go_to_state(self, desired_state, blocking=True, start_condition=None,
                    input_args=(), delay=0.0):
        """
        Go to specified state. If not blocking return thread handle to join at
        a later date. Safety conditions are a list of functions to call and
        check. They should return True if it's safe to continue.

        Returns [bool, thread_handle]. If blocking=True then the thread handle
        is null
        """
        if not self['initialized']:
            raise RuntimeError("Actuator commanded to move before properly initialized.")

        current_state = self['current_state']

        # Sanity checks:
        # Check that the desired state exists.
        if desired_state not in self['states']:
            self._logger.error("Commanded to move to a non existant state.")
            raise KeyError("The specified state does not exist for this \
                            actuator. Please enter a valid state.")

        # Check that the current state isn't the desired state
        # @TODO also check sensing method if given
        if current_state == desired_state:
            print("Commanded to move to current state; no action taken.")
            self._logger.info("Commanded move to current state; no action.")
            return True

        if not current_state and desired_state != self['home_state'] and self['home_first']:
            # If we don't know our current state and we're required to home first:
            self._logger.error("This actuator must be homed before a move is commanded.")
            raise RuntimeError("This actuator must be homed before a move is commanded.")
        elif current_state and self._S[desired_state]['previous_states'] and (current_state not in self._S[desired_state]['previous_states']):
            # Check the optional "previous_state" to make sure it's safe to move
            self._logger.error("Invalid state transition: " + current_state + \
                               " to " + desired_state)
            raise RuntimeError("Invalid state transition: " + current_state + \
                               " to " + desired_state)

        # Perform safety checks: If timeout is specified wait for them to clear:
        safe_to_continue = self._check_start_condition(start_condition)
        if not safe_to_continue:
            self._logger.error("Safety condition not satisfied.")
            raise RuntimeError("Safety condition not satisfied.")

        # If not blocking spin off thread:
        if not blocking:
            self._t = sThread(target=self._go_to_state,
                              args=(desired_state, input_args, delay,),
                              name=self.name + "_go_to_state_" + desired_state,
                              daemon=True)
            self._t.start()
            return None
        else:
            return self._go_to_state(desired_state, input_args)

    @LoggingDecorator
    def home(self, blocking=True, start_condition=None, delay=0.0):
        """
        Command actuator to move to its home state.
        """
        self._logger.debug("Homing...")
        result = self.go_to_state(self['home_state'],
                                  blocking=blocking,
                                  start_condition=start_condition,
                                  delay=delay)
        return result

    @LoggingDecorator
    def toggle(self, blocking=True, start_condition=None, delay=0.0):
        """
        Go to another state. Behavior undefined for non-binary actuators.
        If state is unknown go to the opposite of home state:
        """
        if len(self.states) != 2:
            self._logger.error("Toggle undefined for more than two states.")
            return False

        # If we don't know where we are, go to the opposite of home
        # If that move is unsafe or not allowed (i.e. home first) then
        # go_to_state should handle it.
        if not self['current_state']:
            init_state = self['home_state']
        else:
            init_state = self['current_state']

        desired_state = self.states[0] if (self.states[0] != init_state) else self.states[1]

        success = self.go_to_state(
            desired_state=desired_state,
            blocking=blocking,
            start_condition=start_condition,
            delay=delay)

        return success

    ################################# PRIVATE ##################################

    def _set_states(self):
        """
        1) Go through and check all the input states to make sure they are
        properly defined.

        2) Check whether any sensors are tripped and set current states to those
        values. This is to prevent the need to home.

        3) Check if we have one and only one home state.

        """
        # Create actuation plan and sanity checks
        check_dict = DefaultStateDict().get_required()
        self._S = copy.deepcopy(self.input_states)

        # Check that input structure is correct and populate with default values.
        #@TODO try to do this a little more cleanly. Default dict stuff?
        for (state_key, state) in self._S.items():
            # Cycle through expected dict keys
            for key in check_dict:
                # Check if all keys match;
                if key not in state:
                    # if not and that key is required raise an error.
                    if check_dict[key]['required']:
                        raise ValueError("Required key: " + key + " not found \
                                         in actuator class initialization.")
                    else:
                        # Else populate with the default value:
                        state[key] = check_dict[key]['default']
                elif key == "previous_states" and not isinstance(state[key],list):
                    # Make sure this key is a list, not a string.
                    self._logger.warning("Converting state: " + state_key + \
                                         " 'previous_states' to list.")
                    state[key] = [(state[key])]

        # Try to initialize states (if there's a sensor see if it's tripped)
        current_states = [key for key in self._S if (self._S[key]['sense'] and self._S[key]['sense']())]
        if len(current_states) > 1:
            self._logger.warning("Multiple sensors triggered at once; something is wrong.")
            #raise RuntimeError("Multiple sensors triggered at once; something is wrong.")
        elif len(current_states) == 1:
            self._logger.info("Initializing current state to : " + str(current_states[0]))
            self._set_current_state(current_states[0])

        # Check that we have one and only one homing location
        home_states = [key for key in self._S if self._S[key]['home_state']]

        if len(home_states) != 1:
            self._logger.warning("Actuator must have one and only one home state. \
                                 Probably not fully initialized yet.")
            self.initialized = False
        else:
            self.home_state = home_states[0]
            self.initialized = True

    def _set_current_state(self, new_current_state):
        # Check that it's a reasonable state or None state
        if (new_current_state in self['states']) or not new_current_state:
            with self._current_state_lock:
                self.current_state = new_current_state
            self._logger.info("Set state to : " + str(new_current_state))
        else:
            self._logger.error("Tried to set current state to unreal state.")
            raise ValueError("Tried to set current state to unreal state.")

    def _go_to_state(self, desired_state, input_args=(), delay=0.0):
        # Internal go to state function; return True if successful, false if otherwise.

        # Actuator and sensore objects:
        start_time = time.time()
        success = True

        # Actually move:
        self._logger.debug("Beginning move to " + str(desired_state) + ".")
        with self._movement_lock:
            self._set_current_state(None)

            self._S[desired_state]['actuate'](*input_args)

            # Pause if desired
            sense = self._S[desired_state]['sense']

            # Loop until sensor triggers (if a sensor was defined)
            if sense:
                while not sense():
                    # If we timeout, break:
                    if time.time() - start_time >= self['move_timeout']:
                        success = False
                        break
                    # time.sleep(0.1)

            if success:
                self._logger.debug("Delaying move to " + str(desired_state) + ".")
                time.sleep(delay)

                self._logger.debug("Finished move to " + str(desired_state) + ".")
                self._set_current_state(desired_state)

        # If we timed out, raise a RuntimeError.
        # This doesn't really do anything if we're not blocking (which is maybe good??)
        if not success:
            self._logger.error("Timed out during move to " + str(desired_state) + ".")
            raise RuntimeError("Timed out waiting for sensor to trigger.")

        return success

    def _check_start_condition(self, start_condition):
        # Check safety conditions:
        safe_to_continue = True

        if not start_condition:
            return safe_to_continue

        # Initial check:
        start_time = time.time()
        if callable(start_condition):
            safe_to_continue = safe_to_continue and start_condition()
        else:
            self._logger.error("Safety conditions should be callable!")
            raise ValueError("Safety conditions should be callable!")

        # Wait for timeout seconds.
        while (time.time() - start_time <= self['safety_timeout']) and not safe_to_continue:
            safe_to_continue = safe_to_continue and start_condition()
            time.sleep(0.1)

        if time.time() - start_time > self['safety_timeout']:
            self._logger.error("Timed out waiting on safety conditions for move.")
            raise RuntimeError("Timed out waiting on safety conditions for move.")

        return safe_to_continue

################ TESTING IF CALLED DIRECTLY ###############################
if __name__ == "__main__":
    ################### UNIT TESTING ###########################

    import random
    import code
    import unittest

    # Dummy classes for testing:
    class DummySensor(object):
        """
        Dummy sensor for unit testing.
        """
        def __init__(self, state):
            self.state = state

        def readValue(self):
            """
            Return sensor value.
            """
            return random.random() > 0.2

    class DummyPort(object):
        """
        Dummy port for unit testing.
        """
        def __init__(self):
            pass

        def writeState(self, state):
            """
            Command sensor to go to state..
            """
            time.sleep(0.75)

    def make_actuator(name):
        my_port_re = DummyPort()
        my_port_ex = DummyPort()
        my_sensor_ex = DummySensor('extended')
        my_sensor_re = DummySensor('retracted')

        BA = BaseActuator(name=name)
        BA.add_state('extended',
                     actuate=lambda: my_port_re.writeState("retracted"),
                     sense=my_sensor_re.readValue,
                     home_state=True,
                     previous_states=['retracted'])
        BA.add_state('retracted',
                     actuate=lambda: my_port_ex.writeState('extended'),
                     sense=my_sensor_ex.readValue,
                     previous_states=['extended'])
        return BA

    class TestBaseActuator(unittest.TestCase):
        def setUp(self):
            pass

        def tearDown(self):
            pass

        def test_home(self):
            TEST = make_actuator("test1")
            self.assertTrue(TEST.home())

        def test_duplicate(self):
            with self.assertRaises(RuntimeError):
                # Make sure the instantiation fails.
                TEST = make_actuator(BaseActuator.instances[0]["name"])

        def test_move1(self):
            TEST = make_actuator("test2")
            TEST['home_first'] = False
            self.assertTrue(TEST.go_to_state("retracted"))

        def test_move2(self):
            TEST = make_actuator("test3")
            TEST['home_first'] = False
            self.assertTrue(TEST.go_to_state("extended"))

        def test_toggle(self):
            TEST = make_actuator("test4")
            TEST['home_first'] = False
            self.assertTrue(TEST.toggle())
            self.assertTrue(TEST.toggle())

        def test_blocking(self):
            TEST = make_actuator("test5")
            TEST['home_first'] = False
            TEST.toggle(blocking=False)
            TEST.wait()
            TEST.toggle(blocking=False)
            TEST.wait()

    # Call tests
    unittest.main()
    code.interact(local=locals())
