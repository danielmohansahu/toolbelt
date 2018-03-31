#! /usr/bin/env python3
"""@package docstring
 # file templates.py
 # copyright SoftWear Automation, Inc.
 # author Daniel Sahu <d.sahu@softwearinc.com>
 # date February, 2018
"""

import time

# Try to import from relative path; if we're calling as main import
if __package__:
    from .logger import SoftwearLogger as sLogger
    from .base_classes import BaseActuator
    from .decorators import TimingDecorator
else:
    from logger import SoftwearLogger as sLogger
    from base_classes import BaseActuator
    from decorators import TimingDecorator

class DestackerTemplate(object):
    """
    Template Destacker Class. This is not intended to be actually used; rather
    the implementer should learn how to properly build subsystems from this
    example. As such, much of the functionality is simplified.

    In this example the destacker consists of 4 actuators:
        - A vertical actuator to raise the product off a pallet
        - A horizontal actuator to pull the product onto a table.
        - A gripper on the horizontal stage to grip the rug.
        - A pincher on the vertical stage to pinch the rug.

    Note that we have two methods defined below (home and destack) that directly
    translate to discrete states... maybe our BaseActuator class can be defined
    to another level!
    """

    def __init__(self, api):
        """
        Initialize the destacker class by passing our C API.
        """
        self.api = api

        # Instantiate a logger to track our movements:
        self._logger = sLogger("MyDestacker")

        # Make basic actuators via the BaseActuator class. Check the individual
        # methods below to understand how we instantiate these actuators.
        self.vertical = self._build_vertical_actuator()
        self.horizontal = self._build_horizontal_actuator()
        self.gripper = self._build_gripper_actuator()
        self.pincher = self._build_pincher_actuator()

        self._logger.info("Destacker instantiated.")

    def safeToDestack(self):
        """
        A safety method to check whether or not it's ok to move the horizontal
        stage out onto the table (because that's where our robot is!)
        """
        return self.api.destacker_area_clear()

    @TimingDecorator
    def home(self):
        """
        The home method. This commands the system to move to its home state
        """
        self._logger.info("Homing...")

        self.vertical.home()
        self.horizontal.home()
        self.gripper.home()
        self.pincher.home()

    @TimingDecorator
    def destack(self):
        """
        Destack a single piece onto the table:

        This method checks that we're in our home position:
            - vertical stage up
            - horizontal stage in
            - pinchers open
            - gripper open
        """

        self._logger.info("Beginning destacking.")

        # Make sure we're starting off in the right spot;
        self.home()

        # First lower the vertical stage to grip the rug. Since the mechanical
        # team forgot to put a sensor we have to wait a little bit to make sure
        # it went down.
        self.vertical.go_to_state("down")
        time.sleep(5.0)

        # Then close the pinchers to grab and hold the rug. We also have to
        # wait here because we don't have a sensor.
        self.pinchers.go_to_state("closed")
        time.sleep(0.5)

        # Then raise up. This time we don't have to wait. Because we gave the
        # actuator class a sensing method it will block until the motion is
        # completed. If we don't want that we can call with an optional argument
        # "blocking" = False.
        self.vertical.go_to_state("up")
        # This would return immediately even though the system is still moving
        # self.vertical.go_to_state("up", blocking=False)

        # Next we grip the rug with the horizontal stage gripper. After a short
        # time we release the pinchers on the vertical stage (after we're sure
        # the handoff has occurred)
        self.gripper.go_to_state("closed")
        time.sleep(0.5)
        self.pincher.go_to_state("open")
        time.sleep(0.5)

        # Now we can drag the piece out onto the table. Luckily we have sensors
        # on both states so we don't have to add any sleeps. Make sure the area
        # is clear by passing in a safety condition. By default we will wait
        # (not indefinitely) until this condition is satisfied
        self.horizontal.go_to_state("out", safety_condition=self.safeToDestack)

        # Open the gripper to drop the rug, and then retract the horizontal stage
        # to get out of the way:
        self.gripper.go_to_state("open")
        time.sleep(0.5)
        self.horizontal.go_to_state("in")

        # And then we're done!
        self._logger.info("Finished destacking.")

    ######################### PRIVATE METHODS ##################################

    def _build_vertical_actuator(self):
        """
        Helper function to clean up legibility of constructing an actuator:

        First we define our individual state methods. As below, state1 is
        defined as a dictionary containing important keys that our BaseActuator
        class uses.
            - sense         : A method that returns True if we are at that state
                            (i.e. read a sensor)
            - actuate       : A method that, when called, moves the actuator to
                            the given state (i.e. command a pneumatic valve on)
            - home_state    : At lease one state must have this boolean defined.
                            This is the state that the actuator moves to when
                            the home method is called.
        """

        # Our up state is our home state:
        state1 = BaseActuator.get_default_input()
        state1['home_state'] = True
        state1['sense'] = self.api.destacker_is_up
        state1['actuate'] = self.api.destacker_move_up

        # Our down state has no sensor; it defaults to None:
        state2 = BaseActuator.get_default_input()
        state2['actuate'] = self.api.destacker_move_down

        input_config = {'up':state1, 'down':state2}

        return BaseActuator(input_config, name="DestackerVertical")

    def _build_horizontal_actuator(self):
        """
        Helper function to clean up legibility of constructing an actuator:

        First we define our individual state methods. As below, state1 is
        defined as a dictionary containing important keys that our BaseActuator
        class uses.
            - sense         : A method that returns True if we are at that state
                            (i.e. read a sensor)
            - actuate       : A method that, when called, moves the actuator to
                            the given state (i.e. command a pneumatic valve on)
            - home_state    : At lease one state must have this boolean defined.
                            This is the state that the actuator moves to when
                            the home method is called.
        """

        # Our up state is our home state:
        state1 = BaseActuator.get_default_input()
        state1['home_state'] = True
        state1['sense'] = self.api.destacker_is_out
        state1['actuate'] = self.api.destacker_move_out

        # Our down state has no sensor; it defaults to None:
        state2 = BaseActuator.get_default_input()
        state2['sense'] = self.api.destacker_is_in
        state2['actuate'] = self.api.destacker_move_in

        input_config = {'in':state1, 'out':state2}

        return BaseActuator(input_config, name="DestackerHorizontal")

    def _build_gripper_actuator(self):
        """
        Helper function to clean up legibility of constructing an actuator:

        First we define our individual state methods. As below, state1 is
        defined as a dictionary containing important keys that our BaseActuator
        class uses.
            - sense         : A method that returns True if we are at that state
                            (i.e. read a sensor)
            - actuate       : A method that, when called, moves the actuator to
                            the given state (i.e. command a pneumatic valve on)
            - home_state    : At lease one state must have this boolean defined.
                            This is the state that the actuator moves to when
                            the home method is called.
        """

        # Our open state is our home state:
        state1 = BaseActuator.get_default_input()
        state1['home_state'] = True
        state1['actuate'] = self.api.destacker_close_gripper

        # Our down state has no sensor; it defaults to None:
        state2 = BaseActuator.get_default_input()
        state2['actuate'] = self.api.destacker_open_gripper

        input_config = {'open':state1, 'closed':state2}

        return BaseActuator(input_config, name="DestackerGripper")

    def _build_pincher_actuator(self):
        """
        Helper function to clean up legibility of constructing an actuator:

        First we define our individual state methods. As below, state1 is
        defined as a dictionary containing important keys that our BaseActuator
        class uses.
            - sense         : A method that returns True if we are at that state
                            (i.e. read a sensor)
            - actuate       : A method that, when called, moves the actuator to
                            the given state (i.e. command a pneumatic valve on)
            - home_state    : At lease one state must have this boolean defined.
                            This is the state that the actuator moves to when
                            the home method is called.
        """

        state1 = BaseActuator.get_default_input()
        state1['home_state'] = True
        state1['actuate'] = self.api.destacker_pincher_open

        state2 = BaseActuator.get_default_input()
        state2['actuate'] = self.api.destacker_pincher_close

        input_config = {'open':state1, 'closed':state2}

        return BaseActuator(input_config, name="DestackerPincher")

class RestackerTemplate(object):
    """
    Template Restacker Class. This is not intended to be actually used; rather
    the implementer should learn how to properly build subsystems from this
    example. As such, much of the functionality is simplified.

    In this example the restacker consists of 2 actuators:
        - A horizontal actuator to push the product off a table.
        - A gripper on the horizontal stage to grip the rug.
    """
    def __init__(self, api):
        """
        Initialize the restacker class by passing our C API.
        """
        self.api = api

        # Instantiate a logger to track our movements:
        self._logger = sLogger("MyRestacker")

        # Make basic actuators via the BaseActuator class. Check the individual
        # methods below to understand how we instantiate these actuators.
        self.horizontal = self._build_horizontal_actuator()
        self.gripper = self._build_gripper_actuator()

        self._logger.info("Restacker instantiated.")

    def safeToRestack(self):
        # A safety method to check whether or not it's ok to move the horizontal
        # stage out onto the table (because that's where our robot is!)
        return self.api.restacker_area_clear()

    @TimingDecorator
    def home(self):
        """
        The home method. This commands the system to move to its home state
        """
        self._logger.info("Homing...")

        self.horizontal_stage.home()
        self.gripper_stage.home()

    @TimingDecorator
    def destack(self):
        """
        Destack a single piece onto the table:

        This method checks that we're in our home position:
            - horizontal stage in
            - gripper up
        """

        self._logger.info("Beginning restacking.")

        # Make sure we're starting off in the right spot;
        self.home()

        # First command the horizontal stage "out", i.e. onto the table so it's
        # out over the rug. This call is blocking. We also have a safety condition
        # that we'll wait to make sure is cleared before moving.
        self.horizontal.go_to_state("out", safety_condition=self.safeToRestack)

        # Then lower the gripper to grab the product. We also have to
        # wait here because we don't have a sensor.
        self.gripper.go_to_state("down")
        time.sleep(0.5)

        # Then we retract and kick the rug off the table. This motion is a
        # little complicated because we want to raise the grippers as the
        # horizontal stage is still moving. To do that we call the horizontal
        # movement method with the flag blocking=False so we don't wait for it
        # to finish.
        self.horizontal.go_to_state("in", blocking=False)
        # Wait a short time to gather momentum, then raise the grippers:
        time.sleep(2)
        self.gripper.go_to_state("Up")

        # Then wait to make sure the restacker has finished its move. This
        # method blocks until the move is complete:
        self.horizontal.wait()

        # And then we're done!
        self._logger.info("Finished restacking.")

    ######################### PRIVATE METHODS ##################################

    def _build_horizontal_actuator(self):
        """
        Helper function to clean up legibility of constructing an actuator:

        First we define our individual state methods. As below, state1 is
        defined as a dictionary containing important keys that our BaseActuator
        class uses.
            - sense         : A method that returns True if we are at that state
                            (i.e. read a sensor)
            - actuate       : A method that, when called, moves the actuator to
                            the given state (i.e. command a pneumatic valve on)
            - home_state    : At lease one state must have this boolean defined.
                            This is the state that the actuator moves to when
                            the home method is called.
        """

        # Our up state is our home state:
        state1 = BaseActuator.get_default_input()
        state1['home_state'] = True
        state1['sense'] = self.api.restacker_is_out
        state1['actuate'] = self.api.restacker_move_out

        # Our down state has no sensor; it defaults to None:
        state2 = BaseActuator.get_default_input()
        state2['sense'] = self.api.restacker_is_in
        state2['actuate'] = self.api.restacker_move_in

        input_config = {'in':state1, 'out':state2}

        return BaseActuator(input_config, name="RestackerHorizontal")

    def _build_gripper_actuator(self):
        """
        Helper function to clean up legibility of constructing an actuator:

        First we define our individual state methods. As below, state1 is
        defined as a dictionary containing important keys that our BaseActuator
        class uses.
            - sense         : A method that returns True if we are at that state
                            (i.e. read a sensor)
            - actuate       : A method that, when called, moves the actuator to
                            the given state (i.e. command a pneumatic valve on)
            - home_state    : At lease one state must have this boolean defined.
                            This is the state that the actuator moves to when
                            the home method is called.
        """

        # Our open state is our home state:
        state1 = BaseActuator.get_default_input()
        state1['home_state'] = True
        state1['actuate'] = self.api.restacker_raise_gripper

        # Our down state has no sensor; it defaults to None:
        state2 = BaseActuator.get_default_input()
        state2['actuate'] = self.api.restacker_lower_gripper

        input_config = {'up':state1, 'down':state2}

        return BaseActuator(input_config, name="RestackerGripper")

class SewingMachineTemplate(object):
    """
    This template class illustrates how to build a sewing machine subsystem. As
    with the destacker / restacker classes above it is intended as a template
    and not for actual use.

    This illustrates most of the common actuators for our sewing machines:
        - a presser foot that can be raised up or down
        - a command to home the needle:
        - a command to cut the tail via the sewing machine knife

    Note that a state approach to the sewing machine is less useful than it is
    in the destacker and restacker above. This is because we generally want
    to control the sewing machine actuators individually. In this case we use
    the BaseActuator class for consistency, but feel free to use the best tool
    for your project.

    Because of this most of the methods defined below are simply pass throughs.

    """

    def __init__(self, api):
        """
        Initialize the destacker class by passing our C API.
        """
        self.api = api

        # Instantiate a logger to track our movements:
        self._logger = sLogger("MySewingMachine")

        # Make basic actuators via the BaseActuator class. Check the individual
        # methods below to understand how we instantiate these actuators.
        self.knife = self._build_knife_actuator()
        self.foot = self._build_foot_actuator()

        self._logger.info("Sewing Machine instantiated.")

    def home(self):
        self.needleUp()
        self.knife.home()
        self.foot.home()

    def needleUp(self):
        self.api.needleUp()

    def knifeUp(self):
        self.knife.go_to_state("Up")

    def knifeDown(self):
        self.knife.go_to_state("Down")

    def footUp(self):
        self.foot.go_to_state("Up")

    def footDown(self):
        self.foot.go_to_state("Down")

    ############################## PRIVATE #####################################

    def _build_knife_actuator(self):
        """
        Helper function to clean up legibility of constructing an actuator:

        First we define our individual state methods. As below, state1 is
        defined as a dictionary containing important keys that our BaseActuator
        class uses.
            - sense         : A method that returns True if we are at that state
                            (i.e. read a sensor)
            - actuate       : A method that, when called, moves the actuator to
                            the given state (i.e. command a pneumatic valve on)
            - home_state    : At lease one state must have this boolean defined.
                            This is the state that the actuator moves to when
                            the home method is called.
        """

        # Our open state is our home state:
        state1 = BaseActuator.get_default_input()
        state1['home_state'] = True
        state1['actuate'] = self.api.sewing_machine_knife_up

        # Our down state has no sensor; it defaults to None:
        state2 = BaseActuator.get_default_input()
        state2['actuate'] = self.api.sewing_machine_knife_down

        input_config = {'up':state1, 'down':state2}

        return BaseActuator(input_config, "SewingMachineKnife")

    def _build_foot_actuator(self):
        """
        Helper function to clean up legibility of constructing an actuator:

        First we define our individual state methods. As below, state1 is
        defined as a dictionary containing important keys that our BaseActuator
        class uses.
            - sense         : A method that returns True if we are at that state
                            (i.e. read a sensor)
            - actuate       : A method that, when called, moves the actuator to
                            the given state (i.e. command a pneumatic valve on)
            - home_state    : At lease one state must have this boolean defined.
                            This is the state that the actuator moves to when
                            the home method is called.
        """

        # Our open state is our home state:
        state1 = BaseActuator.get_default_input()
        state1['home_state'] = True
        state1['actuate'] = self.api.sewing_machine_foot_up

        # Our down state has no sensor; it defaults to None:
        state2 = BaseActuator.get_default_input()
        state2['actuate'] = self.api.sewing_machine_foot_down

        input_config = {'up':state1, 'down':state2}

        return BaseActuator(input_config, "SewingMachineFoot")
