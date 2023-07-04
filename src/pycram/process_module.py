"""Implementation of process modules.

Classes:
ProcessModule -- implementation of process modules.
"""
# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

import inspect
import time
from abc import ABC

import rospy

from .designator import MotionDesignatorDescription
from .fluent import Fluent
from typing import Callable, List, Type, Any

from .robot_descriptions.robot_description_handler import InitializedRobotDescription as robot_description


class ProcessModule:
    """
    Implementation of process modules.
    Process modules are the part that communicate with the outer world to execute designators.
    """
    execution_delay = True
    """
    Adds a delay of 0.5 seconds after executing a process module, to make the execution in simulation more realistic
    """

    def __init__(self):
        """Create a new process module."""
        self._running: Fluent = Fluent(False)
        self._designators: List[MotionDesignatorDescription.Motion] = []

    def _execute(self, designator: MotionDesignatorDescription.Motion) -> Any:
        """
        Helper method for internal usage only.
        This method is to be overwritten instead of the execute method.
        """
        pass

    def execute(self, designator: MotionDesignatorDescription.Motion) -> Any:
        """
        Execute the given designator. If the process module is already executing another designator, it queues the
        given designator and executes them in order.

        :param designator: The designator to execute.
        :return: Return of the Process Module if there is any
        """
        self._designators.append(designator)
        # (self._running == False).wait_for()
        self._running.set_value(True)
        designator = self._designators[0]
        try:
            ret = self._execute(designator)
        finally:
            self._running.set_value(False)
        self._designators.remove(designator)
        self._running.set_value(False)
        if ProcessModule.execution_delay:
            time.sleep(0.5)

        return ret


class real_robot:
    """
    Management class for executing designators on the real robot. This is intended to be used in a with environment.
    When importing this class an instance is imported instead.

    Example:

    .. code-block:: python

        with real_robot:
            some designators
    """
    def __init__(self):
        self.pre: str = ""

    def __enter__(self):
        """
        Entering function for 'with' scope, saves the previously set :py:attr:`~ProcessModuleManager.execution_type` and
        sets it to 'real'
        """
        self.pre = ProcessModuleManager.execution_type
        ProcessModuleManager.execution_type = "real"

    def __exit__(self, type, value, traceback):
        """
        Exit method for the 'with' scope, sets the :py:attr:`~ProcessModuleManager.execution_type` to the previously
        used one.
        """
        ProcessModuleManager.execution_type = self.pre

    def __call__(self):
        return self


class simulated_robot:
    """
    Management class for executing designators on the simulated robot. This is intended to be used in a with environment.
    When importing this class an instance is imported instead.

    Example:

    .. code-block:: python

        with simulated_robot:
            some designators
    """
    def __init__(self):
        self.pre: str = ""

    def __enter__(self):
        """
        Entering function for 'with' scope, saves the previously set :py:attr:`~ProcessModuleManager.execution_type` and
        sets it to 'simulated'
        """
        self.pre = ProcessModuleManager.execution_type
        ProcessModuleManager.execution_type = "simulated"

    def __exit__(self, type, value, traceback):
        """
        Exit method for the 'with' scope, sets the :py:attr:`~ProcessModuleManager.execution_type` to the previously
        used one.
        """
        ProcessModuleManager.execution_type = self.pre

    def __call__(self):
        return self


def with_real_robot(func: Callable) -> Callable:
    """
    Decorator to execute designators in the decorated class on the real robot.

    Example:

    .. code-block:: python

        @with_real_robot
        def plan():
            some designators

    :param func: Function this decorator is annotating
    :return: The decorated function wrapped into the decorator
    """
    def wrapper(*args, **kwargs):
        pre = ProcessModuleManager.execution_type
        ProcessModuleManager.execution_type = "real"
        func(*args, **kwargs)
        ProcessModuleManager.execution_type = pre

    return wrapper


def with_simulated_robot(func: Callable) -> Callable:
    """
    Decorator to execute designators in the decorated class on the simulated robot.

    Example:

    .. code-block:: python

        @with_simulated_robot
        def plan():
            some designators

    :param func: Function this decorator is annotating
    :return: The decorated function wrapped into the decorator
    """
    def wrapper(*args, **kwargs):
        pre = ProcessModuleManager.execution_type
        ProcessModuleManager.execution_type = "simulated"
        func(*args, **kwargs)
        ProcessModuleManager.execution_type = pre

    return wrapper


# These are imported, so they don't have to be initialized when executing with
simulated_robot = simulated_robot()
real_robot = real_robot()


class ProcessModuleManager(ABC):
    """
    Base class for managing process modules, any new process modules have to implement this class to register the
    Process Modules
    """
    execution_type = None
    """
    Whether the robot for which the process module is intended for is real or a simulated one
    """
    available_pms = []
    """
    List of all available Process Module Managers
    """

    def __init__(self, robot_name):
        """
        Registers the Process modules for this robot. The name of the robot has to match the name given in the robot
        description.

        :param robot_name: Name of the robot for which these Process Modules are intended
        """
        self.robot_name = robot_name
        ProcessModuleManager.available_pms.append(self)

    @staticmethod
    def get_manager() -> ProcessModuleManager | None:
        """
        Returns the Process Module manager for the currently loaded robot or None if there is no Manager.

        :return: ProcessModuleManager instance of the current robot
        """
        manager = None
        if not ProcessModuleManager.execution_type:
            rospy.logerr(
                f"No execution_type is set, did you use the with_simulated_robot or with_real_robot decorator?")
            return

        for pm_manager in ProcessModuleManager.available_pms:
            if pm_manager.robot_name == robot_description.i.name:
                manager = pm_manager

        if manager:
            return manager
        else:
            rospy.logerr(f"No Process Module Manager found for robot: '{robot_description.i.name}'")

    def navigate(self) -> Type[ProcessModule]:
        """
        Returns the Process Module for navigating the robot with respect to the :py:attr:`~ProcessModuleManager.execution_type`

        :return: The Process Module for navigating
        """
        raise NotImplementedError(
            f"There are no Process Modules for '{inspect.currentframe().f_code.co_name}' for robot '{self.robot_name}'")

    def pick_up(self) -> Type[ProcessModule]:
        """
        Returns the Process Module for picking up with respect to the :py:attr:`~ProcessModuleManager.execution_type`

        :return: The Process Module for picking up an object
        """
        raise NotImplementedError(
            f"There are no Process Modules for '{inspect.currentframe().f_code.co_name}' for robot '{self.robot_name}'")

    def place(self) -> Type[ProcessModule]:
        """
        Returns the Process Module for placing with respect to the :py:attr:`~ProcessModuleManager.execution_type`

        :return: The Process Module for placing an Object
        """
        raise NotImplementedError(
            f"There are no Process Modules for '{inspect.currentframe().f_code.co_name}' for robot '{self.robot_name}'")

    def looking(self) -> Type[ProcessModule]:
        """
        Returns the Process Module for looking at a point with respect to the :py:attr:`~ProcessModuleManager.execution_type`

        :return: The Process Module for looking at a specific point
        """
        raise NotImplementedError(
            f"There are no Process Modules for '{inspect.currentframe().f_code.co_name}' for robot '{self.robot_name}'")

    def detecting(self) -> Type[ProcessModule]:
        """
        Returns the Process Module for detecting an object with respect to the :py:attr:`~ProcessModuleManager.execution_type`

        :return: The Process Module for detecting an object
        """
        raise NotImplementedError(
            f"There are no Process Modules for '{inspect.currentframe().f_code.co_name}' for robot '{self.robot_name}'")

    def move_tcp(self) -> Type[ProcessModule]:
        """
        Returns the Process Module for moving the Tool Center Point with respect to the :py:attr:`~ProcessModuleManager.execution_type`

        :return: The Process Module for moving the TCP
        """
        raise NotImplementedError(
            f"There are no Process Modules for '{inspect.currentframe().f_code.co_name}' for robot '{self.robot_name}'")

    def move_arm_joints(self) -> Type[ProcessModule]:
        """
        Returns the Process Module for moving the joints of the robot arm
        with respect to the :py:attr:`~ProcessModuleManager.execution_type`

        :return: The Process Module for moving the arm joints
        """
        raise NotImplementedError(
            f"There are no Process Modules for '{inspect.currentframe().f_code.co_name}' for robot '{self.robot_name}'")

    def world_state_detecting(self) -> Type[ProcessModule]:
        """
        Returns the Process Module for detecting an object using the world state with respect to the
        :py:attr:`~ProcessModuleManager.execution_type`

        :return: The Process Module for world state detecting
        """
        raise NotImplementedError(
            f"There are no Process Modules for '{inspect.currentframe().f_code.co_name}' for robot '{self.robot_name}'")

    def move_joints(self) -> Type[ProcessModule]:
        """
        Returns the Process Module for moving any joint of the robot with respect to the
        :py:attr:`~ProcessModuleManager.execution_type`

        :return: The Process Module for moving joints
        """
        raise NotImplementedError(
            f"There are no Process Modules for '{inspect.currentframe().f_code.co_name}' for robot '{self.robot_name}'")

    def move_gripper(self) -> Type[ProcessModule]:
        """
        Returns the Process Module for moving the gripper with respect to the :py:attr:`~ProcessModuleManager.execution_type`

        :return: The Process Module for moving the gripper
        """
        raise NotImplementedError(
            f"There are no Process Modules for '{inspect.currentframe().f_code.co_name}' for robot '{self.robot_name}'")

    def open(self) -> Type[ProcessModule]:
        """
        Returns the Process Module for opening drawers with respect to the :py:attr:`~ProcessModuleManager.execution_type`

        :return: The Process Module for opening drawers
        """
        raise NotImplementedError(
            f"There are no Process Modules for '{inspect.currentframe().f_code.co_name}' for robot '{self.robot_name}'")

    def close(self) -> Type[ProcessModule]:
        """
        Returns the Process Module for closing drawers with respect to the :py:attr:`~ProcessModuleManager.execution_type`

        :return: The Process Module for closing drawers
        """
        raise NotImplementedError(
            f"There are no Process Modules for '{inspect.currentframe().f_code.co_name}' for robot '{self.robot_name}'")
