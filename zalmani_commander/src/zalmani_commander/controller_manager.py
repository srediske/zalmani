"""
Functions for loading, unloading, starting and stopping controllers.

Adoption of ros_control/controller_manager functionality with minor
changes.
"""
from typing import List

import rospy
from controller_manager_msgs.srv import (
    ListControllers,
    ListControllersRequest,
    ListControllersResponse,
    LoadController,
    LoadControllerRequest,
    LoadControllerResponse,
    SwitchController,
    SwitchControllerRequest,
    SwitchControllerResponse,
    UnloadController,
    UnloadControllerRequest,
    UnloadControllerResponse,
)


def list_controllers() -> list:
    """Returns a list of all controllers in the controller_manager.

    Returns:
        list[ControllerState]: List of all controllers in the
            controller_manager with name, state and type.
"""
    rospy.wait_for_service('controller_manager/list_controllers')
    s = rospy.ServiceProxy('controller_manager/list_controllers', ListControllers)
    try:
        resp = s.call(ListControllersRequest())
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        resp = ListControllersResponse()
    if len(resp.controller) == 0:
        rospy.loginfo("No controllers are loaded")
    return resp.controller


def start_stop_controllers(
        start_controllers: List[str] = None,
        stop_controllers: List[str] = None, strictness: int = 2,
        start_asap: bool = False, timeout: float = 0.0,
) -> bool:
    """Start and/or stop a list of controllers.

    Start and/or stop a number of controllers in one single time step of
    the controller_manager control loop with the specified strictness.

    Args:
        start_controllers (list): List of controller names to start
            (state: running).
        stop_controllers (list): List of controller names to stop
            (state: stopped).
        strictness (int): Choose between BEST_EFFORT = 1 and STRICT = 2.
            BEST_EFFORT means that even when something goes wrong with
            on controller, the service will still try to start/stop the
            remaining controllers.
            STRICT means that switching will fail if anything goes wrong
            (an invalid controller name, a controller that failed to
            start, etc. )
        start_asap (bool): If True, start the controllers as soon as
            their hardware dependencies are ready, will wait for all
            interfaces to be ready if False.
        timeout (float): The timeout in seconds before aborting pending
            controllers. Waits infinitely if zero.

    Returns:
        bool: True indicates that the controllers were switched
        successfully. The meaning of success depends on the specified
        strictness.
    """
    rospy.wait_for_service('controller_manager/switch_controller')
    s = rospy.ServiceProxy('controller_manager/switch_controller', SwitchController)
    try:
        resp = s.call(SwitchControllerRequest(
            start_controllers=start_controllers,
            stop_controllers=stop_controllers,
            strictness=strictness,
            start_asap=start_asap,
            timeout=timeout,
        ))
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        resp = SwitchControllerResponse()
    if resp.ok:
        if start_controllers:
            rospy.loginfo(f"Started {start_controllers} successfully")
        if stop_controllers:
            rospy.loginfo(f"Stopped {stop_controllers} successfully")
        return True
    else:
        rospy.loginfo(f"Error when starting {start_controllers} and stopping {stop_controllers}")
        return False


def start_controller(name: str) -> bool:
    """Start a single controller.

    Returns:
        bool: True if the controller were started successfully.
    """
    return start_stop_controllers(start_controllers=[name])


def stop_controller(name: str) -> bool:
    """Stop a single controller.

    Returns:
        bool: True if the controller were stopped successfully.
    """
    return start_stop_controllers(stop_controllers=[name])


def load_controller(name: str) -> bool:
    """Load a single controller in the controller_manager.

    Args:
        name (str): Name of the controller.

    Returns:
        bool: indicates if the controller was successfully loaded (true)
            or not (False).
    """
    rospy.wait_for_service('controller_manager/load_controller')
    s = rospy.ServiceProxy('controller_manager/load_controller', LoadController)
    try:
        resp = s.call(LoadControllerRequest(name))
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        resp = LoadControllerResponse()
    if resp.ok:
        rospy.loginfo(f"Loaded controller: {name} successfully")
        return True
    else:
        rospy.loginfo(f"Error when loading controller: {name}")
        return False


def unload_controller(name: str) -> bool:
    """Unload a single controller from the controller_manager.

    Args:
        name (str): Name of the controller.

    Returns:
        bool: indicates if the controller was successfully
            unloaded (true) or not (False).
    """
    rospy.wait_for_service('controller_manager/unload_controller')
    s = rospy.ServiceProxy('controller_manager/unload_controller', UnloadController)
    try:
        resp = s.call(UnloadControllerRequest(name))
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        resp = UnloadControllerResponse()
    if resp.ok:
        rospy.loginfo(f"Unloaded controller: {name} successfully")
        return True
    else:
        rospy.loginfo(f"Error when unloading controller: {name}")
        return False
