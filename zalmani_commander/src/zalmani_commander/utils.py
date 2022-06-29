"""Collection of frequently used functions for ROS/Gazebo."""
from typing import List

import rospy
from gazebo_msgs.msg import LinkStates, ModelState, ODEPhysics
from gazebo_msgs.srv import (
    DeleteModel,
    DeleteModelRequest,
    DeleteModelResponse,
    GetModelState,
    GetModelStateRequest,
    GetModelStateResponse,
    GetPhysicsProperties,
    GetPhysicsPropertiesRequest,
    GetPhysicsPropertiesResponse,
    SetModelConfiguration,
    SetModelConfigurationRequest,
    SetModelConfigurationResponse,
    SetModelState,
    SetModelStateRequest,
    SetModelStateResponse,
    SetPhysicsProperties,
    SetPhysicsPropertiesRequest,
    SetPhysicsPropertiesResponse,
    SpawnModel,
    SpawnModelRequest,
    SpawnModelResponse,
)
from geometry_msgs.msg import Pose, Vector3
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty, EmptyRequest


def pause() -> None:
    """Pause the simulation."""
    rospy.wait_for_service('/gazebo/pause_physics')
    s = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
    try:
        s.call(EmptyRequest())
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")


def unpause() -> None:
    """Unpause the simulation."""
    rospy.wait_for_service('/gazebo/unpause_physics')
    s = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
    try:
        s.call(EmptyRequest())
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")


def reset_world() -> None:
    """Reset the positions and orientations of all models.

    Note:
        Does not reset the time/clock.
    """
    rospy.wait_for_service('/gazebo/reset_world')
    s = rospy.ServiceProxy('/gazebo/reset_world', Empty)
    try:
        s.call(EmptyRequest())
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")


def reset_sim() -> None:
    """Reset the entire simulation environment including the time."""
    rospy.wait_for_service('/gazebo/reset_simulation')
    s = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
    try:
        s.call(EmptyRequest())
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")


def spawn_model(
        name: str, model: str, pose: Pose, ns: str = '/', frame: str = 'world',
) -> bool:
    """Spawn a model in the simulation.

    Args:
        name (str): Name of the model to be spawned.
        model (str): The path of the sdf or urdf file.
        pose (Pose): Initial pose of the canonical link.
        ns (str): Spawn the model and all ROS interfaces under this
            namespace.
        frame (str): Initial 'pose' is defined relative to the frame.

    Returns:
        bool: True if the spawn was successful.
    """
    if 'xml' not in model:
        xml = _load_model(file=model)
    else:
        xml = model

    if '.sdf' in model:
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        s = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    elif '.urdf' in model:
        rospy.wait_for_service('/gazebo/spawn_urdf_model')
        s = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
    else:
        rospy.logerr(
            f"Only models with .sdf and .urdf file extensions are "
            f"possible. Input: {model}"
        )
        return False
    req = SpawnModelRequest(
        model_name=name,
        model_xml=xml,
        robot_namespace=ns,
        initial_pose=pose,
        reference_frame=frame,
    )
    try:
        resp = s.call(req)
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        resp = SpawnModelResponse()
    return resp.success


def _load_model(file: str) -> str:
    try:
        with open(file, 'r') as f:
            return f.read()
    except OSError as e:
        rospy.logerr(f"Could not open/read file: {file}\n{e}")


def delete_model(name: str) -> bool:
    """Delete the model from the simulation.

    Args:
        name (str): Name of the model to be deleted.

    Returns:
        bool: True if the delete was successful.
    """
    rospy.wait_for_service('/gazebo/delete_model')
    s = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    try:
        resp = s.call(DeleteModelRequest(model_name=name))
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        resp = DeleteModelResponse()
    return resp.success


def set_model_config(
        name: str, joints: List[str], positions: List[float],
        *, pause_sim: bool = False,
) -> bool:
    """Set model/robot joint positions without invoking dynamics.

    Args:
        name (str): Name of the model/robot to set state.
        joints (list): All joint names of the model/robot to set
            position. If joint is not listed, preserve current position.
        positions (list): Set joints to this positions.
        pause_sim (bool): If True, the simulation remains paused after
            this service is finished. This is useful if the ros_control
            plugin is used. Otherwise ros_control will try to resume the
            previous positions immediately after the pause is finished.
            Therefore the pause_sim=True should be set and then the
            state of the ros_control plugin should be updated before
            unpausing the simulation.

    Returns:
        bool: True on successful setting of the model state.
    """
    pause()  # Gazebo must be paused for this service to be performed safely.
    rospy.wait_for_service('/gazebo/set_model_configuration')
    s = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
    req = SetModelConfigurationRequest(
        model_name=name,
        urdf_param_name='',  # unused
        joint_names=joints,
        joint_positions=positions,
    )
    try:
        resp = s.call(req)
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        resp = SetModelConfigurationResponse()
    if not pause_sim:
        unpause()
    return resp.success


def set_model_pose(name: str, pose: Pose, frame: str = 'world') -> bool:
    """Set model pose (position and orientation).

    Args:
        name (str): Name of the model to set pose.
        pose (Pose): Desired pose relative to the reference frame.
        frame (str): 'pose' is defined relative to the frame.

    Returns:
        bool: True with successful setting of the model pose.
    """
    rospy.wait_for_service('/gazebo/set_model_state')
    s = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    state = ModelState(model_name=name, pose=pose, reference_frame=frame)
    try:
        resp = s.call(SetModelStateRequest(model_state=state))
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        resp = SetModelStateResponse()
    return resp.success


def get_physics() -> GetPhysicsPropertiesResponse:
    """Get current physics configuration.

    Returns:
        GetPhysicsPropertiesResponse: Contains physics configuration.
    """
    rospy.wait_for_service('/gazebo/get_physics_properties')
    s = rospy.ServiceProxy('/gazebo/get_physics_properties', GetPhysicsProperties)
    try:
        resp = s.call(GetPhysicsPropertiesRequest())
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        resp = GetPhysicsPropertiesResponse()
    return resp


def set_physics(
        step_size: float = None, update_rate: int = None,
        gravity: Vector3 = None, ode_config: ODEPhysics = None,
) -> bool:
    """Set physics configurations.

    Args:
        step_size (float): Time step size for ODE or max step size for
            variable step-time solver (e.g. simbody).
        update_rate (int): The desired frequency with which the
            simulation is to be calculated:
                RTF = max_step_size * update_rate, at update_rate = 0
                the system tries to simulate as fast as possible.
        gravity (Vector3): Gravity vector in m/s^2.
        ode_config (ODEPhysics): Contains physics configurations
            pertaining to ODE.

    Returns:
        bool: True if the physics parameters have been successfully
            changed.
    """
    physics = get_physics()
    if not step_size:
        step_size = physics.time_step
    if not update_rate:
        update_rate = physics.max_update_rate
    if not gravity:
        gravity = physics.gravity
    if not ode_config:
        ode_config = physics.ode_config
    pause()
    rospy.wait_for_service('/gazebo/set_physics_properties')
    s = rospy.ServiceProxy('/gazebo/set_physics_properties', SetPhysicsProperties)
    req = SetPhysicsPropertiesRequest(
        time_step=step_size,
        max_update_rate=update_rate,
        gravity=gravity,
        ode_config=ode_config,
    )
    try:
        resp = s.call(req)
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        resp = SetPhysicsPropertiesResponse()
    unpause()
    return resp.success


def joint_states(timeout: int = None) -> JointState:
    """Retrieve the current joint states from /joint_states.

    Args:
        timeout (int): Duration in seconds before aborting.
            Waits infinitely if zero or None.

    Returns:
        JointState: Broadcast the state of all torque controlled joints.
    """
    if timeout:
        timeout = rospy.Duration(timeout)
    elif timeout == 0:
        timeout = None
    try:
        msg = rospy.wait_for_message(
            topic='/joint_states',
            topic_type=JointState,
            timeout=timeout,
        )
    except rospy.ROSException as e:
        rospy.logerr(e)
        msg = JointState()
    return msg


def link_states(timeout: int = None) -> LinkStates:
    """Retrieve the current link states from /gazebo/link_states.

    Args:
        timeout (int): Duration in seconds before aborting.
            Waits infinitely if zero or None.

    Returns:
        LinkStates: Broadcast all link states in world frame.
    """
    if timeout:
        timeout = rospy.Duration(timeout)
    elif timeout == 0:
        timeout = None
    try:
        msg = rospy.wait_for_message(
            topic='/gazebo/link_states',
            topic_type=LinkStates,
            timeout=timeout,
        )
    except rospy.ROSException as e:
        rospy.logerr(e)
        msg = LinkStates()
    return msg


def get_model_state(name: str,
                    relative_entity_name: str = '',
                    ) -> GetModelStateResponse:
    """Get current model state.

    Args:
        name (str): Name of the model/robot.
        relative_entity_name (str): Return pose and twist relative to
            this entity. An entity can be a model, body or geom. Be sure
            to use gazebo scoped naming notation
            (e.g. [model_name::body_name]). Leave empty or 'world' will
            use inertial world frame.

    Returns:
        GetPhysicsPropertiesResponse: Contains the model state.
    """
    rospy.wait_for_service('/gazebo/get_model_state')
    s = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    try:
        resp = s.call(GetModelStateRequest(name, relative_entity_name))
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        resp = GetModelStateResponse()
    return resp
