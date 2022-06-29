"""
Python module to control a robot with the ros_control package.

To get control over a robot via ros_control actions, an ActionClient
is necessary, which communicates with with the ActionServer of the
corresponding controller.

Note:
    This module is explicitly intended for use with Gazebo and not
    with a real robot.
"""
import xml.etree.ElementTree as ET
from typing import Optional

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryActionResult,
    FollowJointTrajectoryFeedback,
    FollowJointTrajectoryGoal,
    JointTrajectoryControllerState,
)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class Robot:
    """Base class for controlling a robot via SimpleActionClient.

    All public methods of this class are used to control a robot via
    the actionlib SimpleActionClient class or to retrieve information
    from the current state.

    Args:
        controller_name (str): Name (not type!) of the controller which
            should be connected. The name is usually the root for all
            controller specific parameters and topics.
        joints (dict): A dictionary with all joints belonging to the
            controller and the initial joint positions.

    Attributes:
        controller_name (str): "Cleaned" name of the controller and the
            associated ROS topic.
        feedback (FollowJointTrajectoryFeedback): Feedback received
            from the ActionServer.
        joints (dict): The same dictionary as the argument joints, but
            supplemented by the respective joint limits.
        result (FollowJointTrajectoryActionResult): The last received
            result from the ActionServer (only one per goal).
        state (JointTrajectoryControllerState): Current state.
    """

    def __init__(self, controller_name: str, joints: dict):
        self.controller_name = controller_name if controller_name[-1] != '/' else controller_name[:-1]

        self._client = actionlib.SimpleActionClient(
            f'{self.controller_name}/follow_joint_trajectory',
            FollowJointTrajectoryAction,
        )

        if not self._client.wait_for_server(timeout=rospy.Duration(10)):
            rospy.logerr(
                f"Timed out waiting (10 s) to connect "
                f"{self.controller_name} to ActionServer. "
                f"Check ROS network.")
            rospy.signal_shutdown(
                f"Timed out waiting for {self.controller_name} "
                f"ActionServer")

        self.joints = self._get_joint_limits(joints=joints)
        self.init_positions = [self.joints[i]['init_position'] for i in self.joints]

        self.feedback = FollowJointTrajectoryFeedback()
        self.result = FollowJointTrajectoryActionResult()
        self.state = JointTrajectoryControllerState()

        self._state_sub = rospy.Subscriber(self.controller_name + '/state',
                                           JointTrajectoryControllerState,
                                           callback=self._cb_state,
                                           queue_size=1
                                           )
        self._cmd_pub = rospy.Publisher(self.controller_name + '/command',
                                        JointTrajectory,
                                        queue_size=1
                                        )
        rospy.loginfo(f"Initialized ActionClient: {self.controller_name}")

    @staticmethod
    def _get_joint_limits(joints: dict) -> dict:
        """Reads ROS parameter server for joint limits.

        Args:
            joints (dict): Dictionary with the joints of the robot,
                which is supplemented with the joint limits for lower-
                and upper positions, effort and velocity.

        Returns:
            dict: Dictionary with all joint limits found by the
                parameter server.
        """
        urdf = rospy.get_param('/robot_description')
        root = ET.XML(urdf)
        for tag in root.findall("joint"):
            name = tag.attrib["name"]
            if name in joints:
                limit_tag = tag.find("limit")
                limits = {
                    "lower": float(limit_tag.attrib["lower"]),
                    "upper": float(limit_tag.attrib["upper"]),
                    "effort": float(limit_tag.attrib["effort"]),
                    "velocity": float(limit_tag.attrib["velocity"]),
                }
                joints[name].update({'limits': limits})
        return joints

    def trajectory_publisher(self, goal: FollowJointTrajectoryGoal) -> None:
        """Send a new goal (trajectory) to the ActionServer.

        Args:
            goal (FollowJointTrajectoryGoal): Fully described Goal.
        """
        self._client.send_goal(goal=goal,
                               active_cb=self._cb_active,
                               done_cb=self._cb_done,
                               feedback_cb=self._cb_feedback,
                               )

    def trajectory_cmd(self, trajectory: JointTrajectory) -> None:
        """Publish a trajectory via the <controller_name>/command topic.

        Args:
            trajectory (JointTrajectory): Fully described trajectory.
        """
        try:
            self._cmd_pub.publish(trajectory)
        except rospy.ROSInterruptException:
            pass

    def send_trajectory_point(self, point: JointTrajectoryPoint) -> None:
        """Publish a trajectory with a single trajectory point.

        Args:
            point (JointTrajectoryPoint): A trajectory point specifies
                either positions
        """
        trajectory = JointTrajectory(joint_names=[*self.joints], points=[point])
        self.trajectory_cmd(trajectory)

    def _cb_active(self) -> None:
        """Gets called when the ActionServer transitions to Active."""
        rospy.logdebug(f"{self.controller_name}: received new goal")

    def _cb_feedback(self, msg: FollowJointTrajectoryFeedback) -> None:
        """Receive auxiliary information for the current goal.

        Contains information on desired and current joint conditions and
        the resulting error.

        Args:
            msg (FollowJointTrajectoryFeedback): Feedback message from
                the action server.
        """
        self.feedback = msg

    def _cb_done(self, state, result) -> None:
        """Gets the result of the finished goal on transition to Done.

        Possible results (error_string = error_code):
            None = no result exists
            SUCCESSFUL = 0
            INVALID_GOAL = -1
            INVALID_JOINTS = -2
            OLD_HEADER_TIMESTAMP = -3
            PATH_TOLERANCE_VIOLATED = -4
            GOAL_TOLERANCE_VIOLATED = -5

        Possible status:
            PENDING = 0
            ACTIVE = 1
            PREEMPTED = 2
            SUCCEEDED = 3
            ABORTED = 4
            REJECTED = 5
            PREEMPTING = 6
            RECALLING = 7
            RECALLED = 8
            LOST = 9

        Args:
            state (int): Terminal state of the ActionServer.
            result (FollowJointTrajectoryResult): The result from the
                ActionServer.
        """
        self.result.status = GoalStatus(status=state)
        self.result.result = result
        rospy.logdebug(
            f"{self.controller_name}: goal is done.\n"
            f"State: {self.result.status}\nresult: {self.result.result}"
        )

    def _cb_state(self, msg: JointTrajectoryControllerState) -> None:
        """Receive information about the current joint states.

        Contains information on desired and current joint conditions and
        the resulting error.

        Args:
            msg: Message from the /state topic.
        """
        self.state = msg

    def cancel(self) -> None:
        """Send a cancel request for the current goal."""
        self._client.cancel_goal()

    def cancel_all_goals(self) -> None:
        """Send a cancel request for all goals."""
        self._client.cancel_all_goals()

    def get_state(self) -> int:
        """Get the state information for the current goal.

        Possible status:
            PENDING = 0
            ACTIVE = 1
            PREEMPTED = 2
            SUCCEEDED = 3
            ABORTED = 4
            REJECTED = 5
            PREEMPTING = 6
            RECALLING = 7
            RECALLED = 8
            LOST = 9

        Returns:
            int: The goal's current state.
        """
        return self._client.get_state()

    def wait_for_result(self, timeout: float = 0.0) -> bool:
        """Block until the result for the current goal is available.

        Args:
             timeout (float): Max time in seconds to block before
                returning. A zero timeout is interpreted as an infinite
                timeout.

        Returns:
            bool: Returns True if the goal finished. False if the goal
                didn't finish within the allocated timeout.
        """
        timeout = rospy.Duration().from_sec(timeout)
        return self._client.wait_for_result(timeout=timeout)

    def reset(self, time_from_start: float = 0.01) -> None:
        """Sets the robot to its initial pose.

        Publish the initial joint positions on the
        <controller_name>/command topic.

        Args:
            time_from_start (float): The relative time from the
                header.stamp to the trajectory point.
        """
        trajectory_point = JointTrajectoryPoint(
            positions=self.init_positions,
            time_from_start=rospy.Duration.from_sec(time_from_start),
        )
        self.send_trajectory_point(trajectory_point)

    def hold_position(self):
        """Hold position immediately.

        Sends a empty trajectory message on the /command topic which
        will stop the execution of all queued trajectories and enter
        position hold mode. The 'stop_trajectory_duration' parameter in
        the controller controls the duration of the stop motion.
        """
        msg = JointTrajectory()
        msg.joint_names = list(self.joints.keys())
        try:
            self._cmd_pub.publish(msg)
        except rospy.ROSInterruptException:
            pass


class UniversalRobots(Robot):
    """Create and connect a ActionClient for Universal Robots arms.

    The initial joint positions must be in the order of the controller
    config file (.yaml) and should correspond to the positions of the
    launch file for the simulation.

    Args:
        init_pos (list): The initial joint positions.
        effort_controller (bool): If True, it connects the effort
            controller, otherwise it connects the position controller.
    """

    def __init__(
        self,
        init_pos: Optional[list] = None,
        *,
        effort_controller: bool = False
    ):
        if effort_controller:
            topic_name = '/eff_joint_traj_controller'
        else:
            topic_name = '/pos_joint_traj_controller'

        if not init_pos:
            init_pos = [0.0 for _ in range(6)]
            rospy.loginfo(
                f"{topic_name}: Setting all initial joint positions to 0.0")
        if len(init_pos) != 6:
            init_pos = [0.0 for _ in range(6)]
            rospy.logwarn(
                f"{topic_name}: The amount of initial joint positions "
                f"entered is not equal to the amount of joints.")

        joints = {'shoulder_pan_joint': {'init_position': init_pos[0]},
                  'shoulder_lift_joint': {'init_position': init_pos[1]},
                  'elbow_joint': {'init_position': init_pos[2]},
                  'wrist_1_joint': {'init_position': init_pos[3]},
                  'wrist_2_joint': {'init_position': init_pos[4]},
                  'wrist_3_joint': {'init_position': init_pos[5]},
                  }
        super().__init__(controller_name=topic_name, joints=joints)


class TwoFingerGripper(Robot):
    """Create and connect a ActionClient for Robotiq 2f-Gripper.

    The initial joint position should correspond to the position of the
    launch file for the simulation.

    Args:
        init_pos (list): The initial joint position.
        effort_controller (bool): If True, it connects the effort
            controller, otherwise it connects the position controller.
    """

    def __init__(
        self,
        init_pos: Optional[list] = None,
        *,
        effort_controller: bool = False
    ):
        if effort_controller:
            topic_name = '/robotiq_eff_joint_traj_controller'
        else:
            topic_name = '/robotiq_pos_joint_traj_controller'

        if not init_pos:
            init_pos = [0.0]
            rospy.loginfo(
                f"{topic_name}: Setting all initial joint positions to 0.0")
        if len(init_pos) != 1:
            init_pos = [0.0]
            rospy.logwarn(
                f"{topic_name}: The amount of initial joint positions "
                f"entered is not equal to the amount of joints.")

        joints = {'finger_joint': {'init_position': init_pos[0]}}
        super().__init__(controller_name=topic_name, joints=joints)


class ThreeFingerGripper(Robot):
    """Create and connect a ActionClient for Robotiq 3f-Gripper.

    The initial joint positions must be in the order of the controller
    config file (.yaml) and should correspond to the positions of the
    launch file for the simulation.

    Args:
        init_pos (list): The initial joint positions.
        effort_controller (bool): If True, it connects the effort
            controller, otherwise it connects the position controller.
    """

    def __init__(
        self,
        init_pos: Optional[list] = None,
        *,
        effort_controller: bool = False
    ):
        if effort_controller:
            topic_name = '/robotiq_eff_joint_traj_controller'
        else:
            topic_name = '/robotiq_pos_joint_traj_controller'

        if not init_pos:
            init_pos = [0.0 for _ in range(11)]
            rospy.loginfo(
                f"{topic_name}: Setting all initial joint positions to 0.0")
        if len(init_pos) != 11:
            init_pos = [0.0 for _ in range(11)]
            rospy.logwarn(
                f"{topic_name}: The amount of initial joint positions "
                f"entered is not equal to the amount of joints.")

        joints = {'finger_1_joint_1': {'init_position': init_pos[0]},
                  'finger_1_joint_2': {'init_position': init_pos[1]},
                  'finger_1_joint_3': {'init_position': init_pos[2]},
                  'finger_2_joint_1': {'init_position': init_pos[3]},
                  'finger_2_joint_2': {'init_position': init_pos[4]},
                  'finger_2_joint_3': {'init_position': init_pos[5]},
                  'finger_middle_joint_1': {'init_position': init_pos[6]},
                  'finger_middle_joint_2': {'init_position': init_pos[7]},
                  'finger_middle_joint_3': {'init_position': init_pos[8]},
                  'palm_finger_1_joint': {'init_position': init_pos[9]},
                  'palm_finger_2_joint': {'init_position': init_pos[10]},
                  }
        super().__init__(controller_name=topic_name, joints=joints)
