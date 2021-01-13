#!/usr/bin/env python3

import actionlib
import math
import rospy

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from gazebo_msgs.msg import ODEPhysics
from gazebo_msgs.srv import SetPhysicsProperties, SetPhysicsPropertiesRequest, SetModelConfiguration, \
    SetModelConfigurationRequest
from geometry_msgs.msg import Vector3
from time import time
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from std_srvs.srv import Empty
from std_msgs.msg import Float64, Header


class MoveTopic(object):
    """
    Class to control the Robot (UR with Robotiq 3f-gripper) via
    joint_trajectory_controller /command topic (fire and forget).
    """

    def __init__(self):
        """
        This constructor initialises the different necessary connections to the topics.
        """
        rospy.loginfo_once('Initialize control node')
        self.rate = rospy.Rate(50)

        # Publisher to JointTrajectory robot controller
        self.ur_pub = rospy.Publisher('/eff_joint_traj_controller/command', JointTrajectory, queue_size=10)
        self.grip_pub = rospy.Publisher('/robotiq_3f_controller/command', JointTrajectory, queue_size=10)

        # ROS Services
        self.unpause_proxy = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        MoveTopic.unpause(self)
        self.pause_proxy = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_sim_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.reset_env_proxy = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        self.set_physics = rospy.ServiceProxy('/gazebo/set_physics_properties', SetPhysicsProperties)
        self.set_modelconfig = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)

        # Initialize fire and forget command for the gripper
        self.msg_grip = JointTrajectory()
        self.msg_grip.header = Header()
        self.msg_grip.joint_names = ['finger_1_joint_1', 'finger_1_joint_2', 'finger_1_joint_3',
                                     'finger_2_joint_1', 'finger_2_joint_2', 'finger_2_joint_3',
                                     'finger_middle_joint_1', 'finger_middle_joint_2', 'finger_middle_joint_3',
                                     'palm_finger_1_joint', 'palm_finger_2_joint']
        self.msg_grip.points = [JointTrajectoryPoint()]

        # Initialize fire and forget command for the UR
        self.msg_ur = JointTrajectory()
        self.msg_ur.header = Header()
        self.msg_ur.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                   "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        self.msg_ur.points = [JointTrajectoryPoint()]

        self.ur_zero_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.grip_zero_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        rospy.sleep(0.5)

    def open_hand(self):
        pose = self.grip_zero_pos
        MoveTopic.grip_trajectory_publisher(self, pose=pose)

    def close_forceps(self):
        pose = [1.0, 0.0, -1.0, 1.0, 0.0, -1.0, 1.0, 0.0, -1.0, -0.2, 0.2]
        MoveTopic.grip_trajectory_publisher(self, pose=pose)

    def open_forceps(self):
        pose = [-1.0, 0.0, -1.0, -1.0, 0.0, -1.0, -1.0, 0.0, -1.0, -0.2, 0.2]
        MoveTopic.grip_trajectory_publisher(self, pose=pose)

    def close_fist(self):
        pose = [1.22, 1.22, 1.22, 1.22, 1.22, 1.22, 1.22, 1.22, 1.22, 0.0, 0.0]
        MoveTopic.grip_trajectory_publisher(self, pose=pose)

    def grip_single_joint(self, joint, angle):
        rospy.sleep(1)
        pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        pose[joint-1] = angle
        MoveTopic.grip_trajectory_publisher(self, pose=pose)

    def grip_sinusoidal(self, joint, sec):
        if sec == 0:
            while not rospy.is_shutdown():
                MoveTopic.grip_sinusoidal_pose(self, joint=joint)
        else:
            start = time()
            end = start + sec
            while time() <= end:
                MoveTopic.grip_sinusoidal_pose(self, joint=joint)

    def grip_sinusoidal_pose(self, joint):
        # Joint between palm and finger
        if joint in (1, 4, 7):
            sine = 0.5861 * math.sin(time()) + 0.6357
        # Joint in the middle
        elif joint in (2, 5, 8):
            sine = math.pi/4 * math.sin(time()) + math.pi/4
        # Fingertip joint
        elif joint in (3, 6, 9):
            sine = 0.5847 * math.sin(time()) - 0.637
        # Palm joints
        elif joint == 10:
            sine = 0.1852 * math.sin(time()) + 0.0068
        else:
            sine = 0.1852 * math.sin(time()) - 0.0068

        pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        pose[joint-1] = sine
        MoveTopic.grip_trajectory_publisher(self, pose=pose)

    def grip_trajectory_publisher(self, pose):
        self.msg_grip.points[0].positions = pose
        self.msg_grip.points[0].time_from_start = rospy.Duration.from_sec(0.1)
        try:
            self.grip_pub.publish(self.msg_grip)
            #rospy.sleep(1)
            self.rate.sleep()
        except rospy.ROSInterruptException:
            pass

    def ur_single_joint(self, joint, angle):
        rospy.sleep(1)
        pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        pose[joint-1] = angle
        MoveTopic.ur_trajectory_publisher(self, pose=pose)

    def ur_sinusoidal(self, joint, sec):
        if sec == 0:
            while not rospy.is_shutdown():
                MoveTopic.ur_sinusoidal_pose(self, joint=joint)
        else:
            start = time()
            end = start + sec
            while time() <= end:
                MoveTopic.ur_sinusoidal_pose(self, joint=joint)

    def ur_sinusoidal_pose(self, joint):
        if joint in (1, 2):
            sine = 0.5 * math.pi * math.sin(time()) - 0.5 * math.pi
        else:
            sine = math.sin(time()) - 0.5 * math.pi

        pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        pose[joint-1] = sine
        MoveTopic.ur_trajectory_publisher(self, pose=pose)

    def ur_trajectory_publisher(self, pose):
        self.msg_ur.points[0].positions = pose
        self.msg_ur.points[0].time_from_start = rospy.Duration.from_sec(0.1)
        try:
            self.ur_pub.publish(self.msg_ur)
            self.rate.sleep()
        except rospy.ROSInterruptException:
            pass

    def pause(self):
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause_proxy()
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s' % e)
            print('Service call failed: %s %e" % (/gazebo/pause_physics, e)')

    def unpause(self):
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause_proxy()
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s' % e)
            print('Service call failed: %s %e" % (/gazebo/unpause_physics, e)')

    def reset_sim(self):
        MoveTopic.pause(self)
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            self.reset_sim_proxy()
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s' % e)
            print('Service call failed: %s %e" % (/gazebo/reset_simulation, e)')

    def reset_env(self):
        rospy.wait_for_service('/gazebo/reset_world')
        try:
            self.reset_env_proxy()
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s' % e)
            print('Service call failed: %s %e" % (/gazebo/reset_world, e)')

    def physics_params(self, step_size=0.001, update_rate=1000):
        self._time_step = Float64(step_size)
        self._max_update_rate = Float64(update_rate)

        self._gravity = Vector3()
        self._gravity.x = 0.0
        self._gravity.y = 0.0
        self._gravity.z = -9.8

        self._ode_config = ODEPhysics()
        self._ode_config.auto_disable_bodies = False
        self._ode_config.sor_pgs_precon_iters = 0
        self._ode_config.sor_pgs_iters = 50
        self._ode_config.sor_pgs_w = 1.3
        self._ode_config.sor_pgs_rms_error_tol = 0.0
        self._ode_config.contact_surface_layer = 0.001
        self._ode_config.contact_max_correcting_vel = 100.0
        self._ode_config.cfm = 0.0
        self._ode_config.erp = 0.2
        self._ode_config.max_contacts = 20

    def change_physics(self):
        MoveTopic.physics_params(self, step_size=0.002, update_rate=1000)
        MoveTopic.pause(self)
        set_physics_request = SetPhysicsPropertiesRequest()
        set_physics_request.time_step = self._time_step.data
        set_physics_request.max_update_rate = self._max_update_rate.data
        set_physics_request.gravity = self._gravity
        set_physics_request.ode_config = self._ode_config

        response = self.set_physics(set_physics_request)
        rospy.loginfo('Gazebo ' + str(response.status_message))

        MoveTopic.unpause(self)

    def set_pose(self, pose=None):
        '''
        This Service allows user to set model joint positions without invoking dynamics.
        '''
        if pose is None:
            raise ValueError("No new pose was given")
        MoveTopic.pause(self)
        set_pose = SetModelConfigurationRequest()
        set_pose.model_name = 'robot'
        set_pose.urdf_param_name = ''
        set_pose.joint_names = self.msg_ur.joint_names
        set_pose.joint_positions = pose

        response = self.set_modelconfig(set_pose)


class MoveAction(object):
    """
    Class to control the Robot (UR with Robotiq 3f-gripper) via
    joint_trajectory_controller action interface.
    """

    def __init__(self):
        """
        This constructor initialises the different necessary connections to the topics.
        """
        self.rate = rospy.Rate(50)
        self.grip_client = actionlib.SimpleActionClient('/robotiq_3f_controller/follow_joint_trajectory',
                                                        FollowJointTrajectoryAction)
        self.grip_goal = FollowJointTrajectoryGoal()
        self.grip_goal.trajectory.header = Header()
        self.grip_goal.trajectory.joint_names = ['finger_1_joint_1', 'finger_1_joint_2', 'finger_1_joint_3',
                                                 'finger_2_joint_1', 'finger_2_joint_2', 'finger_2_joint_3',
                                                 'finger_middle_joint_1', 'finger_middle_joint_2',
                                                 'finger_middle_joint_3',
                                                 'palm_finger_1_joint', 'palm_finger_2_joint']
        self.grip_goal.trajectory.points = [JointTrajectoryPoint()]
        self.grip_goal.goal_time_tolerance = rospy.Time(1)
        server_up = self.grip_client.wait_for_server(timeout=rospy.Duration(10))
        if not server_up:
            rospy.logerr('Timed out waiting for connection to Action Server.'
                         ' Check ROS network.')
            rospy.signal_shutdown('Timed out waiting for Action Server')

        self.grip_zero_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def open_hand(self):
        pose = self.grip_zero_pos
        MoveAction.grip_trajectory_publisher(self, pose=pose)

    def close_fist(self):
        pose = [1.22, 1.22, 1.22, 1.22, 1.22, 1.22, 1.22, 1.22, 1.22, 0.0, 0.0]
        MoveAction.grip_trajectory_publisher(self, pose=pose)

    def grip_trajectory_publisher(self, pose):
        self.grip_goal.trajectory.header.stamp = rospy.Time.now()
        self.grip_goal.trajectory.points[0].positions = pose
        self.grip_goal.trajectory.points[0].time_from_start = rospy.Duration.from_sec(0.1)
        self.grip_client.send_goal(self.grip_goal)
        MoveAction.wait(self)
        rospy.sleep(1)

    def stop(self):
        self.grip_client.cancel_goal()

    def wait(self, timeout=15):
        self.grip_client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self.grip_client.get_result()


def main():
    rospy.init_node('robot_control')

    # try:
    #     ch = MoveAction()
    #     while not rospy.is_shutdown():
    #         ch.open_hand()
    #         ch.close_fist()
    # except rospy.ROSInterruptException:
    #     pass

    # try:
    #     ch = MoveTopic()
    #     while not rospy.is_shutdown():
    #         ch.open_hand()
    #         ch.close_forceps()
    #         ch.open_hand()
    #         ch.close_fist()
    # except rospy.ROSInterruptException:
    #     pass

    ch = MoveTopic()

    # ch.unpause()
    # ch.close_fist()
    # ch.close_forceps()
    # ch.open_forceps()
    # ch.pause()
    # ch.reset_sim()
    # ch.reset_env()
    # ch.unpause()
    # ch.open_hand()
    # ch.change_physics()

    while not rospy.is_shutdown():
        # ch.open_hand()
        # ch.close_fist()
        ch.grip_single_joint(joint=3, angle=-1.05)
        rospy.sleep(1)
        ch.grip_single_joint(joint=3, angle=-0.07)
        rospy.sleep(1)
        #ch.ur_single_joint(joint=1, angle=-0.1)
        #ch.ur_single_joint(joint=1, angle=0)

    # See __init__ for joint list (finger 1: 1 to 3, finger 2: 4 to 6, finger 3/middle: 7 to 9, palm: 10 to 11)
    ch.grip_sinusoidal(joint=3, sec=0)

    #ch.ur_sinusoidal(joint=2, sec=0)


if __name__ == '__main__':
    main()
