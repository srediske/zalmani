#!/usr/bin/env python
import actionlib
import rospy

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from std_msgs.msg import Header


class MoveTopic(object):
    """
    Class to control the Robot (UR with Robotiq 3f-gripper) via
    joint_trajectory_controller /command topic (fire and forget).
    """

    def __init__(self):
        """
        This constructor initialises the different necessary connections to the topics.
        """
        self.rate = rospy.Rate(50)

        # Publisher to JointTrajectory robot controller
        self.ur_pub = rospy.Publisher('/eff_joint_traj_controller/command', JointTrajectory, queue_size=10)
        self.grip_pub = rospy.Publisher('/robotiq_3f_controller/command', JointTrajectory, queue_size=10)

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

    def grip_trajectory_publisher(self, pose):
        self.rate.sleep()
        self.msg_grip.points[0].positions = pose
        self.msg_grip.points[0].time_from_start = rospy.Duration.from_sec(0.1)
        try:
            self.grip_pub.publish(self.msg_grip)
            self.rate.sleep()
            rospy.sleep(1)
        except rospy.ROSInterruptException:
            pass


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

    def stop(self):
        self.grip_client.cancel_goal()

    def wait(self, timeout=15):
        self.grip_client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self.grip_client.get_result()


def main():
    rospy.init_node('robot_control')

    try:
        ch = MoveAction()
        while not rospy.is_shutdown():
            ch.open_hand()
            ch.close_fist()
    except rospy.ROSInterruptException:
        pass

    # try:
    #     ch = MoveTopic()
    #     while not rospy.is_shutdown():
    #         ch.open_hand()
    #         ch.close_forceps()
    #         ch.open_hand()
    #         ch.close_fist()
    # except rospy.ROSInterruptException:
    #     pass


if __name__ == '__main__':
    main()

