#!/usr/bin/env python3

import json
import rospy
import xml.etree.ElementTree as ET

from gazebo_msgs.srv import SetPhysicsProperties, SetModelConfiguration
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from std_srvs.srv import Empty
from std_msgs.msg import Header


class BenchmarkTraj(object):
    """
    doc
    """

    def __init__(self):
        """
        doc
        """
        rospy.loginfo_once('Initialize joint_trajectory_publisher node')
        self.rate = rospy.Rate(50)

        # Publisher to JointTrajectory robot controller
        self.ur_pub = rospy.Publisher('/eff_joint_traj_controller/command', JointTrajectory, queue_size=10)
        self.grip_pub = rospy.Publisher('/robotiq_3f_controller/command', JointTrajectory, queue_size=10)

        # ROS Services
        self.unpause_proxy = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        BenchmarkTraj.unpause(self)
        self.pause_proxy = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_sim_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.reset_env_proxy = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        self.set_physics = rospy.ServiceProxy('/gazebo/set_physics_properties', SetPhysicsProperties)
        self.set_modelconfig = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)

        # Initialize fire and forget command for the gripper
        self.msg_grip = JointTrajectory()
        self.msg_grip.header = Header()
        self.msg_grip.joint_names = ['palm_finger_1_joint', 'finger_1_joint_1', 'finger_1_joint_2',
                                     'finger_1_joint_3', 'palm_finger_2_joint', 'finger_2_joint_1',
                                     'finger_2_joint_2', 'finger_2_joint_3', 'finger_middle_joint_1',
                                     'finger_middle_joint_2', 'finger_middle_joint_3'
                                     ]
        self.msg_grip.points = [JointTrajectoryPoint()]

        # Initialize fire and forget command for the UR
        self.msg_ur = JointTrajectory()
        self.msg_ur.header = Header()
        self.msg_ur.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                   "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        self.msg_ur.points = [JointTrajectoryPoint()]

        # Trajectory from .txt
        with open('trajectory.txt') as f:
            self.trajectory = json.load(f)

        self.joint_info = {}  # Needed to convert the normalised joint positions into absolute positions.
        urdf = rospy.get_param('/robot_description')
        root = ET.XML(urdf)
        for type_tag in root.findall("joint"):
            name = type_tag.attrib["name"]

            if name in self.msg_grip.joint_names:
                limit_tag = type_tag.find("limit")
                param = {
                    "effort": limit_tag.attrib["effort"],
                    "lower": limit_tag.attrib["lower"],
                    "upper": limit_tag.attrib["upper"],
                    "velocity": limit_tag.attrib["velocity"],
                }
                self.joint_info.update({name: param})

        rospy.sleep(0.5)

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
        BenchmarkTraj.pause(self)
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

    def joint_trajectory(self):
        t_max = self.trajectory['time'][-1]
        t = 0
        xx = len(self.trajectory['commands'])

        #while not rospy.is_shutdown() and t < t_max:
        for i in range(xx):
            #i = int(math.floor(t / 0.016))

            if t == 0 or t > self.trajectory['time'][-1]:
                self.msg_ur.points[0].positions = self.trajectory['commands'][i]['arm']
                #self.msg_grip.points[0].positions = self.trajectory['commands'][i]['gripper']
                self.msg_grip.points[0].positions = BenchmarkTraj.conv_normalized_to_abs(
                    self, self.trajectory['commands'][i]['gripper'])
            else:
                self.msg_ur.points[0].positions = self.trajectory['commands'][i]['arm']

                #self.msg_grip.points[0].positions = self.trajectory['commands'][i]['gripper']
                self.msg_grip.points[0].positions = BenchmarkTraj.conv_normalized_to_abs(
                    self, self.trajectory['commands'][i]['gripper'])

            self.msg_ur.points[0].time_from_start = rospy.Duration.from_sec(0.1)
            self.msg_grip.points[0].time_from_start = rospy.Duration.from_sec(0.1)
            print(i, " from: ", xx)
            try:
                self.ur_pub.publish(self.msg_ur)
                self.grip_pub.publish(self.msg_grip)
                self.rate.sleep()
            except rospy.ROSInterruptException:
                pass

            t += float(64)/1000

    def conv_normalized_to_abs(self, pos_percent):
        positions = []
        for i in range(len(pos_percent)):
            max_pos = float(self.joint_info[self.msg_grip.joint_names[i]]["upper"])
            min_pos = float(self.joint_info[self.msg_grip.joint_names[i]]["lower"])
            target = min_pos + pos_percent[i] * (max_pos - min_pos)
            positions.append(target)
        return positions


def main():
    rospy.init_node('joint_trajectory_publisher')
    try:
        ch = BenchmarkTraj()
        ch.joint_trajectory()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
