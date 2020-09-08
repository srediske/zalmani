
from controller_manager_msgs.srv import SwitchControllerRequest
from threading import Timer

import rospy
import time


class Move(object):
    """
    Class to control the Robot (UR with Robotiq 3f-gripper).
    """

    def __init__(self):
        """
        This constructor initialises the different necessary connections to the topics and services
        and resets the world to start in a good position.
        """
        rospy.init_node("robot_control")

    def open_hand(self):
        print('do stuff')

    def close_forceps(self):
        print('do stuff')

    def close_fist(self):
        print('do stuff')

    def reset_world(self):
        """
        Resets the object poses in the world and the robot joint angles.
        """
        self.__switch_ctrl.call(start_controllers=[],
                                stop_controllers=["robotiq_3f_controller",
                                                  "pos_joint_traj_controller",
                                                  "joint_state_controller"],
                                strictness=SwitchControllerRequest.BEST_EFFORT)
        self.__pause_physics.call()

        joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
                       'wrist_3_joint', 'finger_1_joint_1', 'finger_1_joint_2', 'finger_1_joint_3', 'finger_2_joint_1',
                       'finger_2_joint_2', 'finger_2_joint_3', 'finger_middle_joint_1', 'finger_middle_joint_2',
                       'finger_middle_joint_3' 'palm_finger_1_joint', 'palm_finger_2_joint']

        joint_positions = [1.2, 0.3, -1.5, -0.5, -1.5, 0.0, 0.0, -0.3, 0.0, 0.0, -0.3, 0.0, 0.0, -0.3, 0.0]

        self.__set_model.call(model_name="robot",
                              urdf_param_name="robot_description",
                              joint_names=joint_names,
                              joint_positions=joint_positions)

        timer = Timer(0.0, self.__start_ctrl)
        timer.start()

        time.sleep(0.1)
        self.__unpause_physics.call()

        self.__reset_world.call()


def main():
    print('do stuff')


if __name__ == '__main__':
    main()
