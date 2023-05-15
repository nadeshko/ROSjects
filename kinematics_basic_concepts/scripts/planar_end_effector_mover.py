#!/usr/bin/env python3
"""
subscribes to /ee_pose_commands to find these points and orientations 
generated by circular_motion_ee.py

subscribes to topic /end_effector_real_pose of type geometry_msgs.Vector3
to get real positions and chi angle of frame3 of real robot so we can compare
values

create instance of JointMover from move.py and instance of markerbasics from
rviz_marker.py. using them to move the arm to the calculated poses and publish
marker of the desired new pose

also create instance of method calculate_ik from ik_planar_arm.py to calculate 
thetas to reach position of ellipses.
"""

import rospy
from planar_3dof_control.msg import EndEffector
from geometry_msgs.msg import Vector3
from kinematics_basic_concepts.ik_planar_arm import calculate_ik
from kinematics_basic_concepts.move import JointMover
from kinematics_basic_concepts.rviz_marker import MarkerBasics

class PlanarEndEffectorMover():
    def __init__(self, wait_reach_goal=True):
        self.markerbasic_obj = MarkerBasics()
        self.unique_marker_index = 0

        # start robot
        self.robot_move = JointMover()

        # subscribe to topic for EE pose
        ee_pose_command_topic = "/ee_pose_commands"
        ee_pose_commands_data = None
        rospy.Subscriber(ee_pose_command_topic, EndEffector, self.ee_pose_callback)

        while ee_pose_commands_data is None and not rospy.is_shutdown():
            try:
                ee_pose_commands_data = rospy.wait_for_message(ee_pose_command_topic, EndEffector, timeout=0.5)
            except:
                rospy.logwarn("Waiting for first EE Command Pose in topic = "+ str(ee_pose_command_topic))

        # values from circular motion ee
        self.Pee_x = ee_pose_commands_data.ee_xy_theta.x
        self.Pee_y = ee_pose_commands_data.ee_xy_theta.y
        self.chi = ee_pose_commands_data.ee_xy_theta.z
        self.elbow_pol = ee_pose_commands_data.elbow_policy.data

        # subscription to real pose values
        end_effector_real_pose_topic = "/end_effector_real_pose"
        rospy.Subscriber(end_effector_real_pose_topic, Vector3, self.real_ee_pose_callback)
        end_effector_real_pose_data = None
        while end_effector_real_pose_data is None and not rospy.is_shutdown():
            try:
                end_effector_real_pose_data = rospy.wait_for_message(end_effector_real_pose_topic, Vector3, timeout=0.5)
            except:
                rospy.logwarn("Waiting for first EE Command Pose in topic = "+ str(end_effector_real_pose_topic))
                pass
        
        # real values
        self.Pee_x_real = end_effector_real_pose_data.x
        self.Pee_y_real = end_effector_real_pose_data.y
        self.chi_real = end_effector_real_pose_data.z

    def real_ee_pose_callback(self, msg):
        self.Pee_x_real = msg.x
        self.Pee_y_real = msg.y
        self.chi_real = msg.z

        rospy.loginfo("Pxx_REAL=["+str(self.Pee_x_real)+","+str(self.Pee_y_real)+"] --- CHI="+str(self.chi_real))
        rospy.loginfo("Pxx_OBJE=["+str(self.Pee_x)+","+str(self.Pee_y)+"] --- CHI_real="+str(self.chi))

    def ee_pose_callback(self, msg):
        self.Pee_x = msg.ee_xy_theta.x
        self.Pee_y = msg.ee_xy_theta.y
        self.chi = msg.ee_xy_theta.z
        self.elbow_pol = msg.elbow_policy.data
        r1 = 1.0
        r2 = 1.0
        r3 = 1.0
        DH_parameters={
            "r1":r1,
            "r2":r2,
            "r3":r3}

        theta_array, possible_solution = calculate_ik(
            Pee_x=self.Pee_x, 
            Pee_y=self.Pee_y, 
            chi=self.chi,
            DH_parameters=DH_parameters, 
            elbow_config = self.elbow_pol)

        if possible_solution:
            theta_1 = theta_array[0]
            theta_2 = theta_array[1]
            theta_3 = theta_array[2]

            self.robot_mover.move_all_joints(theta_1, theta_2, theta_3)
            self.markerbasics_object.publish_point(self.Pee_x, self.Pee_y, roll=self.chi, index=self.unique_marker_index)
            self.unique_marker_index += 1 
        else:
            rospy.logerr("NO POSSIBLE SOLUTION FOUND, Robot Cant reach that pose")
        
    def something(self):
        pass

def main():
    rospy.init_node('planar_end_effector_mover')

    planar_object = PlanarEndEffectorMover()
    rospy.spin()

if __name__ == '__main__':
    main()