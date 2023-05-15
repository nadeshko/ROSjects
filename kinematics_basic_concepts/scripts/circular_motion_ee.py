#!/usr/bin/env python3
"""
in charge of generating elliptical positions and publish them in topic 
/ee_pose_commands of message type planar_3dof_control.EndEffector
"""

import rospy
from geometry_msgs.msg import Vector3
from planar_3dof_control.msg import EndEffector
from math import sin, cos, pi

class EE_Client(object):
    def __init__(self):
        self.publisher = rospy.Publisher('/ee_pose_commands', EndEffector, queue_size=1)
        self.rate_ = rospy.Rate(2)
        self.unitary_angle = 0.0
        self.a = 2.5   # radius on the x-axis 
        self.b = 1.0   # radius on the y-axis

    def gen_ellipse_points(self, delta=0.01):
        x = self.a * cos(self.unitary_angle) # x-coord
        y = self.b * sin(self.unitary_angle) # y-coord
        self.unitary_angle += delta

        # after 360 degrees reset to 0
        if self.unitary_angle >= 2*pi: 
            self.unitary_angle = 0.0
        # quadrant locations
        if x >= 0 and y >= 0:
            quadrant = 1
        elif x >= 0 and y < 0:
            quadrant = 2
        elif x < 0 and y < 0:
            quadrant = 3
        elif x < 0 and y > 0:
            quadrant = 4
        else:
            assert False, "ERROR! Not possible!"

        return x, y, quadrant

    def start_loop(self):
        while not rospy.is_shutdown():
            # get new x,y,quadrant points
            x, y, q = self.gen_ellipse_points()

            # msg object
            end_effector_msg = EndEffector()

            # update end effector pose
            end_effector_pose = Vector3()
            end_effector_pose.x = x
            end_effector_pose.y = y
            end_effector_pose.z = self.unitary_angle

            if q == 1 or q == 4:
                elbow_config = "down"
            else:
                elbow_config = "up"

            end_effector_msg.ee_xy_theta = end_effector_pose
            end_effector_msg.elbow_policy.data = elbow_config
            print("Elipse Point=\n"+str(end_effector_pose)+", \nelbow="+str(elbow_config))

            self.publisher.publish(end_effector_msg)
            self.rate_.sleep()

def main():
    rospy.init_node('circular_motion_ee')

    end_effector_obj = EE_Client()
    try:
        end_effector_obj.start_loop()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()