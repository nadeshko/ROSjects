#!/usr/bin/env python3
import rospy
from daruma_description.rviz_marker import MarkerBasics
from math import pi

class RotateDarumaSequentialy(object):
    def __init__(self):
        self.position_x = 0.0
        self.position_y = 0.0
        self.position_z = 0.0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        self.markerbasics_object = MarkerBasics()

    def rotate_z(self, angle):
        self.yaw = angle
        self.update_pose()

    def rotate_y(self, angle):
        self.pitch = angle
        self.update_pose()

    def rotate_x(self, angle):
        self.roll = angle
        self.update_pose()

    def update_pose(self):
        self.markerbasics_object.publish_point(self.position_x, 
                                                self.position_y, 
                                                self.position_z,
                                                roll=self.roll,
                                                pitch=self.pitch, 
                                                yaw=self.yaw)

if __name__ == '__main__':
    rospy.init_node('daruma_rotate_sequential', anonymous=True)
    daruma_obj = RotateDarumaSequentialy()

    gamma = pi/2.0
    beta = pi/2.0
    alpha = pi/2.0

    input("Press Enter to Init Daruma in place...")
    daruma_obj.update_pose()

    input("Press Enter to rotate in X axis angle="+str(gamma))
    daruma_obj.rotate_x(gamma)

    input("Press Enter to rotate in Y axis angle="+str(beta))
    daruma_obj.rotate_y(beta)

    input("Press Enter to rotate in Z axis angle="+str(alpha))
    daruma_obj.rotate_z(alpha)