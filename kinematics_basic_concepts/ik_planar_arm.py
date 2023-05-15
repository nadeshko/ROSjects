#!/usr/bin/env python3
from math import pi, sin, cos, atan2, sqrt, pow
import math

class ComputeIK():
    def __init__(self, P_x, P_y, phi_angle, DH_param, elbow_config):
        self.P_x = P_x
        self.P_y = P_y
        self.phi = phi_angle
        self.DH_param = DH_param
        self.elbow_config = elbow_config

    def compute_ik(self):
        r1 = self.DH_param.get('r1')
        r2 = self.DH_param.get('r2')
        r3 = self.DH_param.get('r3')

        X_3x = cos(self.phi)
        X_3y = sin(self.phi)
        U = r3*X_3x
        W = r3*X_3y

        G = (pow(self.P_x, 2) + pow(U, 2) + pow(self.P_y, 2) + pow(W, 2) 
                - 2*(self.P_x*U + self.P_y*W) - pow(r1, 2) - pow(r2, 2)) / (2*r1*r2)
        if (G >= -1 and G <= 1) :
            possible_solution = True
        else:
            possible_solution = False

        if possible_solution == True:
            if self.elbow_config == 'up':
                # negative
                theta_2 = atan2(-1*sqrt(1-pow(G, 2)), G)
            else:
                # positive
                theta_2 = atan2(sqrt(1-pow(G, 2)), G)

        
            theta_1 = atan2((self.P_x - U), (self.P_y - W)) - atan2(r2*sin(theta_2), r1 + r2*cos(theta_2))
            theta_3 = atan2(X_3x, X_3y) - theta_1 - theta_2
            theta_ar = [theta_1, theta_2, theta_3]
      
        return theta_ar, possible_solution

def calculate_ik(Pee_x, Pee_y, chi, DH_parameters, elbow_config = "down"):
    """
    -----INPUTS-----
    Pee_x: X-axis End Effector coordinate
    Pee_y: Y-axis End Effector coordinate
    chi: angle Phi (angle of X3 with X0 frame axis)
    DH_parameters: dictionary containing r_1, r_2, and r_3 (only DH param. required) 
    elbow_config: selection to choose positive/negative theta_2 solution

    -----PURPOSE-----
    Uses the computeIK class
    """
    ik = ComputeIK(Pee_x, Pee_y, chi, DH_parameters, elbow_config)
    theta_ar, possible_solution = ik.compute_ik()
    if possible_solution == True:
        print(f"Possible solution found! Config: {elbow_config}")
        print(f"   theta_1: {theta_ar[0]},\n   theta_2: {theta_ar[1]},\n   theta_3: {theta_ar[2]}")
    else:
        print("No possible solution!")
    
    return theta_ar, possible_solution

if __name__ == '__main__':
    # minimum config
    Pee_x = 1.0
    Pee_y = 0.0
    chi = - pi/2
    r1 = 1.0
    r2 = 1.0
    r3 = 1.0
    DH_parameters = {
        'r1': r1, 
        'r2': r2, 
        'r3': r3}

    calculate_ik(Pee_x, Pee_y, chi, DH_parameters)
    calculate_ik(Pee_x, Pee_y, chi, DH_parameters, "up")

