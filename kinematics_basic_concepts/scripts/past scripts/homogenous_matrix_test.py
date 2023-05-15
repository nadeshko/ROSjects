#!/usr/bin/env python3

import math
from sympy import *
from sympy.interactive import printing

# To make display prety
printing.init_printing(use_latex = True)

theta_i = Symbol("theta_i")
alpha_i = Symbol("alpha_i")
r_i = Symbol("r_i")
d_i = Symbol("d_i")

DH_Matric_Generic = Matrix([[cos(theta_i), -sin(theta_i)*cos(alpha_i), sin(theta_i)*sin(alpha_i), r_i*cos(theta_i)],
                            [sin(theta_i), cos(theta_i)*cos(alpha_i), -cos(theta_i)*sin(alpha_i), r_i*sin(theta_i)],
                            [0, sin(alpha_i), cos(alpha_i), d_i],
                            [0,0,0,1]])

result_simpl = simplify(DH_Matric_Generic)

preview(DH_Matric_Generic, viewer='file', filename="DH_Matric_Generic.png", dvioptions=['-D','300'])
preview(result_simpl, viewer='file', filename="result_simpl.png", dvioptions=['-D','300'])

theta1 = Symbol("theta_1")
theta2 = Symbol("theta_2")
theta3 = Symbol("theta_3")
r1 = Symbol("r_1")
r2 = Symbol("r_2")
r3 = Symbol("r_3")

A_01 = Matrix([[cos(theta1), -sin(theta1), 0, r1*cos(theta1)],
            [sin(theta1), cos(theta1), 0, r1*sin(theta1)],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])

A_12 = Matrix([[cos(theta2), -sin(theta2), 0, r2*cos(theta2)],
            [sin(theta2), cos(theta2), 0, r2*sin(theta2)],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])

A_23 = Matrix([[cos(theta3), -sin(theta3), 0, r3*cos(theta3)],
            [sin(theta3), cos(theta3), 0, r3*sin(theta3)],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])

preview(A_01, viewer='file', filename="A_01.png", dvioptions=['-D','300'])
preview(A_12, viewer='file', filename="A_12.png", dvioptions=['-D','300'])
preview(A_23, viewer='file', filename="A_23.png", dvioptions=['-D','300'])

A_03 = A_01 * A_12 * A_23

preview(A_03, viewer='file', filename="A_03.png", dvioptions=['-D','300'])

A_03_simplify = simplify(A_03)

preview(A_03_simplify, viewer='file', filename="A_03_simplify.png", dvioptions=['-D','300'])


"""
alha_value = math.pi/2.0
beta_value = math.pi/2.0
gamma_value = math.pi/2.0

TM_subs = TM.subs(alpha,alha_value).subs(beta,beta_value).subs(gamma,gamma_value)

preview(TM_subs, viewer='file', filename="TM_subs.png", dvioptions=['-D','300'])"""