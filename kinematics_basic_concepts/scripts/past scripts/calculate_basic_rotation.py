#!/usr/bin/env python3

import math
from sympy import *

alpha = Symbol("alpha")
beta = Symbol("beta")
gamma = Symbol("gamma")

R_z = Matrix([[cos(alpha), -sin(alpha), 0],
          [sin(alpha), cos(alpha), 0],
          [0, 0, 1]])

R_y = Matrix([[cos(beta), 0, sin(beta)],
          [0, 1, 0],
          [-sin(beta), 0, cos(beta)]])

R_x = Matrix([[1, -0, 0],
          [0, cos(gamma), -sin(gamma)],
          [0, sin(gamma), cos(gamma)]])

preview(R_z, viewer='file', filename="R_z.png", dvioptions=['-D','300'])
preview(R_y, viewer='file', filename="R_y.png", dvioptions=['-D','300'])
preview(R_x, viewer='file', filename="R_x.png", dvioptions=['-D','300'])

TM = R_z * R_y * R_x

preview(TM, viewer='file', filename="TM.png", dvioptions=['-D','300'])

alha_value = math.pi/2.0
beta_value = math.pi/2.0
gamma_value = math.pi/2.0

TM_subs = TM.subs(alpha,alha_value).subs(beta,beta_value).subs(gamma,gamma_value)

preview(TM_subs, viewer='file', filename="TM_subs.png", dvioptions=['-D','300'])