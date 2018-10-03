import numpy as np 
from numpy import array
from sympy import symbols, cos, sin, pi, simplify, sqrt, atan2
from sympy.matrices import Matrix


### Create symbols for joint variables
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') # theta_i
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')


### KUKA KR210
# DH Parameters
s = {alpha0: 0,     a0: 0,      d1: 0.75,       
     alpha1: -pi/2, a1: 0.35,   d2: 0,      q2: q2-pi/2,
     alpha2: 0,     a2: 1.25,   d3: 0,
     alpha3: -pi/2, a3: -0.054, d4: 1.50,
     alpha4:  pi/2, a4: 0,      d5: 0,
     alpha5: -pi/2, a5: 0,      d6: 0,
     alpha6: 0,     a6: 0,      d7: 0.303,  q7: 0
    }

## IK Orientation Symbolic math
R0_1 = Matrix([[ cos(q1),            -sin(q1),             0],
               [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0)], 
               [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0)]
              ])
R0_1 = R0_1.subs(s)
R1_2 = Matrix([[ cos(q2),            -sin(q2),             0],
               [ sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1)], 
               [ sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1)]
              ])
R1_2 = R1_2.subs(s)
R2_3 = Matrix([[ cos(q3),            -sin(q3),             0],
               [ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2)], 
               [ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2)]
              ])
R2_3 = R2_3.subs(s)
R3_4 = Matrix([[ cos(q4),            -sin(q4),             0],
               [ sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3)], 
               [ sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3)]
              ])
R3_4 = R3_4.subs(s)
R4_5 = Matrix([[ cos(q5),            -sin(q5),             0],
               [ sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4)], 
               [ sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4)]
              ])
R4_5 = R4_5.subs(s)
R5_6 = Matrix([[ cos(q6),            -sin(q6),             0],
               [ sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5)], 
               [ sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5)]
              ])
R5_6 = R5_6.subs(s)



# R0_6 = simplify(R0_1 * R1_2 * R2_3 * R3_4 * R4_5 * R5_6)

R36_LHS = simplify(R3_4 * R4_5 * R5_6)


roll, pitch, yaw = symbols('roll pitch yaw')
R0_3 = simplify(R0_1 * R1_2 * R2_3)

R_z = Matrix([[ cos(np.pi), -sin(np.pi), 0,  0],
              [ sin(np.pi),  cos(np.pi), 0,  0],
              [       0,        0, 1,  0],
              [       0,        0, 0,  1]])
R_y = Matrix([[ cos(-np.pi/2), 0, sin(-np.pi/2),  0],
              [          0, 1, 0,           0],
              [ -sin(-np.pi/2), 0, cos(-np.pi/2), 0],
              [ 0, 0, 0, 1] ])
R_corr = simplify(R_z * R_y)


Rrpy = Rot(Z,yaw) * Rot(Y,pitch) * Rot(X,roll) * R_corr
R36_RHS = inv(R0_3) * Rrpy
