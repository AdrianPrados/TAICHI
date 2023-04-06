''' Code created by Adrian Prados and Blanca Lopez, 
researchers from RoboticsLab, University Carlos III of Madrid, Spain'''

import math
import sys
import numpy as np
import time
import copy


global Dist_d, Dist_a, Alpha,name



def URModel(name):
    ''' Select the values for the model of the robotic arm'''
    if name == 'ur3' or name =='UR3':
        Dist_d = [0.1519,0,0,0.11235,0.08535,0.0819]
        Dist_a = [0,-0.24365,-0.21325,0,0,0]
        Alpha = [90,0,0,90,-90,0]
    elif name == 'ur5' or name == 'UR5':
        Dist_d = [0.089159,0,0,0.10915,0.09465,0.0823]
        Dist_a = [0,-0.425,-0.39225,0,0,0]
        Alpha = [90,0,0,90,-90,0]
    elif name == 'UR10' or name == 'UR10':
        Dist_d = [0.1273,0,0,0.163941,0.1157,0.0922]
        Dist_a = [0,-0.612,-0.5723,0,0,0]
        Alpha = [90,0,0,90,-90,0]
    else:
        print("Model incorrect")

    return Dist_d, Dist_a,Alpha




def Rot(Theta,Alpha,Dz,Da):
    ''' Generates the rotational matrix for the theta alpha dz and da values'''

    Matriz = [[math.cos(math.radians(Theta)),-math.cos(math.radians(Alpha))*math.sin(math.radians(Theta)),math.sin(math.radians(Alpha))*math.sin(math.radians(Theta)),Da*math.cos(math.radians(Theta))],
    [math.sin(math.radians(Theta)),math.cos(math.radians(Alpha))*math.cos(math.radians(Theta)),-math.sin(math.radians(Alpha))*math.cos(math.radians(Theta)),Da*math.sin(math.radians(Theta))],
    [0,math.sin(math.radians(Alpha)),math.cos(math.radians(Alpha)),Dz],
    [0,0,0,1]
    ]
    #print(type(Matriz))
    Matriz = np.array(Matriz)
    return Matriz

def Cinematica_Inversa(M,name):
    ''' Calculate the inverse kinematics via analytical process taking into account the matrix of the end efector''' 

    Dist_d, Dist_a, Alpha = URModel(name)
    theta = np.zeros((8, 6)) #Save the 8 solutions

    #Theta 1
    T_06 = M
    vec_aux = [[0],[0],[0],[1]]
    P_05 = np.matmul(T_06,[[0],[0],[-Dist_d[5]],[1]]) - vec_aux
    Phi1_1 = math.degrees(math.atan2(math.radians(P_05[1]),math.radians(P_05[0])))
    Sqrt = Dist_d[3]/math.sqrt((P_05[0]**2)+(P_05[1]**2))
    Phi2_1 = math.degrees(math.acos(Sqrt))
    Theta1_1 = Phi1_1 - Phi2_1 + 90
    Theta1_2 = Phi1_1 + Phi2_1 + 90
    theta[0:4, 0] = Theta1_1 
    theta[4:8, 0] = Theta1_2 


    #Theta 5
    
    P_06 = T_06[0:3,3]
    for i in range(2):
        if i == 0:
            Theta1 = Theta1_1
        if i == 1:
            Theta1 = Theta1_2

        Variable = ((P_06[0]*math.sin(math.radians(Theta1)))-(P_06[1]*math.cos(math.radians(Theta1)))-Dist_d[3])/Dist_d[5]
        #print(Variable)
        Theta5_1 = math.degrees(math.acos(Variable))
        Theta5_2 = math.degrees(-math.acos(Variable))
        if i == 0:
            theta[0,4]=Theta5_1
            theta[1,4]=Theta5_1
            theta[2,4]=Theta5_2
            theta[3,4]=Theta5_2
        if i == 1:
            theta[4,4]=Theta5_1
            theta[5,4]=Theta5_1
            theta[6,4]=Theta5_2
            theta[7,4]=Theta5_2
    
    #Theta 6

    for i in [0,2,4,6]:

        T_01 = Rot(theta[i,0],Alpha[0],Dist_d[0],Dist_a[0])
        T_10 = np.linalg.inv(T_01)
        T_16 = np.matmul(T_10,T_06)
        T_61 = np.linalg.inv(T_16)

        z_61 = T_61[0:3,2]
        theta6_phi1 = -z_61[1]/math.sin(math.radians(theta[i,4]))
        theta6_phi2 = z_61[0]/math.sin(math.radians(theta[i,4]))
        Theta6 = math.degrees(math.atan2(theta6_phi1,theta6_phi2))

        theta[i,5] = Theta6
        theta[i+1,5] = Theta6


    #Theta 3

    for i in [0,2,4,6]:
        T_01 = Rot(theta[i,0],Alpha[0],Dist_d[0],Dist_a[0])
        T_10 = np.linalg.inv(T_01)
        T_16 = np.matmul(T_10,T_06)

        T_45 = Rot(theta[i,4],Alpha[4],Dist_d[4],Dist_a[4])
        T_56 = Rot(theta[i,5],Alpha[5],Dist_d[5],Dist_a[5])
        T_46 = np.matmul(T_45,T_56)
        T_64 = np.linalg.inv(T_46)
        T_14 = np.matmul(T_16,T_64)
        d4_aux = [[0],[-Dist_d[3]],[0],[1]]
        vec_aux = [[0],[0],[0],[1]]
        P_13 = np.matmul(T_14,d4_aux)-vec_aux

        P_13_mod = (math.sqrt(P_13[0]**2 + P_13[1]**2 + P_13[2]**2))**2
        cos_val = (P_13_mod - Dist_a[1]**2 - Dist_a[2]**2)/(2*Dist_a[1]*Dist_a[2])

        #print(cos_val)
        if cos_val<0:
            cos_val = max(-1,cos_val)
        elif cos_val>0:
            cos_val = min(1,cos_val)
        
        Theta3_1 = math.degrees(math.acos(cos_val))
        Theta3_2 = -math.degrees(math.acos(cos_val))
        theta[i,2] = Theta3_1
        theta[i+1,2] = Theta3_2


    #Theta 2
    for i in range(8):

        T_01 = Rot(theta[i,0],Alpha[0],Dist_d[0],Dist_a[0])
        T_10 = np.linalg.inv(T_01)
        T_16 = np.matmul(T_10,T_06)

        T_45 = Rot(theta[i,4],Alpha[4],Dist_d[4],Dist_a[4])
        T_56 = Rot(theta[i,5],Alpha[5],Dist_d[5],Dist_a[5])
        T_46 = np.matmul(T_45,T_56)
        T_64 = np.linalg.inv(T_46)
        T_14 = np.matmul(T_16,T_64)
        d4_aux = [[0],[-Dist_d[3]],[0],[1]]
        vec_aux = [[0],[0],[0],[1]]
        P_13 = np.matmul(T_14,d4_aux)-vec_aux

        Phi1_2 = -math.degrees(math.atan2((P_13[1]),(-P_13[0])))
        Theta3 = -theta[i,2]
        Phival = ((-Dist_a[2]*math.sin(math.radians(Theta3)))/(math.sqrt(P_13[0]**2 + P_13[1]**2 + P_13[2]**2)))               
        Phi2_2 = math.degrees(math.asin(Phival))
        Theta2 = Phi1_2 + Phi2_2 
        theta[i,1] = Theta2     

    #Theta 4
    for i in range(8):
        T_01 = Rot(theta[i,0],Alpha[0],Dist_d[0],Dist_a[0])
        T_10 = np.linalg.inv(T_01)
        T_16 = np.matmul(T_10,T_06)

        T_45 = Rot(theta[i,4],Alpha[4],Dist_d[4],Dist_a[4])
        T_56 = Rot(theta[i,5],Alpha[5],Dist_d[5],Dist_a[5])
        T_46 = np.matmul(T_45,T_56)
        T_64 = np.linalg.inv(T_46)
        T_14 = np.matmul(T_16,T_64)

        T_12 = Rot(theta[i,1],Alpha[1],Dist_d[1],Dist_a[1])
        T_23 = Rot(theta[i,2],Alpha[2],Dist_d[2],Dist_a[2])
        T_13 = np.matmul(T_12,T_23)
        T_31 = np.linalg.inv(T_13)
        T_34 = np.matmul(T_31,T_14)
        X_34 = T_34[0:3,0]
        Theta4 = math.degrees(math.atan2(X_34[1],X_34[0]))
        theta[i,3] = Theta4


    ''' Limitation of the rotation between -pi and pi'''
    theta2 = copy.copy(theta)
    for i in range(8):
        for j in range(6):
            aux_theta = theta[i,j]
            rad_theta = math.radians(aux_theta)
            if rad_theta < -math.pi:
                rad_theta = rad_theta + 2*math.pi 
            elif rad_theta > math.pi:
                rad_theta = rad_theta - 2*math.pi
            else:
                rad_theta = rad_theta

            theta2[i,j] = rad_theta

    return theta2


#M = [[-1,0,0,-0.2],[0,-1,0,0],[0,0,1,0.4],[0,0,0,1]]
#M = np.array(sys.argv[1:])

# Esto es para MATLAB

# M = [M[0:4],M[4:8],M[8:12],M[12:16]]
# M = np.array(M)
# M = np.transpose(M)
# print(name)
# Sol = Cinematica_Inversa(M,name)
try:
    if launcher == 'matlab':
        M = [M[0:4],M[4:8],M[8:12],M[12:16]]
        M = np.array(M)
        M = np.transpose(M)
        Sol = Cinematica_Inversa(M,name)
except:
    print('Using Python')



# ----------------- Values to prove ------------------

#M = np.array([[0,-0.588,0.8090,0.179],[-0.629,0.629,0.457,0.024],[-0.777,-0.509,-0.37,-0.229],[0,0,0,1]]) # [pi/2, pi/3, pi/4, pi/5, pi/5, pi/2];

#M = np.array([[0.25,-0.433,0.866,0.161],[-0.433,0.75,0.5,-0.028],[-0.866,-0.500,0,-0.158],[0,0,0,1]]) #[pi/3, pi/3, pi/3, pi/3, pi/3, pi/3];

#M = np.array([[ 1.0000     ,    0     ,    0  , -0.457],[0    ,     0  , -1.0000  ,  -0.194],[0  ,  1.0000     ,    0  ,  0.067],[0    ,     0       ,  0   , 1.0000]])  #[ 0 0 0 0 0 0]

#M = np.array([[ 0   ,      0   , 1.0000   , 0.1940],[1.0000    ,     0     ,    0  ,  0.1280],[0  ,  1.0000     ,    0  , -0.0920],[0,0,0,1]]) #[pi/2, pi/2, pi/2, pi/2, 0, pi/2];

#M = np.array([[ 0  , -1.0000     ,    0   , 0.1120],[1.0000    ,     0    ,     0  ,  0.1280],[ 0     ,    0  ,  1.0000  , -0.0100],[0,0,0,1]]) #[pi/2, pi/2, pi/2, pi/2, pi/2, pi/2];

#M = np.array([[ 0   , 1.0000     ,    0 ,  -0.1120],[-1.0000     ,    0     ,    0  , -0.2990],[ 0     ,    0 ,   1.0000  ,  0.4770],[0,0,0,1]]) #-[pi/2, pi/2, pi/2, pi/2, pi/2, pi/2];

#M = np.array([[ 1.0000   ,      0     ,    0 ,  -0.0300],[0   ,      0  , -1.0000  ,  0.0300],[ 0  ,  1.0000   ,   0  ,  0.2370],[0,0,0,1]]) #-[pi, pi, pi, pi, pi, pi]; o [pi, pi, pi, pi, pi, pi];

#M = np.array([[ 0  ,  1.0000    ,     0  , -0.1580],[0    ,     0 ,  -1.0000  ,  0.0300],[-1.0000    ,     0     ,    0  , -0.0610],[0,0,0,1]]) #-[pi, pi, pi/2, pi, pi, pi];

#M = np.array([[ 0  , -1.0000      ,   0  , -0.3290],[0    ,     0  , -1.0000 ,   0.0300],[1.0000    ,     0     ,    0  ,  0.3650],[0,0,0,1]]) #[pi, pi, pi/2, pi, pi, pi];

#M = np.array([[0.2780  ,  0.7390  , -0.6140  ,  0.1840],[-0.9190  ,  0.3910  ,  0.0530  , -0.2000],[ 0.2790  ,  0.5490  ,  0.7870  ,  0.3730],[0,0,0,1]]) #[5.9363 4.4892 4.2677 6.0280 4.8716 3.8185]

#M = np.array([[0.4441 ,   0.5605  ,  0.6990  , -0.1326],[-0.4828  , -0.5075  ,  0.7137  ,  0.4230],[0.7547  , -0.6545  ,  0.0452  ,  0.4155],[0,0,0,1]]) #[2.3278 3.9106 6.2678 3.2506 6.2236 1.4234]

#M = np.array([[ 0.7591  ,  0.1825 ,  -0.6248  ,  0.1711],[-0.0173  ,  0.9652  ,  0.2609  ,  0.1610],[0.6507  , -0.1872  ,  0.7359  ,  0.2145],[0,0,0,1]]) #[ 1.00319677 -3.81707473  1.58242406  0.82007757  2.30109194  0.51205125]

#Sol = Cinematica_Inversa(M)
#print(Sol)
