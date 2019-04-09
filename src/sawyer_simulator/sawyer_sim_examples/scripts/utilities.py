#!/usr/bin/env python

"""
SDK Force Control
"""
"""
Authors: Ernesto Solanes and Luis Gracia
Year:2017
"""
import rospy
import threading

import math
from numpy import *
import numpy as np #import math library
from tf.transformations import euler_from_quaternion
#from tf.transformations import quaternion_from_euler
from tf.transformations import quaternion_matrix
from numpy.linalg import norm

#from hrl_geom.transformations import quaternion_from_euler
from operator import itemgetter

def make_cmd(joints, _cmd):
    return dict([(joint, _cmd.item(i))
                  for i, joint in enumerate(joints)])

def make_cmd_mod(joints, _cmd, vel):
    return dict([(joint, _cmd.item(i)+vel[joint])
                  for i, joint in enumerate(joints)])

def DenHartStandard(limb,joints):
    
    ang = [limb.joint_angle(joints[0]),limb.joint_angle(joints[1]),limb.joint_angle(joints[2]),\
           limb.joint_angle(joints[3]),limb.joint_angle(joints[4]),limb.joint_angle(joints[5]),\
           limb.joint_angle(joints[6])]

    #DH (theta_i,d_i,a_i,alpha_i)
    DH = [[ ang[0],                       0.317-0.0011,   0.081, -1.571],
          [ ang[1] + math.pi/2,           0.1925,         0.0,    1.571],
          [ ang[2] + math.pi,             0.4,            0.0,    1.571],
          [ ang[3],                      -0.1685,         0.0,   -1.571],
          [ ang[4] + math.pi,             0.4,            0.0,   -1.571],
          [ ang[5],                       0.1363,         0.0,    1.571],
          [ ang[6] - math.pi/2-0.1741,    0.13375,        0.0,    0.0  ],
         ]
    return DH

def DenHartStandard_mod(ang):

    #DH (theta_i,d_i,a_i,alpha_i)
    DH = [[ ang[0],                       0.317-0.0011,   0.081, -1.571],
          [ ang[1] + math.pi/2,           0.1925,         0.0,    1.571],
          [ ang[2] + math.pi,             0.4,            0.0,    1.571],
          [ ang[3],                      -0.1685,         0.0,   -1.571],
          [ ang[4] + math.pi,             0.4,            0.0,   -1.571],
          [ ang[5],                       0.1363,         0.0,    1.571],
          [ ang[6] - math.pi/2-0.1741,    0.13375,        0.0,    0.0  ],
         ]
    return DH

def MassParam_Sawyer():
    
    M = np.zeros((7,1))
    M[0] = 5.3213
    M[1] = 4.505  
    M[2] = 1.745
    M[3] = 2.5097
    M[4] = 1.1136
    M[5] = 1.5625
    M[6] = 0.3292
    
    return M

def InertiaParam_Sawyer():

    I = np.zeros((7,3,3))
    
    #Link 0
    ixx = [0.05331,     0.0224,      0.02551,       0.01016,        0.01356,      0.004733,   0.0002155]
    ixy = [0.01173,    -0.0002936,   1.495E-05,     0.0002662,      0.0001352,    4.627E-05,  1.477E-06]
    ixz = [0.004709,   -0.0002399,   4.41E-06,      9.745E-06,      1.811E-05,    0.0001153, -8.453E-06]
    iyy = [0.02366,     0.0173,      0.003418,      0.006908,       0.001366,     0.003176,   0.000311 ]
    iyz = [0.008018,   -0.006088,   -0.00332,       0.003032,       0.001056,    -0.001156,  -3.707E-07]
    izz = [0.0579,      0.01461,      0.0253,       0.006568,       0.01355,      0.002968,   0.0003598]
    
    for i in range(0,7,1):
        I[i][0][0] = ixx[i]
        I[i][0][1] = ixy[i] 
        I[i][0][2] = ixz[i]
        I[i][1][0] = ixy[i]
        I[i][1][1] = iyy[i]
        I[i][1][2] = iyz[i]
        I[i][2][0] = ixy[i]
        I[i][2][1] = iyz[i] 
        I[i][2][2] = izz[i]
 
    return I

def CdMParam_Sawyer():

    CdM = np.zeros((7,1,3))

    x = [-0.05707, -0.002748, -0.0001437,   0.004204,    -0.0006548,   0.006016,  0.01043]
    y = [0.09344, -0.04987,  -0.1238,       -0.04116,     0.1406,     -0.03019,   0.0004209]
    z = [0.01118,   0.02677,   -0.01582,    0.02833,      0.005431,    0.02367,  -0.0289]
   
    for i in range(0,7,1):
        CdM[i][0][0] = x[i]
        CdM[i][0][1] = y[i]
        CdM[i][0][2] = z[i]

    return CdM

class ForceSensorClass(object):

      def __init__(self):
          self._event = threading.Event()
          self._msg = None

      def __call__(self, msg):
          self._msg = msg
          self._event.set()
      
      def get_msg(self, timeout = None):
          self._event.wait(timeout)
          return self._msg

def _j_1_T_j(theta,d,a,alpha):
    
    T = [[math.cos(theta),-math.sin(theta)*math.cos(alpha),math.sin(theta)*math.sin(alpha),a*math.cos(theta)],
         [math.sin(theta),math.cos(theta)*math.cos(alpha),-math.cos(theta)*math.sin(alpha),a*math.sin(theta)],
         [0,math.sin(alpha),math.cos(alpha),d],
         [0,0,0,1],
        ]
    
    return T

def fkine(DH):

    T=[[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]]

    for i in range (0,7):
        T[i]=_j_1_T_j(DH[i][0],DH[i][1],DH[i][2],DH[i][3])

    Tf=np.mat(T[0])*np.mat(T[1])*np.mat(T[2])*np.mat(T[3])*np.mat(T[4])*np.mat(T[5])*np.mat(T[6])

    return Tf

def t2r(T):

    try:
       return T[0:3,0:3]
    
    except:
       return np.array(T)[0:3,0:3]

def jacobn(limb,joints,DH):
    
    ang = [limb.joint_angle(joints[0]),limb.joint_angle(joints[1]),limb.joint_angle(joints[2]),\
           limb.joint_angle(joints[3]),limb.joint_angle(joints[4]),limb.joint_angle(joints[5]),\
           limb.joint_angle(joints[6])]

    U = np.mat([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
    J = mat([[],[],[],[],[],[]])

    for i in range (7-1,-1,-1):
 	U = _j_1_T_j(DH[i][0],DH[i][1],DH[i][2],DH[i][3])*U

        d = matrix([[-U[0,0]*U[1,3] + U[1,0]*U[0,3]],\
                    [-U[0,1]*U[1,3] + U[1,1]*U[0,3]],\
                    [-U[0,2]*U[1,3] + U[1,2]*U[0,3]]])

        delta = U[2,0:3].T   # nz  oz  az

        J = concatenate((concatenate((d,delta)),J),1)

    return J

def jacob0(limb,joints,DH):
    
    Jn = jacobn(limb,joints,DH)
    Tn = fkine(DH)
    R = t2r(Tn)
    Jo = concatenate( ( concatenate( (R,zeros((3,3))) ,1) , concatenate( (zeros((3,3)),R) ,1) ))*Jn
    
    #for i in range (0,6,1):
        #Jo[i,3]=-Jo[i,3]
    
    return Jo


def jacobn_rec(Jn,RPY):

    m = [[1,           0,              sin(RPY[1])],\
         [0, cos(RPY[0]), -sin(RPY[0])*cos(RPY[1])],\
         [0, sin(RPY[0]),  cos(RPY[0])*cos(RPY[1])]]

    #print "matr: "
    #print m

    Jn_rec = np.mat(concatenate( (concatenate((np.eye(3), zeros((3,3))),1), concatenate((zeros((3,3)), np.linalg.inv(m)),1))))*np.mat(Jn)

    return Jn_rec

def jacob_end(J_base,Rot):
    
    J = concatenate( ( concatenate( (np.linalg.inv(Rot),zeros((3,3))) ,1) , concatenate( (zeros((3,3)),np.eye(3)) ,1) ))*J_base
   
    print concatenate( ( concatenate( (np.linalg.inv(Rot),zeros((3,3))) ,1) , concatenate( (zeros((3,3)),np.eye(3)) ,1) ))

    return J

def rotx(theta):

    ct = cos(theta)
    st = sin(theta)

    return mat([[1,  0,    0],
            [0,  ct, -st],
            [0,  st,  ct]])

def roty(theta):

    ct = cos(theta)
    st = sin(theta)

    return mat([[ct,   0,   st],
            [0,    1,    0],
            [-st,  0,   ct]])

def rotz(theta):

    ct = cos(theta)
    st = sin(theta)

    return mat([[ct,      -st,  0],
            [st,       ct,  0],
            [ 0,    0,  1]])

def Quat2rpy(quat):
    
    q = [quat[3],quat[0],quat[1],quat[2]]

    m = [[1 - 2*q[2]*q[2] - 2*q[3]*q[3]  ,2*q[1]*q[2] - 2*q[0]*q[3]       ,2*q[1]*q[3] + 2*q[0]*q[2]],\
         [2*q[1]*q[2] + 2*q[0]*q[3]      ,1 - 2*q[1]*q[1] - 2*q[3]*q[3]   ,2*q[2]*q[3] - 2*q[0]*q[1]],\
         [2*q[1]*q[3] - 2*q[0]*q[2]      ,2*q[2]*q[3] + 2*q[0]*q[1]       ,1 - 2*q[1]*q[1] - 2*q[2]*q[2]]]

    return  R2rpy(m)

def R2rpy(m):
    
    rpy = [0,0,0]
    
    if norm(m[2][2])<finfo(float).eps and norm(m[1][2])<finfo(float).eps:
       # singularity
       rpy[0] = 0
       rpy[1] = arctan2(-m[0][2], m[2][2])
       rpy[2] = arctan2(-m[1][0], m[1][1])

    else:
       rpy[0] = arctan2(-m[1][2],m[2][2])
       sr = sin(rpy[0])
       cr = cos(rpy[0])
       rpy[1] = arctan2(m[0][2], cr*m[2][2] - sr*m[1][2])
       rpy[2] = arctan2(-m[0][1], m[0][0])
    
    return rpy

def rpy2R(RPY):

    r = rotx(RPY[0]) * roty(RPY[1]) * rotz(RPY[2])
    
    return r


def rpy2Quat(rpy):

    R = rpy2R(rpy)

    quaternion = np.empty((4, ), dtype=np.float64)
    quaternion[0] = np.sqrt(np.trace(R)+1.0)/2.0

    Lx = R[2,1] - R[1,2]	
    Ly = R[0,2] - R[2,0]	
    Lz = R[1,0] - R[0,1]	

    if R[0,0] >= R[1,1] and R[0,0] >= R[2,2]:
       Lx1 = R[0,0] - R[1,1] - R[2,2] + 1.0
       Ly1 = R[1,0] + R[0,1]			
       Lz1 = R[2,0] + R[0,2]
    elif R[1,1] >= R[2,2]:
       Lx1 = R[1,0] + R[0,1]			
       Ly1 = R[1,1] - R[0,0] - R[2,2] + 1.0	
       Lz1 = R[2,1] + R[1,2]		
    else:
       Lx1 = R[2,0] + R[0,2]			
       Ly1 = R[2,1] + R[1,2]			
       Lz1 = R[2,2] - R[0,0] - R[1,1] + 1.0	

    if Lx >= 0 or Ly >= 0 or Lz >= 0:
       Lx = Lx + Lx1;
       Ly = Ly + Ly1;
       Lz = Lz + Lz1;
    else:
       Lx = Lx - Lx1;
       Ly = Ly - Ly1;
       Lz = Lz - Lz1;

    if np.linalg.norm([Lx,Ly,Lz]) == 0:
       quaternion[0] = 1.0
       quaternion[1] = 0.0
       quaternion[2] = 0.0
       quaternion[3] = 0.0 
    else:
       s = sqrt(1.0-quaternion[0]*quaternion[0])/np.linalg.norm([Lx,Ly,Lz])
       quaternion[1] = s*Lx
       quaternion[2] = s*Ly
       quaternion[3] = s*Lz


    return quaternion


def rne_dh(Q,Qd,Qdd,grav,fext,n,I,Rl,m,n_p):
    
    z0 = np.zeros((3,1))
    z0[2] = 1
    tau =np.zeros((n_p,1,n))
    q =np.zeros((n,1))
    qd =np.zeros((n,1))
    qdd =np.zeros((n,1))
    f = np.zeros((3,1))
    nn = np.zeros((3,1))

    base = np.zeros((4,4))
    base[0][0] = 1
    base[1][1] = 1
    base[2][2] = 1

    for p in range(0,n_p,1):
        for i in range(0,n,1):
            q[i] = Q[i][p]
            qd[i] = Qd[i][p]        
            qdd[i] = Qdd[i][p]

        DH = DenHartStandard_mod(q)

        Fm = np.zeros((n,3,1))
        Nm = np.zeros((n,3,1))
        pstarm = np.zeros((n,3))
        
        Rm = np.zeros((n,3,3))#Size of Rm?  #modify this

        Rb = t2r(base) #define robot base
        Rb = Rb.T
       
        w = np.zeros((3,1))
        wd = np.zeros((3,1))
        vd = Rb.dot(grav)

        # Compute the link rotation matrices     
        for j in range (0,n,1):
            Tj = _j_1_T_j(DH[j][0],DH[j][1],DH[j][2],DH[j][3])
            Rm[j][:][:] = t2r(Tj)
            d = DH[j][1]
            alpha = DH[j][3]
            pstar = np.array([DH[j][2],d*sin(alpha),d*cos(alpha)])[np.newaxis]
            pstarm[j][0] = DH[j][2]
            pstarm[j][1] = d*sin(alpha)
            pstarm[j][2] = d*cos(alpha)
             
        #the forward recursion
        for j in range(0,n,1):
            Rt = Rm[j][:][:].T
            pstar = pstarm[j][:]
            r = Rl[j][:][:]
            wd = Rt.dot(wd + z0*qdd[j] + np.cross(w.T,(z0*qd[j]).T).T)
            w = Rt.dot(w+z0*qd[j])
            vd = (np.cross(wd.T,pstar.T)).T + np.cross(w.T,np.cross(w.T,pstar.T)).T + Rt.dot(vd)
            vhat = np.cross(wd.T,r).T + np.cross(w.T,np.cross(w.T,r)).T + vd   
            F = m[j]*vhat
            N = I[j][:][:].dot(wd) + np.cross(w.T,(I[j][:][:].dot(w)).T).T
            Fm[j][:][:] = F
            Nm[j][:][:] = N   

        #the backward recursion
        for i in range(0,3,1):
            f[i] = fext[i]
            nn[i] = fext[i+3]

        for j in range(n-1,-1,-1):      
            pstar = pstarm[j][:]

            if j == n-1:
               R = np.array([[1,0,0],[0,1,0],[0,0,1]])
            else:
               R = Rm[j+1][:][:]

            r = Rl[j][0][:]
            nn = R.dot(nn + np.cross(((R.T).dot(pstar.T)),f.T).T) + np.cross((pstar + r).T, Fm[j][:][:].T).T + Nm[j][:][:]
            f = R.dot(f) + Fm[j][:][:]
            R = Rm[j][:][:]
            t = (nn.T[0][:]).dot((R.T).dot(z0)) # if Jm and friction, then: + G[j]*G[J]*J[]*qdd[j] - friction[j]
            tau[p][0][j] = t[0]

    return tau.reshape(n_p,n)

def rne_dh_mod(Q,Qd,Qdd,grav,fext,n, I, Rl,m,n_p):
    
    z0 = np.zeros((3,1))
    z0[2] = 1
    tau =np.zeros((n_p,1,n))
    q =np.zeros((n,1))
    qd =np.zeros((n,1))
    qdd =np.zeros((n,1))
    f = np.zeros((3,1))
    nn = np.zeros((3,1))

    base = np.zeros((4,4))
    base[0][0] = 1
    base[1][1] = 1
    base[2][2] = 1

    for p in range(0,n_p,1):
        for i in range(0,n,1):
            q[i] = Q[i][p]
            qd[i] = Qd[i][p]        
            qdd[i] = Qdd[i][p]

        DH = DenHartStandard_mod(q)

        Fm = np.zeros((n,3,1))
        Nm = np.zeros((n,3,1))
        pstarm = np.zeros((n,3))
        
        Rm = np.zeros((n,3,3))#Size of Rm?  #modify this

        Rb = t2r(base) #define robot base
        Rb = Rb.T
       
        w = np.zeros((3,1))
        wd = np.zeros((3,1))
        vd = Rb.dot(grav)

        # Compute the link rotation matrices     
        for j in range (0,n,1):
            Tj = _j_1_T_j(DH[j][0],DH[j][1],DH[j][2],DH[j][3])
            Rm[j][:][:] = t2r(Tj)
            d = DH[j][1]
            alpha = DH[j][3]
            pstar = np.array([DH[j][2],d*sin(alpha),d*cos(alpha)])[np.newaxis]
            pstarm[j][0] = DH[j][2]
            pstarm[j][1] = d*sin(alpha)
            pstarm[j][2] = d*cos(alpha)
             
        #the forward recursion
        for j in range(0,n,1):
            Rt = Rm[j][:][:].T
            pstar = pstarm[j][:]
            r = Rl[j][:][:]
            wd = Rt.dot(wd + z0*qdd[j] + np.cross(w.T,(z0*qd[j]).T).T)
            w = Rt.dot(w+z0*qd[j])
            vd = (np.cross(wd.T,pstar.T)).T + np.cross(w.T,np.cross(w.T,pstar.T)).T + Rt.dot(vd)
            vhat = np.cross(wd.T,r).T + np.cross(w.T,np.cross(w.T,r)).T + vd   
            F = m[j]*vhat
            N = I[j][:][:].dot(wd) + np.cross(w.T,(I[j][:][:].dot(w)).T).T
            Fm[j][:][:] = F
            Nm[j][:][:] = N   

        #the backward recursion
        for i in range(0,3,1):
            f[i] = fext[i]
            nn[i] = fext[i+3]

        for j in range(n-1,-1,-1):      
            pstar = pstarm[j][:]

            if j == n-1:
               R = np.array([[1,0,0],[0,1,0],[0,0,1]])
            else:
               R = Rm[j+1][:][:]

            r = Rl[j][0][:]
            nn = R.dot(nn + np.cross(((R.T).dot(pstar.T)),f.T).T) + np.cross((pstar + r).T, Fm[j][:][:].T).T + Nm[j][:][:]
            f = R.dot(f) + Fm[j][:][:]
            R = Rm[j][:][:]
            t = (nn.T[0][:]).dot((R.T).dot(z0)) # if Jm and friction, then: + G[j]*G[J]*J[]*qdd[j] - friction[j]
            tau[p][0][j] = t[0]

    return tau.reshape(n_p,n)

def inertiaMatrix(q,n,I,Rl,m):

    #q it has to be a row vectos
    n_p=n    

    M = np.zeros((n,n))
    Q = np.zeros((n,n_p))
    qd = np.zeros((n,n_p))
    qdd = np.zeros((n,n_p))

    for p in range(0,n,1):
        for k in range(0,n_p,1):
            Q[p][k] = q[p]
            if p == k:
               qdd[p][k] = 1.0

    grav = np.zeros((3,1))
    fext = np.zeros((6,1))

    m = rne_dh_mod(Q,qd,qdd,grav,fext,n, I, Rl,m,n_p)

    return m

def fs_request(ang):

    DH = [[ ang[0],                       0.317-0.0011,   0.081, -1.571],
          [ ang[1] + math.pi/2,           0.1925,         0.0,    1.571],
          [ ang[2] + math.pi,             0.4,            0.0,    1.571],
          [ ang[3],                      -0.1685,         0.0,   -1.571],
          [ ang[4] + math.pi,             0.4,            0.0,   -1.571],
          [ ang[5],                       0.1363,         0.0,    1.571],
          [ ang[6] - math.pi/2-0.1741,    0.13375,        0.0,    0.0  ],
         ]

    U = np.mat([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
    Jn = mat([[],[],[],[],[],[]])

    for i in range (7-1,-1,-1):
 	U = _j_1_T_j(DH[i][0],DH[i][1],DH[i][2],DH[i][3])*U

        d = matrix([[-U[0,0]*U[1,3] + U[1,0]*U[0,3]],\
                    [-U[0,1]*U[1,3] + U[1,1]*U[0,3]],\
                    [-U[0,2]*U[1,3] + U[1,2]*U[0,3]]])

        delta = U[2,0:3].T   # nz  oz  az

        Jn = concatenate((concatenate((d,delta)),Jn),1)
  
    return np.linalg.cond(Jn)

