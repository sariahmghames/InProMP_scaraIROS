from __future__ import division
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import tf.transformations as tf_tran
from mpl_toolkits import mplot3d
import scipy.optimize as opt
from sympy import symbols, diff, sin, cos, tan, Matrix
from sympy.functions import transpose
from matplotlib import interactive
from sympy import *


class PRRkinematics():

    def __init__(self, ):
        self.ndof = 3
        self.l1 = 0.210 # m
        self.l2 = 0.1407 # m
        self.lg = 0.0893 # m
        self.shiftX_Plink1_plate = 0
        self.shiftY_Plink1_plate = 0.0377
        self.shiftZ_Plink1_plate = 0
        self.shiftX_plate_R1link2 = 0.0347
        self.shiftY_plate_R1link2 = 0.067
        self.shiftZ_plate_R1link2 = 0
        self.Zgripper = 0.1016 + 0.01 # 0.01m to reach the last tip of the gripper
        self.d1 = self.shiftX_Plink1_plate + self.shiftX_plate_R1link2
        self.d2 = self.shiftY_Plink1_plate +  self.shiftY_plate_R1link2
        self.d3 = self.shiftZ_Plink1_plate + self.shiftZ_plate_R1link2
        self.beta_homing = np.pi # from the z prismatic axis, first revolute joint
        self.alpha_homing = np.pi # from the z prismatic axis, second revolute joint
        self.MIN_R1 = 0 # rad
        self.MIN_R2 = 0 # rad
        self.MAX_R1 = np.pi # rad
        self.MAX_R2 = np.pi # rad
        self.MIN_X = - (self.l1 + self.l2 + self.lg + self.d1)  # -440mm (when beta 180 degree and alpha 0)
        self.MIN_Y = 0  # (when beta 0/180 degree and alpha 0/180)
        self.MIN_Z = 0
        self.MAX_X = self.l1 + self.l2 + self.lg + self.d1 # 440mm (when beta 0 degree and alpha 0)   ---- > to consider the offsets in joints ???????
        self.MAX_Y = self.l1 + self.l2 + self.lg + self.d2 # 440mm (when beta 90 degree and alpha 0) 
        self.MAX_Z = 0.51 # m 
        self.alpha_offset = 0.06775 # rad
        self.beta_offset = 0.09397 # rad
        self.numJoints = 3
        self.T_desired = []
        self.IK = []


    def PRR_FK(self,conf):
        L1 = self.l1
        L2 = self.l2
        Lg = self.lg
        x_ee_12 = L1 * np.cos(conf[1]) + (L2+Lg) * np.cos(conf[1]+conf[2])
        y_ee_12 = L1 * np.cos(conf[1]) + (L2+Lg) * np.sin(conf[1]+conf[2])
        z_ee_12 = conf[0]
        T01 = np.array([[1, 0, 0, self.d1], [0, 1, 0, self.d2], [0, 0, 1, self.d3], [0, 0, 0, 1]])
        x_ee = x_ee_12 + self.d1
        y_ee = y_ee_12 + self.d2
        z_ee = z_ee_12 + self.d3 + self.Zgripper # check if -Zgripper
        return [x_ee, y_ee, z_ee]


    def endEff_traj(self, joint_trajectory):
        endEffTrajectory = np.zeros( (joint_trajectory.shape[0], self.ndof) )
        for i in range( joint_trajectory.shape[0]):
            #print('traj= ', joint_trajectory[i, :])
            endEffTrajectory[i, :] = self.PRR_FK(joint_trajectory[i, :])
        return endEffTrajectory


    def joint_traj(self, endEff_traj):
        joint_traj = np.zeros( (endEff_traj.shape[1], self.ndof) )
        #print('endEff_traj.shape[1]=', endEff_traj.shape[1])
        for i in range( endEff_traj.shape[1]):
            #print('traj= ', joint_trajectory[i, :])
            IK = self.PRR_IK1(endEff_traj[0, i], endEff_traj[1, i], endEff_traj[2, i] )
            #print('array IK=', np.asarray(IK).any())

            # if np.isnan(np.asarray(IK).any()):
            #     IK = self.PRR_IK2(endEff_traj[0, i], endEff_traj[1, i], endEff_traj[2, i] )
            #     if np.isnan(np.asarray(IK).any()):
            #         print('IK solution is out of robot workspace')
            joint_traj[i, :] = IK
        return joint_traj


    ## IK of the PRR arm: 2 solutions:1st solution
    def PRR_IK1(self, x_ee, y_ee,z_ee): # those are in Plink1 frame
        L1 = self.l1
        L2 = self.l2
        Lg = self.lg
        x_ee = x_ee - self.d1 # to become in R1link2 frame
        y_ee = y_ee - self.d2
        z_ee = z_ee - self.d3 - self.Zgripper  # check if +Zgripper
        # print('x_ee=', x_ee)
        # print('y_ee=', y_ee)
        # print('z_ee=', z_ee)
        c_beta = (x_ee**2 + y_ee**2 - L1**2 - (L2+Lg)**2) / (2*L1*(L2+Lg)) 
        #c_beta = round(c_beta, 3)
        #print('c_beta=', c_beta)
        # check cos and sin feasability 
        if c_beta >1 or c_beta < 0:
            c_beta = self.SC_sat(c_beta)
        s1_beta = np.sqrt(1-(c_beta)**2)
        #print('s1_beta=', s1_beta)
        beta_s1 = np.arctan2(s1_beta, c_beta)
        #print('beta_s1', beta_s1)
        k1 = L1 + (L2+Lg)*c_beta
        k21 = (L2+Lg)*s1_beta
        alpha_s1 = np.arctan2(y_ee, x_ee) - np.arctan2(k21, k1)
        IK1 = self.IK_sat_sing(z_ee, alpha_s1, beta_s1)
        self.IK = IK1
        return IK1
        

    # IK of the PRR arm: 2 solutions:2nd solution
    def PRR_IK2(self, x_ee, y_ee, z_ee):
        L1 = self.l1
        L2 = self.l2
        Lg = self.lg
        x_ee = x_ee - self.d1
        y_ee = y_ee - self.d2
        z_ee = z_ee - self.d3 - self.Zgripper
        c_beta = (x_ee**2 + y_ee**2 - L1**2 - (L2+Lg)**2) / (2*L1*(L2+Lg)) 
        #print('c_beta=', c_beta)
        c_beta = round(c_beta, 3)
        if c_beta >1 or c_beta < 0:
            c_beta = self.SC_sat(c_beta)
        s2_beta = - np.sqrt(1-(c_beta)**2)
        #print('s2_beta=', s2_beta)
        beta_s2 = np.arctan2(s2_beta, c_beta)
        #print('beta_s2=', beta_s2)
        k1 = L1 + (L2+Lg)*c_beta
        k22 = (L2+Lg)*s2_beta
        alpha_s2 = np.arctan2(y_ee, x_ee) - np.arctan2(k22, k1)
        #print('alpha_s2=', alpha_s2)
        #s2_alpha = (y_ee * k1 - x_ee * k22) / (k1**2 + k22**2)
        #c2_alpha = (y_ee - k1 * s2_alpha) / k22
        IK2 = self.IK_sat_sing(z_ee, alpha_s2, beta_s2)
        return IK2


    def PRR_TF(self, q):
        L1 = self.l1
        L2 = self.l2
        Lg = self.lg
        M1 = q[0]
        alpha = q[1]
        beta = q[2]
        T01 = np.array([[1.0, 0.0, 0.0, self.d1], [0.0, 1.0, 0.0, self.d2], [0.0, 0.0, 1.0, M1+self.d3], [0.0, 0.0, 0.0, 1.0]])
        T12 = np.array([[np.cos(alpha), -np.sin(alpha), 0.0, L1* np.cos(alpha)], [np.sin(alpha), np.cos(alpha), 0.0, L1* np.sin(alpha)], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
        T23 = np.array([[np.cos(beta), -np.sin(beta), 0, (L2+Lg)* np.cos(beta)], [np.sin(beta), np.cos(beta), 0.0, (L2+Lg)* np.sin(beta)], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
        return np.dot(np.dot(T01, T12), T23)


    def PRR_TF_sym(self, theta):
        L1 = self.l1
        L2 = self.l2
        Lg = self.lg
        #theta1, theta2, theta3 = symbols('theta1 theta2 theta3', real=True)
        M1 = theta[0]
        alpha = theta[1]
        beta = theta[2]
        T01 = np.array([[1.0, 0.0, 0.0, self.d1], [0.0, 1.0, 0.0, self.d2], [0.0, 0.0, 1.0, M1+ self.d3], [0.0, 0.0, 0.0, 1.0]])
        T12 = np.array([[np.cos(alpha), -np.sin(alpha), 0, L1* np.cos(alpha)], [np.sin(alpha), np.cos(alpha), 0.0, L1* np.sin(alpha)], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
        T23 = np.array([[np.cos(beta), -np.sin(beta), 0, (L2+Lg)* np.cos(beta)], [np.sin(beta), np.cos(beta), 0.0, (L2+Lg)* np.sin(beta)], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
        return np.dot(np.dot(T01, T12), T23)

              

    def analytic_jacobian(self, theta):
        F1 = self.l1 * np.cos(theta[1]) + (self.l2+self.lg) * np.cos(theta[1] + theta[2]) 
        F2 = self.l1 * np.sin(theta[1]) + (self.l2 + self.lg) * np.sin(theta[1] + theta[2]) 
        F3 = theta[0]
        J11 = 0.0 
        J12 = - self.l1 * np.sin(theta[1]) - (self.l2+self.lg) * np.sin(theta[1] + theta[2])
        J13 = - (self.l2+self.lg) * np.sin(theta[1] + theta[2])
        J21 = 0.0
        J22 = self.l1 * np.cos(theta[1]) + (self.l2+self.lg) * np.cos(theta[1] + theta[2])
        J23 = (self.l2+self.lg) * np.cos(theta[1] + theta[2])
        J31 = 1.0
        J32 = 0.0 
        J33 = 0.0
        jac_th = np.array([[J11, J12, J13], [J21, J22, J23], [J31, J32, J33]])  # add F3
        return jac_th 


    # IK validity
    def IK_validity(self, IK):
        dof1 = IK[0]
        dof2 = IK[1]
        dof3 = IK[2]
        return((self.MIN_Z <= dof1 <= self.MAX_Z) and (self.MIN_R1 <= dof2 <= self.MAX_R1) and  (self.MIN_R2 <= dof3 <= self.MAX_R2))

    # IK saturation 
    def IK_sat_sing(self, dof1, dof2, dof3):
        if dof1 < self.MIN_Z:
            dof1 = self.MIN_Z

        if dof1 > self.MAX_Z: 
            dof1 = self.MAX_Z

        if dof2 < self.MIN_R1:
            dof2 = self.MIN_R1

        if dof2 > self.MAX_R1:
            dof2 = self.MAX_R1

        if dof3 < self.MIN_R2:
            dof3 = self.MIN_R2

        if dof3 < self.MIN_R2:
            dof3 = self.MIN_R2
        return np.array([dof1, dof2, dof3])


    def SC_sat(self,x):
        if x > 1:
            x = 1
        else:
            x = 0
        return x

    # IK saturation 
    def IK_sat(self, IK):
        dof1 = IK[:,0]
        dof2 = IK[:,1]
        dof3 = IK[:,2]
        if (any(dof1) < self.MIN_Z):
            ind1 = np.where(dof1 < self.MIN_Z)
            dof1[ind1] = self.MIN_Z

        if (any(dof1) > self.MAX_Z): 
            ind2 = np.where(dof1 > self.MAX_Z)
            dof1[ind2] = self.MAX_Z

        if (any(dof2) < self.MIN_R1):
            ind3 = np.where(dof2 < self.MIN_R1)
            dof2[ind3] = self.MIN_R1

        if (any(dof2) > self.MAX_R1):
            ind4 = np.where(dof2 > self.MAX_R1)
            dof2[ind4] = self.MAX_R1 

        if (any(dof3) < self.MIN_R2):
            ind5 = np.where(dof3 < self.MIN_R2)
            dof3[ind5] = self.MIN_R2

        if (any(dof3) > self.MAX_R2):
            ind6 = np.where(dof3 > self.MAX_R2)
            dof3[ind6] = self.MAX_R2 
        return np.array([dof1, dof2, dof3])

    
    def plot_workspace(self,):
        q1 = np.linspace(self.MIN_Z, self.MAX_Z, 30)
        q2 = np.linspace(self.MIN_R1, self.MAX_R1, 30)
        q3 = np.linspace(self.MIN_R2, self.MAX_R2, 30)
        reach_pts = []

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        for i in range(len(q1)):
            for j in range(len(q2)):
                for k in range(len(q3)):
                    Xee = self.PRR_FK(np.array([q1[i], q2[j], q3[k]]))
                    reach_pts.append(Xee)
                    ax.scatter(Xee[0],Xee[1],Xee[2],marker="*")
        ax.set_xlabel('Xee')
        ax.set_ylabel('Yee')
        ax.set_zlabel('Zee')
        ax.set_title('Scara Reachable Space')
        plt.show()
        return reach_pts 


    def laplace_cost_and_grad_pose(self,theta, mu_theta, inv_sigma_theta, mu_x, inv_sigma_x, mu_ang_euler_des,inv_sig_euler):

        T_current = self.PRR_TF_sym(theta)  

        jac_th = self.analytic_jacobian(theta) 
        pos_th = T_current[0:3, 3]
        pos_th = np.squeeze(pos_th)
        jac_pos_th = jac_th[0:3, :]
        diff1 = theta - mu_theta  
        tmp1 = np.dot( inv_sigma_theta, diff1)
        diff2 = pos_th - mu_x
        tmp2 = np.dot( inv_sigma_x, diff2 )


        cost_scara = 0.5 * (np.dot(diff1, tmp1 ) + np.dot( diff2, tmp2 )) 
        grad = tmp1 + np.dot(jac_pos_th.T, tmp2 ) 
        return cost_scara, grad


    def IK_PRR_graspberry_pose(self, mu_theta, sig_theta, mu_x, sig_x, mu_ang_euler_des, sig_euler): 
        inv_sig_theta = np.linalg.inv(sig_theta)
        inv_sig_x = np.linalg.inv( sig_x )
        inv_sig_euler = np.linalg.inv( sig_euler)
        print('starting IK')
        cost_grad = lambda theta: self.laplace_cost_and_grad_pose(theta, mu_theta, inv_sig_theta, mu_x, inv_sig_x,
                                                                   mu_ang_euler_des, inv_sig_euler )  # lambda operator defines the variables of the function, theta is qt or "yt"
        
        print('ending IK')
        cost = lambda theta: cost_grad(theta )[0] 
        grad = lambda theta: cost_grad(theta)[1] 

        res = opt.minimize(cost, mu_theta, method='BFGS', jac=grad)
        #res = opt.minimize( cost, mu_theta, method='BFGS') 
        post_mean = res.x
        tmp = cost(post_mean)
        post_cov = res.hess_inv
        return post_mean, post_cov



if __name__ == "__main__":

    kin = PRRkinematics()

    # workspace range 
    reach_pts = kin.plot_workspace()
    reach_pts = np.squeeze(reach_pts)
    r , c = np.shape(reach_pts)
    max_xee = np.max(reach_pts[:,0])
    indx = np.where(reach_pts[:,0] == np.amax(reach_pts[:,0]))
    max_yee = np.max(reach_pts[:,1])
    indy = np.where(reach_pts[:,1] == np.amax(reach_pts[:,1]))
    max_zee = np.max(reach_pts[:,2])
    indz = np.where(reach_pts[:,2] == np.amax(reach_pts[:,2]))
    print(max_xee)
    print(max_yee)
    print(max_zee)
    print(indx)
    print(indy)
    print(indz)

    # find y corresponding to max_xee
    yxmax = reach_pts[indx,1]
    zxmax = reach_pts[indx,2]
    xymax = reach_pts[indy,0]
    zymax = reach_pts[indy,2]
    print('yxmax:',yxmax)
    print('zxmax:',zxmax)
    print('xymax:',xymax)
    print('zymax:',zymax)

    np.savetxt('reach_pts.csv', reach_pts, delimiter=",")







