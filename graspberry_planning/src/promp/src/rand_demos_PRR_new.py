#!/usr/bin/env python
from __future__ import absolute_import, division, print_function

from tempfile import TemporaryFile
from timeit import default_timer as timer
from scipy.interpolate import interp2d
from scipy.interpolate import Rbf
from mpl_toolkits.mplot3d import axes3d
from numpy import empty
from scipy import signal

import roslaunch
import rospy
import scipy
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseArray, Point
from std_msgs.msg import Float32MultiArray, String


MOVE, PAUSE, RESUME, UNLOCK, HOME, RESET = 0, 1, 2, 3, 4, 5
MIN_X, MIN_Y, MIN_Z, MAX_X, MAX_Y, MAX_Z = -440, 0, 0, 440, 440, 510

# Collecting random demos in a single array
overall_demos = []

loop = 0

# draw sphere
def sphere(r):
    #u = np.linspace(0, 2 * np.pi, 50)
    #v = np.linspace(0, np.pi, 50)
    u, v = np.mgrid[0.0:2.0*np.pi:100j, 0.0:np.pi:100j]
    xs = (np.cos(u) * np.sin(v))
    ys = (np.sin(u) * np.sin(v))
    zs = np.cos(v) 
    return xs*r, ys*r, zs*r  
    

def demo1_gen(goal, samples):  # Cubic spline : a Heuristic path planning approach
    global loop
    fig = plt.figure(figsize=(10,6))
    ax = axes3d.Axes3D(fig)
    #print('leng of goal:', len(goal))
    for l in range(len(goal)):
        x_init, y_init, z_init = -440, 0, 0

        X = np.array([x_init, goal[l][0]])
        Y = np.array([y_init, goal[l][1]])
        Z = np.array([z_init, goal[l][2]])


        #fig = plt.figure(figsize=(10,6))
        #ax = axes3d.Axes3D(fig)
        #ax.set_title("End-Effector trajectory Generation for Strawberry Picking")
        ax.scatter3D(X,Y,Z, c='r')
        r1 = 50
        xs, ys, zs = sphere(r1)
        ax.plot_surface(xs+goal[l][0], ys+goal[l][1], zs+goal[l][2], rstride=1, cstride=1, color="r", alpha=0.3, linewidth=0)


        rbfi = scipy.interpolate.Rbf(X,Y,Z,function='cubic',smooth=5, episilon=5)  # radial basis function 

        X_new = np.linspace(x_init, goal[l][0], 162)
        Y_new = np.linspace(y_init, goal[l][1], 162)
        Z_new = rbfi(X_new, Y_new)

        ax.plot(X_new, Y_new, Z_new)


        Xp = X_new.flatten()
        Yp = Y_new.flatten()
        Zp = Z_new.flatten()

        traj1 = np.zeros((len(Xp),3))
        for i in range(len(Xp)):
            traj1[i] = np.array([Xp[i], Yp[i], Zp[i]])
        t1_r, t1_c = traj1.shape
        print('t1_r=', t1_r)
        print('t1_c=', t1_c)
        noisy_traj1(traj1, t1_r, t1_c, samples, X_new, Y_new, Z_new)
        #ax.plot(X_new, Y_new, Z_new)
        loop = loop + 1
    ax.set_xlabel('X[mm]', fontsize = 15)
    ax.set_ylabel('Y[mm]', fontsize = 15)
    ax.set_zlabel('Z[mm]', fontsize = 15)
    ax.xaxis._axinfo['label']['space_factor'] = 5.0
    ax.yaxis._axinfo['label']['space_factor'] = 5.0
    ax.zaxis._axinfo['label']['space_factor'] = 5.0
    #ax.set_title("End-Effector random demo based cubic spline trajectory generation for Strawberry Picking")
    plt.show()
    demons_size = len(overall_demos)

    # print each demo size
    onedemo_rows, onedemo_col = overall_demos[0].shape
    np.savez_compressed('./PRR_demos_new', data =overall_demos)

def noisy_sig(mean, std, pts):
    y =  np.random.normal(mean,std,pts)
    yhat = signal.savgol_filter(y,53,3)
    return yhat


def noisy_traj1(traj1, r, c, samples, X_new, Y_new,Z_new): 
    overall_demos.append(traj1)
    for s in range(samples):
        #print('sample:',s)
        clean_subsamples = 8 # to avoid negative values generation for the noisy traj
        noise_X = noisy_sig(0,2,r-clean_subsamples)
        noise_Y = noisy_sig(0,2,r-clean_subsamples)
        noise_Z = noisy_sig(0,2,r-clean_subsamples)
        #noise_X = X_new[r:] 
        #noise_Y = Y_new[r:] 
        #noise_Z = Z_new[r:]
        noise_arr = np.array([noise_X, noise_Y, noise_Z])
        noise_arr = noise_arr.reshape(r-clean_subsamples,3)
        
        clean_traj1 = traj1[:(clean_subsamples),:]
        #print('clean traj= ', clean_traj1)
        noisy_traj1 = traj1[clean_subsamples:, :] + noise_arr

        overall_traj = np.concatenate([clean_traj1, noisy_traj1])
        print('overall traj shape=', overall_traj.shape)
        ind1 = np.where(overall_traj[:, 0] < MIN_X)
        if len(ind1)!= 0:
            overall_traj[ind1,0] = MIN_X
        ind2 = np.where(overall_traj[:, 0] > MAX_X)
        if len(ind2)!= 0:
            overall_traj[ind2,0] = MAX_X
        ind3 = np.where(overall_traj[:, 1] < MIN_Y)
        if len(ind3)!= 0:
            overall_traj[ind3,1] = MIN_Y
        ind4 = np.where(overall_traj[:, 1] > MAX_Y)
        if len(ind4)!= 0:
            overall_traj[ind4,1] = MAX_Y
        ind5 = np.where(overall_traj[:, 2] < MIN_Z)
        if len(ind5)!= 0:
            overall_traj[ind5,2] = MIN_Z
        ind6 = np.where(overall_traj[:, 2] > MAX_Z)
        if len(ind6)!= 0:
            overall_traj[ind6,2] = MAX_Z
        overall_demos.append(overall_traj)
        noisy_traj1_f = overall_traj.transpose()
        plt.plot(noisy_traj1_f[0], noisy_traj1_f[1], noisy_traj1_f[2], label='$sample = {s}$'.format(s=s))


if __name__ == '__main__':
    print('Please enter the strawberry goal position in mm:')
    xg = input()
    yg = input()
    zg = input() 
    goal = np.array([xg, yg, zg]) # -200, 260, 400
    print('Please enter the number of random demos to be generated from a single trajectory:')
    samples = input() # 10
    # more strawberries in a single cluster
    goal1 = goal + np.array([50, 50, 50])
    goal2 = goal + np.array([-50, -50, -50])
    goal3 = goal + np.array([0, 50, 50])
    goal4 = goal + np.array([0, -50, -50])
    goal5 = goal + np.array([50, 50, 0])
    goal6 = goal + np.array([-50, -50, 0])
    goal7 = goal + np.array([50, 0, 50])
    goal8 = goal + np.array([-50, 0, -50])
    goal9 = goal + np.array([100, 0, 0])
    goal10 = goal + np.array([-100, 0, 0])
    goal = np.array([goal, goal1, goal2, goal3, goal4, goal5, goal6, goal7, goal8, goal9, goal10])
    demo1_gen(goal, samples) 















