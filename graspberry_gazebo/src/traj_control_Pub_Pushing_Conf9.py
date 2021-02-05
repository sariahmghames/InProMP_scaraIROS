#!/usr/bin/env python

import rospy, math, time
#from graspberry_planning import PRR_kinematics 
import PRR_kinematics
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension
import os

PRR_kin = PRR_kinematics.PRRkinematics()

def differ(v, a):
    vJ = []
    aJ = []
    for i in range(len(a)):
        vJ.append(v[i])
        aJ.append(a[i])
    print('len vJ=', len(vJ))
    print('end v=', v[-1])
    vJ.append(v[-1])
    vJ.append(0)
    aJ.append(0)
    aJ.append(0)
    return vJ, aJ


def get_JointTraj():
    i = 2
    t_traj = 2
    wpts_tot = []
    loop = 0
    # config7 is when 1 neighbor are below the goal and at an angle
    with open('../../graspberry_planning/src/promp/src/GRASPberry_Config9_PushingWpts.npz', 'r') as f0:
        wpts = np.load(f0)
        wpts = np.squeeze(wpts)
        wpts_cond = wpts[0]
        wpts_Ncond = wpts[1]
        wpts_notmod = wpts[2]
        goal_update = wpts[3]
        goal = wpts[4]
        l_cond = len(wpts_cond)
        l_Ncond = len(wpts_Ncond)
        l_Nmod = len(wpts_notmod)
        print('l_Nmod=', l_Nmod)
        # arrange cond and not modified in same list
        if l_Nmod != 0:
            for l in range(l_Nmod):
                wpts_tot.append(wpts_cond[loop])
                wpts_tot.append(wpts_notmod[l])
                wpts_tot.append(wpts_cond[loop+1])
                loop += 2

	joints_condtraj = []
	J0c = np.zeros(l_cond + l_Nmod)
	J1c = np.zeros(l_cond+l_Nmod)
	J2c = np.zeros(l_cond+l_Nmod)
	for ind in range(l_cond+l_Nmod):
		xc = wpts_tot[ind][0]
		print('xc= ', xc)
		yc = wpts_tot[ind][1]
		zc = wpts_tot[ind][2]
		joints_condtraj.append(PRR_kin.PRR_IK1(xc, yc, zc))

	print('joint_condtraj=', joints_condtraj)
	print('joint_condtraj shape', len(joints_condtraj))
	for ind in range(len(joints_condtraj)):
		joint_condtraj = np.expand_dims(joints_condtraj[ind], axis = 1)
		J0c[ind] =  joint_condtraj[0]
		J1c[ind] =  joint_condtraj[1]
		J2c[ind] =  joint_condtraj[2]

	ind_wpts = []
	ind_btwWpts = []

	print('J0c=', J0c)
    with open('../../graspberry_planning/src/promp/src/GRASPberry_Config9_Pushing.npz', 'r') as f1:
        Q1 = np.load(f1)
        Q1 = np.squeeze(Q1)
        print('Q1 shape=', Q1.shape)
        print('Q1-99 =', Q1[:,99]) # the cam2 position
        print('Q1-100 =', Q1[:,100])
        print('Q1-101 =', Q1[:,101])

    if l_Nmod >1:
    # find index of cond wpts along the traj
        for l in range(l_Nmod):
            wpt = wpts_notmod[l]
            #ind_wptnotmod = np.where((np.abs(Q1[2,:]- wpt[2]) <= 0.00005) and (np.abs(Q1[2,:]- wpt[2]) <= 0.00005))
            ind_wptnotmod = np.where(np.abs(np.around(Q1[2,:],4)- np.round(wpt[2],4)) == 0)
            print('ind wptnotmod =',ind_wptnotmod)
            ind_wptnotmod = ind_wptnotmod[0][0]
            ind_wpts.append(ind_wptnotmod)
            print('ind wpt cond =',ind_wptnotmod)

        for it in range(0,len(ind_wpts),2):
        	lis = np.linspace(ind_wpts[it], ind_wpts[it+1], 6, endpoint=False, dtype=int)
        	lis = lis.tolist()
        	print('lis=', lis)
        	lis.remove(lis[0])
        	ind_btwWpts.append(lis)

        print('ind btwwpts=',ind_btwWpts)

        # mid wpts between cond ones
        J0mid = JJ0[ind_btwWpts] # remove first one as it should be already accounted for
        J1mid = JJ1[ind_btwWpts]
        J2mid = JJ2[ind_btwWpts]
        print('J0mid=', J0mid)


	#cluster_init = np.transpose(np.array([0, 0.5, 0.41-0.15-0.016]))
	#Q1[:,-1] =  cluster_init
    joint_traj = PRR_kin.joint_traj(Q1)
    JJ0 = joint_traj[:,0]
    JJ0 = JJ0
    JJ1 = joint_traj[:,1]
    JJ1 = JJ1
    JJ2 = joint_traj[:,2]
    JJ2 = JJ2
    J0 = []
    J1 = []
    J2 = []

    for i in range(len(JJ0)):
		J0.append(JJ0[i])
		J1.append(JJ1[i])
		J2.append(JJ2[i])

    J0.append(JJ0[-1])
    J0.append(JJ0[-1])
    J0.append(JJ0[-1]+0.016)
    J0.append(JJ0[-1]+0.066) # 0.016+d(goal, neighbor) = 0.016+0.03
    J1.append(JJ1[-1])
    J1.append(JJ1[-1])
    J1.append(JJ1[-1])
    J1.append(JJ1[-1])
    J2.append(JJ2[-1])
    J2.append(JJ2[-1])
    J2.append(JJ2[-1])
    J2.append(JJ2[-1])
    J0 = [x for x in J0]
    #J1 = [x - np.pi for x in J1]
    l = len(J0)
    dt = float(t_traj)/l

    v0 = np.diff(J0)/dt
    v1 = np.diff(J1)/dt
    v2 = np.diff(J2)/dt
    # accelerations
    a0 = np.diff(v0)/dt
    #print('len a0=', len(a0))
    a1 = np.diff(v1)/dt
    a2 = np.diff(v2)/dt

    vJ0, aJ0 = differ(v0, a0)
    vJ1, aJ1 = differ(v1, a1)
    vJ2, aJ2 = differ(v2, a2)


    if l_Nmod >1:
        # concatenate wpts and mid pts
        for id in range(0,len(J0c), len(J0c)):
        	for lc in range(l_Nmod-1):
        		#J0mid = J0mid[lc] # to adjust later for possible x wpts btw more than 1 pair of cond pts
        		#J1mid = J1mid[lc]
        		#J2mid = J2mid[lc]
        		J0c = np.concatenate((J0c[id], J0c[id+1], J0c[id+2], J0mid, J0c[id+3], J0c[id+4], J0c[id+5]), axis = None)	
        		J1c = np.concatenate((J1c[id], J1c[id+1], J1c[id+2], J1mid, J1c[id+3], J1c[id+4], J1c[id+5]),axis = None)	
        		J2c = np.concatenate((J2c[id], J2c[id+1], J2c[id+2], J2mid, J2c[id+3], J2c[id+4], J2c[id+5]),axis = None)	

    # check if goal is not last point of the traj
    if len(goal_update)!= 0:
        print(' yes goal update')
        J0g, J1g, J2g = PRR_kin.PRR_IK1(goal[0], goal[1], goal[2])
        wptsJ0_afterC = np.array([J0g, JJ0[-1], J0[-2], J0[-1], JJ0[-1], 0])
        wptsJ1_afterC = np.array([J1g-3.14, JJ1[-1]- 3.14, J1[-2]- 3.14, J1[-1]- 3.14, JJ1[-1]- 3.14, 0])
        wptsJ2_afterC = np.array([J2g, JJ2[-1], J2[-2], J2[-1], JJ2[-1], 0])
    else:
        wptsJ0_afterC = np.array([JJ0[-1], J0[-2], J0[-1], JJ0[-1], 0])
        wptsJ1_afterC = np.array([JJ1[-1]- 3.14, J1[-2]- 3.14, J1[-1]- 3.14, JJ1[-1]- 3.14, 0])
        wptsJ2_afterC = np.array([JJ2[-1], J2[-2], J2[-1], JJ2[-1], 0])

    print('J0c[0]=', J0c[0])
    print('JJ0[140]=', JJ0[140])
    J0c = np.concatenate((JJ0[99], JJ0[105], JJ0[110], JJ0[115], JJ0[120], JJ0[130],  JJ0[140], J0c), axis = None)
    J1c = J1c -3.14
    J1c = np.concatenate((JJ1[99]-3.14, JJ1[105]-3.14, JJ1[110]-3.14, JJ1[115]-3.14, JJ1[120]-3.14, JJ1[130]-3.14, JJ1[140]-3.14, J1c), axis = None)
    J2c = np.concatenate((JJ2[99], JJ2[105], JJ2[110], JJ2[115], JJ2[120], JJ2[130], JJ2[140], J2c), axis = None)


    wptsJ0 = np.concatenate((J0c, wptsJ0_afterC), axis = None)
    wptsJ1 = np.concatenate((J1c, wptsJ1_afterC), axis = None)
    wptsJ2 = np.concatenate((J2c, wptsJ2_afterC), axis = None)
    print('len wptsJ0=', len(wptsJ0))

    #wpts = np.concatenate((wptsJ0_afterC, wptsJ1_afterC, wptsJ2_afterC), axis= None)
    wpts = np.concatenate((wptsJ0, wptsJ1, wptsJ2), axis= None)
    print('wpts=', wpts)


    Joints0Traj = Float64MultiArray()
    pub0 = rospy.Publisher('get_Joint0Traj', Float64MultiArray, queue_size=1000)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
    	Joints0Traj.data = wpts
        pub0.publish(Joints0Traj)
        rate.sleep()


if __name__ == '__main__':
	# Initialize the node
	try:
	   rospy.init_node('JointTrajectory_publisher')
	   get_JointTraj()
	except rospy.ROSInterruptException:
		pass




