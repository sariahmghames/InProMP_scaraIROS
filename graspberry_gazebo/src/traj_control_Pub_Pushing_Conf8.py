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
    # config7 is when 1 neighbor are below the goal and at an angle
    with open('../../graspberry_planning/src/promp/src/GRASPberry_Config8_PushingWpts.npz', 'r') as f0:
        wpts = np.load(f0)
        wpts = np.squeeze(wpts)
        wpts_cond = wpts[0]
        wpts_Ncond = wpts[1]
        l_cond = len(wpts_cond)
        l_Ncond = len(wpts_Ncond)


	joints_condtraj = []
	J0c = np.zeros(l_cond)
	J1c = np.zeros(l_cond)
	J2c = np.zeros(l_cond)
	for ind in range(l_cond):
		xc = wpts_cond[ind][0]
		print('xc= ', xc)
		yc = wpts_cond[ind][1]
		zc = wpts_cond[ind][2]
		joints_condtraj.append(PRR_kin.PRR_IK1(xc, yc, zc))

	print('joint_condtraj=', joints_condtraj)
	print('joint_condtraj shape', len(joints_condtraj))
	for ind in range(len(joints_condtraj)):
		joint_condtraj = np.expand_dims(joints_condtraj[ind], axis = 1)
		J0c[ind] =  joint_condtraj[0]
		J1c[ind] =  joint_condtraj[1]
		J2c[ind] =  joint_condtraj[2]

	print('J0c=', J0c)
    with open('../../graspberry_planning/src/promp/src/GRASPberry_Config8_Pushing.npz', 'r') as f1:
        Q1 = np.load(f1)
        Q1 = np.squeeze(Q1)
        print('Q1 shape=', Q1.shape)
        print('Q1-99 =', Q1[:,99]) # the cam2 position
        print('Q1-100 =', Q1[:,100])
        print('Q1-101 =', Q1[:,101])


    for l in range(l_cond):
        wpt = wpts_cond[l]
        print('wpt=', wpt)
        print('wpt transp=', np.transpose(np.asarray(wpt)))
        ind_wpt = np.where(np.sum(np.abs(Q1.transpose() - np.asarray(wpt)), axis=-1).any() <= 0.1 and np.sum(np.abs(Q1.transpose() - np.asarray(wpt)), axis=-1).any() >= -0.1)
        print('ind wpt cond =', ind_wpt)

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




