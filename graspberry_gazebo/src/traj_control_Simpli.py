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
	path, dirs, files = next(os.walk("../../graspberry_planning/src/promp/src/data"))
	file_count = len(files)
	i = 0
	t_traj = 2 
	with open('../../graspberry_planning/src/promp/src/data/GRASPberry_traject{}_task_conditioned.npz'.format(i), 'r') as f1:
		Q1 = np.load(f1)
		Q1 = np.squeeze(Q1)

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
    	J0.append(JJ0[-1]+0.076)
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

        # sub-sampling Joints Trajectory
        wptsJ0 = [JJ0[-40], JJ0[-20], JJ0[-1], J0[-2], J0[-1]]
        wptsJ1 = [JJ0[-40]-3.14, JJ1[-20]- 3.14, JJ1[-1]- 3.14, J1[-2]- 3.14, J1[-1]- 3.14]
        wptsJ2 = [JJ0[-40], JJ2[-20], JJ2[-1], J2[-2], J2[-1]]
        wpts = np.concatenate((wptsJ0, wptsJ1, wptsJ2), axis= None)

        # listoflists0 = []
        # listoflists0.append(wptsJ0)
        # listoflists0.append(wptsJ1)
        # listoflists0.append(wptsJ2)
        # Joints1Traj.data = listoflists0
        # print('traj pos shape=', len(listoflists0))
        # print('traj posJ0 shape=', len(listoflists0[0]))
        # print('traj posJ01 =', listoflists0[0][0])
        # print('traj posJ02 =', listoflists0[0][1])
        # print('traj posJ03 =', listoflists0[0][2])
        # print('traj posJ04 =', listoflists0[0][3])
        # print('traj posJ05 =', listoflists0[0][4])
        Joints0Traj = Float64MultiArray()
        Joints1Traj = Float64MultiArray()
        Joints2Traj = Float64MultiArray()
        pub0 = rospy.Publisher('get_Joint0Traj', Float64MultiArray, queue_size=1000)
        #pub1 = rospy.Publisher('get_Joint1Traj', Float64MultiArray, queue_size=10)
        #pub2 = rospy.Publisher('get_Joint2Traj', Float64MultiArray, queue_size=10)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            Joints0Traj.data = wpts
           #Joints1Traj.data = wptsJ1
            #Joints2Traj.data = wptsJ2
            pub0.publish(Joints0Traj)
            #pub1.publish(Joints1Traj)
            #pub2.publish(Joints2Traj)
            rate.sleep()


if __name__ == '__main__':
	# Initialize the node
	try:
		rospy.init_node('JointTrajectory_publisher')
		get_JointTraj()
	except rospy.ROSInterruptException:
		pass




