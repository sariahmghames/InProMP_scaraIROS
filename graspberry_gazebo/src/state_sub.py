#!/usr/bin/env python
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import rospy, math, time
import numpy as np
import matplotlib.pyplot as plt
import PRR_kinematics
from gazebo_msgs.srv import GetLinkState
from gazebo_msgs.msg import LinkState
from sensor_msgs.msg import JointState
from control_msgs.msg import JointControllerState
from control_msgs.msg import FollowJointTrajectoryActionFeedback
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import scipy
from scipy import signal

PRR_kin = PRR_kinematics.PRRkinematics()
config = 2
act_EEtraj_pos = []
des_EEtraj_pos = []
act_EEtraj_vel = []
EEtraj_err = []
XeeA = []
YeeA = []
ZeeA = []
XeeD = [] 
YeeD = []
ZeeD = []
straw19 = []
straw20 = []
straw21 = []
straw22 = []
straw23 = []
straw24 = []
straw25 = []
straw26 = []
straw7 = []
straw8 = []
straw9 = []
Xstr19 = []
Ystr19 = []
Zstr19 = []
Xstr20 = [] 
Ystr20 = []
Zstr20 = []

Xstr21 = []
Ystr21 = []
Zstr21 = []
Xstr22 = [] 
Ystr22 = []
Zstr22 = []
Xstr23 = [] 
Ystr23 = []
Zstr23 = []

Xstr24 = []
Ystr24 = []
Zstr24 = []
Xstr25 = [] 
Ystr25 = []
Zstr25 = []
Xstr26 = [] 
Ystr26 = []
Zstr26 = []

Xstr7 = []
Ystr7 = []
Zstr7 = []
Xstr8 = [] 
Ystr8 = []
Zstr8 = []
Xstr9 = [] 
Ystr9 = []
Zstr9 = []

# Fixed Transformations
X_straw19_R1link2M = 0.53545246
Y_straw19_R1link2M = 0.03619128
Z_straw19_R1link2M = 0.342001206


X_straw20_R1link2M = 0.535498129
Y_straw20_R1link2M = 0.032677457
Z_straw20_R1link2M = 0.2571328319

X_straw19_Plink1M = Y_straw19_R1link2M - PRR_kin.d1
Y_straw19_Plink1M = X_straw19_R1link2M + PRR_kin.d2
Z_straw19_Plink1M = Z_straw19_R1link2M 


X_straw20_Plink1M = Y_straw20_R1link2M - PRR_kin.d1
Y_straw20_Plink1M = X_straw20_R1link2M + PRR_kin.d2
Z_straw20_Plink1M = Z_straw20_R1link2M

X_R1link2M_world = 9.94963734
Y_R1link2M_world = -2.1829798
Z_R1link2M_world = 0.77575

X_R1link2M_Plink1M = PRR_kin.d1
Y_R1link2M_Plink1M = PRR_kin.d2
Z_R1link2M_Plink1M = 0

X_Plink1M_world = X_R1link2M_world + X_R1link2M_Plink1M 
Y_Plink1M_world = Y_R1link2M_world + Y_R1link2M_Plink1M 
Z_Plink1M_world = Z_R1link2M_world - Z_R1link2M_Plink1M 


# with open('../../graspberry_planning/src/promp/src/GRASPberry_Config7_PushingWpts.npz', 'r') as f0:
# 	wpts = np.load(f0)
# 	wpts = np.squeeze(wpts)
# 	wpts_cond = wpts[0]
# 	wpts_Ncond = wpts[1]
# 	wpts_notmod = wpts[2]
# 	goal_update = wpts[3]

# with open('../../graspberry_planning/src/promp/src/GRASPberry_Config7_Pushing.npz', 'r') as f1:
#     Q1 = np.load(f1)
#     Q1 = np.squeeze(Q1)



def joint_state_callback(msg):
	global counter
	global PRR_kin 
	if config == 7:
		store_gazebo_models7()
	elif config == 8:
		store_gazebo_models8()
	elif config == 9:
		store_gazebo_models9()
	elif config == 2:
		store_gazebo_models2()
	#sec = msg.header.stamp.secs
	#act_joint_pos = [msg.feedback.actual.positions[0], msg.feedback.actual.positions[1], msg.feedback.actual.positions[2] ]
	#des_joint_pos = [msg.feedback.desired.positions[0], msg.feedback.desired.positions[1], msg.feedback.desired.positions[2]]
	#act_joint_vel = [msg.feedback.actual.velocities[0], msg.feedback.actual.velocities[1], msg.feedback.actual.velocities[2]]
	#joint_err = [msg.feedback.error.positions[0], msg.feedback.error.positions[1], msg.feedback.error.positions[2]]
	EE_act_pos = PRR_kin.PRR_FK(np.array([msg.feedback.actual.positions[0], msg.feedback.actual.positions[1]+3.14, msg.feedback.actual.positions[2]]))
	EE_des_pos = PRR_kin.PRR_FK(np.array([msg.feedback.desired.positions[0], msg.feedback.desired.positions[1]+3.14, msg.feedback.desired.positions[2]]))
	EE_act_err = PRR_kin.PRR_FK(np.array([msg.feedback.error.positions[0], msg.feedback.error.positions[1]+3.14, msg.feedback.error.positions[2]]))
	act_EEtraj_pos.append(EE_act_pos)
	des_EEtraj_pos.append(EE_des_pos)
	EEtraj_err.append(EE_act_err)
	#rospy.sleep()


def store_gazebo_models7():
    #try:
	client = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
	rospy.wait_for_service('/gazebo/get_link_state')
	#print "straw19 Pose:", client("thorvald_ii::/straw19" , "world" )
	#print "straw20 Pose:", client("thorvald_ii::/straw20" , "world" )
	Dict = {'Links': ["thorvald_ii::/straw19", "thorvald_ii::/straw20"]}
	resp_coordinates19 = client(Dict['Links'][0], "world")
	resp_coordinates20 = client(Dict['Links'][1], "world")
	#print '\n'
	#print 'Status19.success = ', resp_coordinates19.success
	#print '\n'
	#print 'Status20.success = ', resp_coordinates20.success
	#print('linkName19=', Dict['Links'][0])
	#print('linkName20=', Dict['Links'][1])  
	straw19.append([resp_coordinates19.link_state.pose.position.x, resp_coordinates19.link_state.pose.position.y, resp_coordinates19.link_state.pose.position.z])
	straw20.append([resp_coordinates20.link_state.pose.position.x, resp_coordinates20.link_state.pose.position.y, resp_coordinates20.link_state.pose.position.z]) 
	#print("Position de X : " , resp_coordinates19.link_state.pose.position.x)

	#print("Quaternion X : " , resp_coordinates19.link_state.pose.orientation.x)
	#except rospy.ServiceException as e:
		#raise IOError(str(e))
		#rospy.loginfo("Get Link State service call failed:{0}".format(e))


def store_gazebo_models8():
    #try:
	client = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
	rospy.wait_for_service('/gazebo/get_link_state')
	#print "straw21 Pose:", client("thorvald_ii::/straw21" , "world" )
	#print "straw22 Pose:", client("thorvald_ii::/straw22" , "world" )
	#print "straw23 Pose:", client("thorvald_ii::/straw23" , "world" )
	Dict = {'Links': ["thorvald_ii::/straw21", "thorvald_ii::/straw22", "thorvald_ii::/straw23"]}
	resp_coordinates21 = client(Dict['Links'][0], "world")
	resp_coordinates22 = client(Dict['Links'][1], "world")
	resp_coordinates23 = client(Dict['Links'][2], "world")
	#print '\n'
	#print 'Status21.success = ', resp_coordinates21.success
	#print '\n'
	#print 'Status22.success = ', resp_coordinates22.success
	#print '\n'
	#print 'Status23.success = ', resp_coordinates23.success
	straw21.append([resp_coordinates21.link_state.pose.position.x, resp_coordinates21.link_state.pose.position.y, resp_coordinates21.link_state.pose.position.z])
	straw22.append([resp_coordinates22.link_state.pose.position.x, resp_coordinates22.link_state.pose.position.y, resp_coordinates22.link_state.pose.position.z]) 
	straw23.append([resp_coordinates23.link_state.pose.position.x, resp_coordinates23.link_state.pose.position.y, resp_coordinates23.link_state.pose.position.z])
	#print("Position de X : " , resp_coordinates21.link_state.pose.position.x)


def store_gazebo_models9():
    #try:
	client = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
	rospy.wait_for_service('/gazebo/get_link_state')
	#print "straw24 Pose:", client("thorvald_ii::/straw24" , "world" )
	#print "straw25 Pose:", client("thorvald_ii::/straw25" , "world" )
	#print "straw26 Pose:", client("thorvald_ii::/straw26" , "world" )
	Dict = {'Links': ["thorvald_ii::/straw24", "thorvald_ii::/straw25", "thorvald_ii::/straw26"]}
	resp_coordinates24 = client(Dict['Links'][0], "world")
	resp_coordinates25 = client(Dict['Links'][1], "world")
	resp_coordinates26 = client(Dict['Links'][2], "world")
	#print '\n'
	#print 'Status24.success = ', resp_coordinates24.success
	#print '\n'
	#print 'Status25.success = ', resp_coordinates25.success
	#print '\n'
	#print 'Status26.success = ', resp_coordinates26.success
	straw24.append([resp_coordinates24.link_state.pose.position.x, resp_coordinates24.link_state.pose.position.y, resp_coordinates24.link_state.pose.position.z])
	straw25.append([resp_coordinates25.link_state.pose.position.x, resp_coordinates25.link_state.pose.position.y, resp_coordinates25.link_state.pose.position.z]) 
	straw26.append([resp_coordinates26.link_state.pose.position.x, resp_coordinates26.link_state.pose.position.y, resp_coordinates26.link_state.pose.position.z])
	#print("Position de X : " , resp_coordinates24.link_state.pose.position.x)


def store_gazebo_models2():
    #try:
	client = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
	rospy.wait_for_service('/gazebo/get_link_state')
	#print "straw7 Pose:", client("thorvald_ii::/straw7" , "world" )
	#print "straw8 Pose:", client("thorvald_ii::/straw8" , "world" )
	#print "straw9 Pose:", client("thorvald_ii::/straw9" , "world" )
	Dict = {'Links': ["thorvald_ii::/straw7", "thorvald_ii::/straw8", "thorvald_ii::/straw9"]}
	resp_coordinates7 = client(Dict['Links'][0], "world")
	resp_coordinates8 = client(Dict['Links'][1], "world")
	resp_coordinates9 = client(Dict['Links'][2], "world")
	#print '\n'
	#print 'Status7.success = ', resp_coordinates7.success
	#print '\n'
	#print 'Status8.success = ', resp_coordinates8.success
	#print '\n'
	#print 'Status9.success = ', resp_coordinates9.success
	straw7.append([resp_coordinates7.link_state.pose.position.x, resp_coordinates7.link_state.pose.position.y, resp_coordinates7.link_state.pose.position.z])
	straw8.append([resp_coordinates8.link_state.pose.position.x, resp_coordinates8.link_state.pose.position.y, resp_coordinates8.link_state.pose.position.z]) 
	straw9.append([resp_coordinates9.link_state.pose.position.x, resp_coordinates9.link_state.pose.position.y, resp_coordinates9.link_state.pose.position.z])
	#print("Position de X : " , resp_coordinates7.link_state.pose.position.x)
    

def cleanup():
	for act_EE_pos in act_EEtraj_pos:
		XeeA.append(act_EE_pos[0])
		YeeA.append(act_EE_pos[1])
		ZeeA.append(act_EE_pos[2])


	for des_EE_pos in des_EEtraj_pos:
		#des_EE_pos = np.squeeze(np.array([des_EEtraj_pos[desp]]))
		#des_EE_pos =  des_EE_pos.reshape((1, 3))
		XeeD.append(des_EE_pos[0])
		YeeD.append(des_EE_pos[1])
		ZeeD.append(des_EE_pos[2])

	smooth_posD = signal.medfilt(np.array([XeeD, YeeD, ZeeD]), kernel_size=1)
	print('smooth=', smooth_posD.shape)

	l_str19 = len(straw19)
	l_str20 = len(straw20)
	#print('len straw 19=', l_str19)

	if config == 7:
		#for str19 in straw19:
		for s19 in range(0, len(straw19), 5):
			Xstr19.append(-straw19[s19][0] + X_Plink1M_world)
			Ystr19.append(-straw19[s19][1] + Y_Plink1M_world)
			Zstr19.append(straw19[s19][2] - Z_Plink1M_world)
		#for str20 in straw20:
		for s20 in range(0, len(straw20), 5):
			Xstr20.append(-straw20[s20][0] + X_Plink1M_world)
			Ystr20.append(-straw20[s20][1] + Y_Plink1M_world)
			Zstr20.append(straw20[s20][2] - Z_Plink1M_world)

		ind_reach_straw19 = np.where(np.abs(np.around(ZeeA,2)- np.round(Zstr19[0],2)) == 0)
		ind_minreach_straw19 = ind_reach_straw19[0][0]
		ind_maxreach_straw19 = ind_reach_straw19[0][-1]
		Xee_minreach_straw19 = XeeA[ind_minreach_straw19]
		Xee_maxreach_straw19 = XeeA[ind_maxreach_straw19]
		min_reach_straw19 = np.abs(Xstr19[0] - Xee_minreach_straw19)
		max_reach_straw19 = np.abs(Xstr19[0] - Xee_maxreach_straw19)
		print('Zstr19[0] =', Zstr19[0])
		print('ind_reach_straw19 =', ind_minreach_straw19)
		print('Xreach_straw19 =',Xee_minreach_straw19)
		print('min_reach_straw19 =',min_reach_straw19)
		print('max_reach_straw19 =',max_reach_straw19)

		ind_reach_straw20 = np.where(np.abs(np.around(ZeeA,2)- np.round(Zstr20[0],2)) == 0)
		ind_minreach_straw20 = ind_reach_straw20[0][0]
		ind_maxreach_straw20 = ind_reach_straw20[0][-1]
		Xee_minreach_straw20 = XeeA[ind_minreach_straw20]
		Xee_maxreach_straw20 = XeeA[ind_maxreach_straw20]
		min_reach_straw20 = np.abs(Xstr20[0] - Xee_minreach_straw20)
		max_reach_straw20 = np.abs(Xstr20[0] - Xee_maxreach_straw20)
		print('ind_reach_straw20 =',ind_minreach_straw20)
		print('Xreach_straw20 =',Xee_minreach_straw20)
		print('min_reach_straw20 =',min_reach_straw20)
		print('max_reach_straw20 =',max_reach_straw20)


	elif config == 8:
		#for str21 in straw21:
		for s21 in range(0, len(straw21), 5):
			Xstr21.append(-straw21[s21][0] + X_Plink1M_world)
			Ystr21.append(-straw21[s21][1] + Y_Plink1M_world)
			Zstr21.append(straw21[s21][2] - Z_Plink1M_world)

		ind_reach_straw21 = np.where(np.abs(np.around(ZeeA,2)- np.round(Zstr21[0],2)) == 0)
		ind_minreach_straw21 = ind_reach_straw21[0][0]
		ind_maxreach_straw21 = ind_reach_straw21[0][-1]
		Xee_minreach_straw21 = XeeA[ind_minreach_straw21]
		Xee_maxreach_straw21 = XeeA[ind_maxreach_straw21]
		min_reach_straw21 = np.abs(Xstr21[0] - Xee_minreach_straw21)
		max_reach_straw21 = np.abs(Xstr21[0] - Xee_maxreach_straw21)
		print('min_reach_straw21 =',min_reach_straw21)
		print('max_reach_straw21 =',max_reach_straw21)
			
		#for str22 in straw22:
		for s22 in range(0, len(straw22), 5):
			Xstr22.append(-straw22[s22][0] + X_Plink1M_world)
			Ystr22.append(-straw22[s22][1] + Y_Plink1M_world)
			Zstr22.append(straw22[s22][2] - Z_Plink1M_world)

		ind_reach_straw22 = np.where(np.abs(np.around(ZeeA,2)- np.round(Zstr22[0],2)) == 0)
		ind_minreach_straw22 = ind_reach_straw22[0][0]
		ind_maxreach_straw22 = ind_reach_straw22[0][-1]
		Xee_minreach_straw22 = XeeA[ind_minreach_straw22]
		Xee_maxreach_straw22 = XeeA[ind_maxreach_straw22]
		min_reach_straw22 = np.abs(Xstr22[0] - Xee_minreach_straw22)
		max_reach_straw22 = np.abs(Xstr22[0] - Xee_maxreach_straw22)
		print('min_reach_straw22 =',min_reach_straw22)
		print('max_reach_straw22 =',max_reach_straw22)

		#for str23 in straw23:
		for s23 in range(0, len(straw23), 5):
			Xstr23.append(-straw23[s23][0] + X_Plink1M_world)
			Ystr23.append(-straw23[s23][1] + Y_Plink1M_world)
			Zstr23.append(straw23[s23][2] - Z_Plink1M_world)

		ind_reach_straw23 = np.where(np.abs(np.around(ZeeA,2)- np.round(Zstr23[0],2)) == 0)
		ind_minreach_straw23 = ind_reach_straw23[0][0]
		ind_maxreach_straw23 = ind_reach_straw23[0][-1]
		Xee_minreach_straw23 = XeeA[ind_minreach_straw23]
		Xee_maxreach_straw23 = XeeA[ind_maxreach_straw23]
		min_reach_straw23 = np.abs(Xstr23[0] - Xee_minreach_straw23)
		max_reach_straw23 = np.abs(Xstr23[0] - Xee_maxreach_straw23)
		print('min_reach_straw23 =',min_reach_straw23)
		print('max_reach_straw23 =',max_reach_straw23)

	elif config ==9:
		#for str24 in straw24:
		for s24 in range(0, len(straw24), 5):
			Xstr24.append(-straw24[s24][0] + X_Plink1M_world)
			Ystr24.append(-straw24[s24][1] + Y_Plink1M_world)
			Zstr24.append(straw24[s24][2] - Z_Plink1M_world)

		ind_reach_straw24 = np.where(np.abs(np.around(ZeeA,1)- np.round(Zstr24[0],1)) == 0)
		ind_reach_straw24 = np.squeeze(ind_reach_straw24)
		print('ind_reach_straw24 =', ind_reach_straw24.shape)
		print('ind_reach_straw24 =', ind_reach_straw24)
		ind_minreach_straw24 = ind_reach_straw24[0]
		print('ind_minreach_straw24 =', ind_minreach_straw24)
		ind_maxreach_straw24 = ind_reach_straw24[-1]
		Xee_minreach_straw24 = XeeA[ind_minreach_straw24]
		Xee_maxreach_straw24 = XeeA[ind_maxreach_straw24]
		min_reach_straw24 = np.abs(Xstr24[0] - Xee_minreach_straw24)
		max_reach_straw24 = np.abs(Xstr24[0] - Xee_maxreach_straw24)
		print('min_reach_straw24 =',min_reach_straw24)
		print('max_reach_straw24 =',max_reach_straw24)

		#for str25 in straw25:
		for s25 in range(0, len(straw25), 5):
			Xstr25.append(-straw25[s25][0] + X_Plink1M_world)
			Ystr25.append(-straw25[s25][1] + Y_Plink1M_world)
			Zstr25.append(straw25[s25][2] - Z_Plink1M_world)

		ind_reach_straw25 = np.where(np.abs(np.around(ZeeA,2)- np.round(Zstr25[0],2)) == 0)
		#ind_reach_straw25 = np.squeeze(ind_reach_straw25)
		ind_minreach_straw25 = ind_reach_straw25[0][0]
		ind_maxreach_straw25 = ind_reach_straw25[0][-1]
		Xee_minreach_straw25 = XeeA[ind_minreach_straw25]
		Xee_maxreach_straw25 = XeeA[ind_maxreach_straw25]
		min_reach_straw25 = np.abs(Xstr25[0] - Xee_minreach_straw25)
		max_reach_straw25 = np.abs(Xstr25[0] - Xee_maxreach_straw25)
		print('min_reach_straw25 =',min_reach_straw25)
		print('max_reach_straw25 =',max_reach_straw25)

		#for str26 in straw26:
		for s26 in range(0, len(straw26), 5):
			Xstr26.append(-straw26[s26][0] + X_Plink1M_world)
			Ystr26.append(-straw26[s26][1] + Y_Plink1M_world)
			Zstr26.append(straw26[s26][2] - Z_Plink1M_world)


		ind_reach_straw26 = np.where(np.abs(np.around(ZeeA,2)- np.round(Zstr26[0],2)) == 0)
		ind_minreach_straw26 = ind_reach_straw26[0][0]
		ind_maxreach_straw26 = ind_reach_straw26[0][-1]
		Xee_minreach_straw26 = XeeA[ind_minreach_straw26]
		Xee_maxreach_straw26 = XeeA[ind_maxreach_straw26]
		min_reach_straw26 = np.abs(Xstr26[0] - Xee_minreach_straw26)
		max_reach_straw26 = np.abs(Xstr26[0] - Xee_maxreach_straw26)
		print('min_reach_straw26 =',min_reach_straw26)
		print('max_reach_straw26 =',max_reach_straw26)

	
	elif config ==2:
		#for str7 in straw7:
		for s7 in range(0, len(straw7), 5):
			Xstr7.append(-straw7[s7][0] + X_Plink1M_world)
			Ystr7.append(-straw7[s7][1] + Y_Plink1M_world)
			Zstr7.append(straw7[s7][2] - Z_Plink1M_world)

		ind_reach_straw7 = np.where(np.abs(np.around(ZeeA,2)- np.round(Zstr7[0],2)) == 0)
		ind_minreach_straw7 = ind_reach_straw7[0][0]
		ind_maxreach_straw7 = ind_reach_straw7[0][-1]
		Xee_minreach_straw7 = XeeA[ind_minreach_straw7]
		Xee_maxreach_straw7 = XeeA[ind_maxreach_straw7]
		min_reach_straw7 = np.abs(Xstr7[0] - Xee_minreach_straw7)
		max_reach_straw7 = np.abs(Xstr7[0] - Xee_maxreach_straw7)
		print('min_reach_straw7 =',min_reach_straw7)
		print('max_reach_straw7 =',max_reach_straw7)

		for s8 in range(0, len(straw8), 5):
			Xstr8.append(-straw8[s8][0] + X_Plink1M_world)
			Ystr8.append(-straw8[s8][1] + Y_Plink1M_world)
			Zstr8.append(straw8[s8][2] - Z_Plink1M_world)

		ind_reach_straw8 = np.where(np.abs(np.around(ZeeA,2)- np.round(Zstr8[0],2)) == 0)
		ind_minreach_straw8 = ind_reach_straw8[0][0]
		ind_maxreach_straw8 = ind_reach_straw8[0][-1]
		Xee_minreach_straw8 = XeeA[ind_minreach_straw8]
		Xee_maxreach_straw8 = XeeA[ind_maxreach_straw8]
		min_reach_straw8 = np.abs(Xstr8[0] - Xee_minreach_straw8)
		max_reach_straw8 = np.abs(Xstr8[0] - Xee_maxreach_straw8)
		print('min_reach_straw8 =',min_reach_straw8)
		print('max_reach_straw8 =',max_reach_straw8)

		for s9 in range(0, len(straw9), 5):
			Xstr9.append(-straw9[s9][0] + X_Plink1M_world)
			Ystr9.append(-straw9[s9][1] + Y_Plink1M_world)
			Zstr9.append(straw9[s9][2] - Z_Plink1M_world)

		ind_reach_straw9 = np.where(np.abs(np.around(ZeeA,2)- np.round(Zstr9[0],2)) == 0)
		ind_minreach_straw9 = ind_reach_straw9[0][0]
		ind_maxreach_straw9 = ind_reach_straw9[0][-1]
		Xee_minreach_straw9 = XeeA[ind_minreach_straw9]
		Xee_maxreach_straw9 = XeeA[ind_maxreach_straw9]
		min_reach_straw9 = np.abs(Xstr9[0] - Xee_minreach_straw9)
		max_reach_straw9 = np.abs(Xstr9[0] - Xee_maxreach_straw9)
		print('min_reach_straw9 =',min_reach_straw9)
		print('max_reach_straw9 =',max_reach_straw9)

	fig1 = plt.figure()
	ax = fig1.add_subplot(111, projection='3d')
	mpl.rcParams['legend.fontsize'] = 10
	mpl.rcParams['legend.markerscale'] = 1.5
	mpl.rcParams['legend.loc'] = 'best'
	mpl.rcParams['legend.numpoints'] = 1 
	mpl.rcParams['legend.scatterpoints'] = 1 
	ax.scatter(XeeA, YeeA, ZeeA, label= 'Actual Int-ProMP', c='b', alpha = 0.5, marker='*')
	#ax.scatter(smooth_posD[0,:], smooth_posD[1,:], smooth_posD[2,:], label= 'Desired Int-ProMP', c='b', marker='*') 
	if config == 7:
		ax.scatter(Xstr19, Ystr19, Zstr19, label= 'Goal straw', s = 50, c='r', marker='o')
		ax.scatter(Xstr20, Ystr20, Zstr20, label= 'Neighbor straw', c='g', alpha = 0.4,  marker='o')
	elif config == 8:
		ax.scatter(Xstr21, Ystr21, Zstr21, label= 'Goal straw', s = 50, c='r', marker='o')
		ax.scatter(Xstr22, Ystr22, Zstr22, label= 'Neighbor straw', c='g', alpha = 0.4, marker='o')
		ax.scatter(Xstr23, Ystr23, Zstr23, c='g', alpha = 0.5, marker='o')
	elif config == 9:
		ax.scatter(Xstr24, Ystr24, Zstr24, label= 'Neighbor straw', alpha = 0.4, c='g', marker='o')
		ax.scatter(Xstr25, Ystr25, Zstr25, c='g', alpha = 0.5, marker='o')
		ax.scatter(Xstr26, Ystr26, Zstr26, label= 'Goal straw', s = 50, c='r', alpha = 0.5, marker='o')

	elif config == 2:
		ax.scatter(Xstr7, Ystr7, Zstr7, label= 'Ripe goal', c='r', marker='o')
		ax.scatter(Xstr8, Ystr8, Zstr8, c='g', alpha = 0.5, marker='o')
		ax.scatter(Xstr9, Ystr9, Zstr9, label= 'Unripe neighbor', s = 50, c='g', alpha = 0.5, marker='o')


	#ax.scatter([i[0] for i in wpts_cond], [i[1]-0.1047 for i in wpts_cond], [i[2] for i in wpts_cond], label= 'Update Pose', s = 100, c='g', marker='o')
	#for wptnm in wpts_notmod:	
	#	ax.scatter(wptnm[0], wptnm[1]-0.1047, wptnm[2], s = 100, c='r', marker='o')
	#for wptgu in goal_update:	
	#	ax.scatter(wptgu[0], wptgu[1]-0.1047, wptgu[2], s = 100, c='g', marker='o')
	#ax.scatter(Q1[0,99], Q1[1,99]-0.1047, Q1[2,99], label= 'Cam Pose', s = 100, c='c', marker='o')
	#ax.scatter(Q1[0,-1], Q1[1,-1]-0.1047, Q1[2,-1], label= 'Straw Pose', s = 100, c='r', marker='o')
	#ax.text( Q1[0,-1]-0.01,  Q1[1,-1]-0.1047, Q1[2,-1]+0.1, 'Goal', style='italic', weight = 'bold')
	#ax.scatter(np.linspace(Q1[0,-1], Q1[0,-1], 20), np.linspace(Q1[1,-1]-0.1047, Q1[1,-1]-0.1047, 20), np.linspace(Q1[2,-1], Q1[2,-1]+0.1, 20), c='g', marker='.')
	ax.set_xlabel('X')
	ax.set_ylabel('Y')
	ax.set_zlabel('Z')
	#ax.legend(bbox_to_anchor=(-0.5, 0.7, 1., .102))
	#plt.title('Scene Simulation Interactive ProMP with cluster 7')
	plt.show()

	

if __name__ == '__main__':
	#counter = 0
	rospy.init_node('state_listener', anonymous=True)
	sub = rospy.Subscriber('/scara_arm/arm_traj_controller/follow_joint_trajectory/feedback', FollowJointTrajectoryActionFeedback, joint_state_callback)
	rospy.on_shutdown(cleanup)
	rospy.spin()