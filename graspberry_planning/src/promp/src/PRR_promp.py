import numpy as np
import phase as phase
import basis as basis
import promps as promps
import matplotlib.pyplot as plt
import tf.transformations as tf_tran
import PRR_kinematics
import clustering
import formulations as form
from mpl_toolkits.mplot3d import Axes3D
from numbers import Number
from keyboard import press



PRR_kin = PRR_kinematics.PRRkinematics()


Xee = list()
Yee = list()
Zee = list()

coord_s1 = []
tf = []  # create a list()
X_straw = [-200, 260, 300] # mm , to b input from Raymond detection package --> in thorvald /base_link frame


# cd to promp folder
with open('./PRR_demos.npz', 'r') as f:
    Q = np.load(f) # Q shape is (121, 162, 7), 3D array, i = 121 is demo length, 162 is samples length and 7 is dof length
    #print(Q['data'])
    Q = Q['data']
    print('Q length:',len(Q))

with open('./100demos.npz', 'r') as f:
    data = np.load(f)
    time = data['time']  # Q shape is (121, 162, 7), 3D array, i = 121 is demo length, 162 is samples length and 7 is dof length
    print(len(time))
    print('t:',time.shape)

time_104 = time[len(time)-1]
#time_104 = np.array([time_104])
diff = len(Q)-len(time)
for i1 in range(len(time)):
	tf.append(time[i1])

time_104 = np.repeat(np.array([time_104]), diff, axis = 0)
for i2 in range(len(time_104)):
	tf.append(time_104[i2])

print('Q=', Q.shape)
sdemo = Q.shape[0]
#sdemo = np.squeeze(sdemo)
ssamples = Q.shape[1]
print('ssamples', ssamples)
sdof = Q.shape[2]
print('sdof', sdof)
print('time len=', len(tf))

## random time vector, since we didn't collected data
tff = np.linspace(0,1, 162)
tff = np.repeat(np.array([tff]), sdemo, axis = 0)
print('tff', tff.shape)

Xj_s1 = np.zeros((sdemo, ssamples))
Yj_s1 = np.zeros((sdemo, ssamples))
Zj_s1 = np.zeros((sdemo, ssamples))
Xj_s2 = np.zeros((sdemo, ssamples))
Yj_s2 = np.zeros((sdemo, ssamples))
Zj_s2 = np.zeros((sdemo, ssamples))

# store not valid solutions
nValid_sol1 = []
nValid_sol2 = []

################################################
#To plot demonstrated end-eff trajectories
def plotEE():
    fig1 = plt.figure()
    ax = fig1.add_subplot(111, projection='3d')
    for i in range(0,len(Q)): # demos, 
        endEffTraj = Q[i] # 1 demo
        Xee.append(endEffTraj[:,0])
        Yee.append(endEffTraj[:,1])
        Zee.append(endEffTraj[:,2])
        x_ee = endEffTraj[:,0] / 1000
        y_ee = endEffTraj[:,1] / 1000
        z_ee = endEffTraj[:,2] / 1000
        for j in range(ssamples):
        	#print('x_ee[0]=', x_ee[0])
        	#print('y_ee[0]=', y_ee[0])
        	#print('z_ee[0]=', z_ee[0])
        	IK_sol1 = PRR_kin.PRR_IK1(x_ee[j], y_ee[j], z_ee[j])
        	#print('IK_sol1=', IK_sol1)
        	IK_sol2 = PRR_kin.PRR_IK2(x_ee[j], y_ee[j], z_ee[j])
        	#print('IK_sol2=', IK_sol2)
        	valid_sol1 = PRR_kin.IK_validity(IK_sol1)
        	valid_sol2 = PRR_kin.IK_validity(IK_sol2)
       		if valid_sol1 == True:
       			Xj_s1[i,j] = IK_sol1[0]
       			Yj_s1[i,j] = IK_sol1[1]
       			Zj_s1[i,j] = IK_sol1[2]
       		else:		
       			nValid_sol1.append([[x_ee[j]*1000, y_ee[j]*1000, z_ee[j]*1000], [IK_sol1[0], IK_sol1[1], IK_sol1[2]]])
       		if valid_sol2 == True:
       			Xj_s2[i,j] = IK_sol2[0]
       			Yj_s2[i,j] = IK_sol2[1]
       			Zj_s2[i,j] = IK_sol2[2]
       		else:
       			nValid_sol2.append([[x_ee[j]*1000, y_ee[j]*1000, z_ee[j]*1000], [IK_sol2[0], IK_sol2[1], IK_sol2[2]] ])
        ax.scatter(endEffTraj[:,0], endEffTraj[:,1], endEffTraj[:,2], c='b', marker='.') #X, Y , Z
    plt.title('EndEff')
    plt.show()
    fig2 = plt.figure()
    plt.plot(tff, Q[:,2], c='r', marker='.') #X, Y , Z
    plt.title('EndEff xee(t)')
    plt.show()

plotEE()
#print('Xj_sol1:', len(Xj_s1))
print('nValid_sol1', len(nValid_sol1))
print('nValid_sol2', len(nValid_sol2))
#print('nValid_sol1', nValid_sol1[0])
print('nValid_sol2', nValid_sol2[0])

######################################
# To plot demonstrated trajectories Vs time for the scara 3dof arm 
def plotCoord():
	global Xj_s1, Xj_s2
	global Yj_s1, Yj_s2
	global Zj_s1, Zj_s2
	global coord_s1
	coord_s1 = np.array([Xj_s1, Yj_s1, Zj_s1])
	for plotDoF in range(sdof):
		coord1 = coord_s1[plotDoF]
		plt.figure()
		for c in range(len(coord1)):
			plt.plot(coord1[c])
		plt.title('DoF {}'.format(plotDoF))
		plt.show()

plotCoord()


# Get joint trajectories
joint_data = np.transpose(np.array([np.transpose(Xj_s1), np.transpose(Yj_s1), np.transpose(Zj_s1)]))

r_coord, c_coord, d_coord = np.transpose(joint_data).shape
print('r', r_coord)
print('c', c_coord)
print('d', d_coord)

#for h in range(len(tf)):
#	print('time shape', len(tf[h]))


################################################################
phaseGenerator = phase.LinearPhaseGenerator() # generates z = z_dot *time, a constructor of the class LinearPhaseGenerator
basisGenerator = basis.NormalizedRBFBasisGenerator(phaseGenerator, numBasis=5, duration=1, basisBandWidthFactor=3,
                                                   numBasisOutside=1)  # passing arguments of the __init__() method
time_normalised = np.linspace(0, 1, 100)  # 1sec duration 
nDof = 3
plotDof = 1



##################################################
# Conditioning in Task Space (without using the desired euler angles)

learnedProMP = promps.ProMP(basisGenerator, phaseGenerator, nDof)
learner = promps.MAPWeightLearner(learnedProMP) # for learning weights from demos property, it can b implemented by Max likelihood ML or MAP to learn param theta (mu and covMat) 
learner.learnFromData(joint_data, tff)  # get mu omega and sigma omega
mu_theta, sig_theta = learnedProMP.getMeanAndCovarianceTrajectory(np.array([1.0]))  # theta here is not parameter but joint angles, looking for conditioning time t = 1s = Tf,
#print('init mu_theta=', np.squeeze(mu_theta))
#print('init sig_theta=', np.squeeze(sig_theta))
sig_theta = np.squeeze(sig_theta) 
mu_x = np.array([-0.3, 0.23, 0.35]) # yt star in 2.2 ... desired mean of ee position at each dof at t = 1s,  conditioning in task space, at time t = 1s = Tf,
sig_x = np.eye(3) * 0.0000002  # sigma ee position star (sigma y star)
# For tomato picking with franka the orientation desired at ee is always the orientation when franka is strate up , all links aligned, means when all angles are 0, q_home = [0.0, 0.0, 0.0, 0, 0, 0, 0]
q_home = [0.0, 0.0, 0.0] # initial joint pos, to capture the desired ee orientation at t=1s as at t = 0
T_desired = PRR_kin.PRR_TF(q_home)    # ?? should be eye() for no orientation change desired of the ee with respect to the learnt ProMP orientation at t = T = 1s
mu_ang_euler_des = tf_tran.euler_from_matrix(T_desired, 'szyz') # get desired euler angles (ee orientation) at homing position from TF desired, to set it = desired ee orient at t = 1s
sig_euler = np.eye(3) * 0.0002 # cov of desired ee orientation 
post_mean, post_cov = PRR_kin.IK_PRR_graspberry_pose(np.squeeze(mu_theta), sig_theta, mu_x, sig_x, mu_ang_euler_des, sig_euler) # mean and cov of joint config corresponding to the desired/conditioned task pose at t = 1s
taskProMP = learnedProMP.jointSpaceConditioning(1.0, desiredTheta=post_mean, desiredVar=post_cov)  # task conditioned ProMP generated in the joint space, eq (4) with input from eq (5) and (6)
trajectories_task_conditioned = taskProMP.getTrajectorySamples(time_normalised, 20) # get samples from the ProMP in the joint space 
#print('traj cond=', trajectories_task_conditioned.shape) # 100, 3, 20 , 100 is time dim from 0 to 1 , 20 are samples from the ProMP



plt.figure()
plt.plot(time_normalised, trajectories_task_conditioned[:, plotDof, :])
plt.xlabel('time')
plt.title('Joint 1 trajectories from a task-space conditioning')

plt.figure()
plt.plot(time_normalised, trajectories_task_conditioned[:, plotDof+1, :])
plt.xlabel('time')
plt.title('Joint 2 trajectories from a task-space conditioning')

plt.figure()
plt.plot(time_normalised, trajectories_task_conditioned[:, plotDof-1, :])
plt.xlabel('time')
plt.title('Joint 0 trajectories from a task-space conditioning')


##############################################
# Plot of end-effector trajectories
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
for i in range(trajectories_task_conditioned.shape[2]): # shape[2] is time dimension 
    endEffTraj = PRR_kin.endEff_traj(trajectories_task_conditioned[:, :, i])
    ax.scatter(endEffTraj[:, 0], endEffTraj[:, 1], endEffTraj[:, 2], c='b', marker='.')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.title('EndEff Conditioned trajectories')


# ##################################################
# # To save the task conditioned trajectories for playing back on robot

with open('GRASPberry_traject_task_conditioned.npz', 'w') as f:
    np.save(f, trajectories_task_conditioned)

# ##############################################
plt.show()
print('Finished basic framework')



#####################################################################################################################################################################
#																																									#
#																																									#
#####################################################################################################################################################################



# Pushing Strategy (not here):
#-------------------

print('Start conditioning in neighbourhood:')
print('Start conditioning in neighbourhood 1) out of cluster:')
ind_wpt1 = np.where(Q[0,:,2] == Q[0,-1,2] - 50)
#print('ind=', ind_wpt1)
#t_cond_wpt1 = np.round(tff[0,ind_wpt1],2)
t_cond_wpt1 = np.array([0.9])
t_goal = np.array([1.0])
t_cond = [np.squeeze(t_cond_wpt1, axis = 0), t_goal]
#print('t=',np.squeeze(t_cond_wpt1) )
DesiredVar = np.zeros([len(t_cond)*nDof, len(t_cond)*nDof])


mu_theta_wpt1, sig_theta_wpt1 = learnedProMP.getMeanAndCovarianceTrajectory(t_cond_wpt1)
sig_theta_wpt1 = np.squeeze(sig_theta_wpt1) 
mu_x_wpt1 = np.array([mu_x[0], mu_x[1], mu_x[2] - 0.05])
sig_x_wpt1 = sig_x
mean_wpt1, cov_wpt1 = PRR_kin.IK_PRR_graspberry_pose(np.squeeze(mu_theta_wpt1), sig_theta_wpt1, mu_x_wpt1, sig_x_wpt1, mu_ang_euler_des, sig_euler) #  posterior distribution in joint space
DesiredVar[0:3, 0:3] = cov_wpt1
DesiredVar[3:6, 3:6]= post_cov
DesiredTheta = np.reshape(np.array([mean_wpt1, post_mean]), len(t_cond)*nDof)
#print('DesiredTheta=', DesiredTheta)
#print('DesiredVar=', DesiredVar[0:3, 0:3])
#print('DesiredVar2=', DesiredVar[3:6, 3:6])
taskProMP2 = learnedProMP.jointSpaceConditioning(t_cond, desiredTheta= DesiredTheta, desiredVar=DesiredVar)  # task conditioned ProMP generated in the joint space, eq (4) with input from eq (5) and (6)
trajectories_task_conditioned2 = taskProMP2.getTrajectorySamples(time_normalised, 20) # get samples from the ProMP in the joint space 


# Plot of end-effector trajectories
fig2 = plt.figure()
ax2 = fig2.add_subplot(111, projection='3d')
for i in range(trajectories_task_conditioned2.shape[2]): # shape[2] is time dimension 
    endEffTraj2 = PRR_kin.endEff_traj(trajectories_task_conditioned2[:, :, i])
    ax2.scatter(endEffTraj2[:, 0], endEffTraj2[:, 1], endEffTraj2[:, 2], c='g', marker='.')
ax2.set_xlabel('X')
ax2.set_ylabel('Y')
ax2.set_zlabel('Z')
plt.title('EndEff Conditioned trajectories at 2 time steps')
plt.show()

