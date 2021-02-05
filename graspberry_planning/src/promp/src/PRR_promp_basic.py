import numpy as np
import phase as phase
import basis as basis
import promps as promps
import tf.transformations as tf_tran
import matplotlib.pyplot as plt
import PRR_kinematics
#import clustering
#import formulations as form
import request
from mpl_toolkits.mplot3d import Axes3D
from numbers import Number
from keyboard import press
from matplotlib.lines import Line2D



PRR_kin = PRR_kinematics.PRRkinematics()


Xee = list()
Yee = list()
Zee = list()

coord_s1 = []
tf = []  # create a list()

# cd to promp folder
with open('./PRR_demos_new.npz', 'r') as f:
    Q = np.load(f) # Q shape is (121, 162, 7), 3D array, i = 121 is demo length, 162 is samples length and 7 is dof length
    Q = Q['data']
    #print(Q[0])
    print('Q length:',len(Q))

with open('./100demos.npz', 'r') as f:
    data = np.load(f)
    time = data['time']  # Q shape is (121, 162, 7), 3D array, i = 121 is demo length, 162 is samples length and 7 is dof length
    print(len(time))
    print('t:',time.shape)

time_104 = time[len(time)-1]
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
tff1 = np.linspace(0,1, 162)
tff1 = np.repeat(np.array([tff1]), sdemo, axis = 0)

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
          IK_sol1 = PRR_kin.PRR_IK1(x_ee[j], y_ee[j], z_ee[j])
          IK_sol2 = PRR_kin.PRR_IK2(x_ee[j], y_ee[j], z_ee[j])
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
    

plotEE()


# Get joint trajectories
joint_data = np.transpose(np.array([np.transpose(Xj_s1), np.transpose(Yj_s1), np.transpose(Zj_s1)])) # 121 x 162 x 3


################################################################
phaseGenerator = phase.LinearPhaseGenerator() # generates z = z_dot *time, a constructor of the class LinearPhaseGenerator
basisGenerator = basis.NormalizedRBFBasisGenerator(phaseGenerator, numBasis=4, duration=1, basisBandWidthFactor=3,
                                                   numBasisOutside=10)  # passing arguments of the __init__() method, best nb of basis: 4, 5
time_normalised = np.linspace(0, 1, 100)  # 1sec duration 
nDof = 1
plotDof = 1


##################################################
# Learnt promp in Task Space 

learnedProMP1 = promps.ProMP(basisGenerator, phaseGenerator, nDof)
#learnedProMP2 = promps.ProMP(basisGenerator2, phaseGenerator, nDof)
learner1 = promps.MAPWeightLearner(learnedProMP1)  
#learner2 = promps.MAPWeightLearner(learnedProMP2)  

learntTraj1Xee = learner1.learnFromXDataTaskSapce(Q[:,:,0]/1000, tff1)
traj1_Xee = learnedProMP1.getTrajectorySamplesX(time_normalised, 1) # get samples from the ProMP in the joint space 
learntTraj1Yee = learner1.learnFromYDataTaskSapce(Q[:,:,1]/1000, tff1)
traj1_Yee = learnedProMP1.getTrajectorySamplesY(time_normalised, 1) 
learntTraj1Zee = learner1.learnFromZDataTaskSapce(Q[:,:,2]/1000, tff1)
traj1_Zee = learnedProMP1.getTrajectorySamplesZ(time_normalised, 1)

shiftX_Plink1_plate = 0
shiftY_Plink1_plate = 0.0377
shiftZ_Plink1_plate = 0
shiftX_plate_R1link2 = 0.0347
shiftY_plate_R1link2 = 0.067
shiftZ_plate_R1link2 = 0
Zgripper = 0.1016 + 0.01

start = []
IC0 = np.array([-0.44+shiftX_Plink1_plate+shiftX_plate_R1link2, 0+shiftY_Plink1_plate+shiftY_plate_R1link2, 0+Zgripper]) # self.shiftX_plate_R1link2 = 0.0347
start.append(IC0)
mu_x_IC = start[0] 

goal = request.get_goals()
 

#####################################################################################################################################
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(traj1_Xee, traj1_Yee, traj1_Zee, c='b', marker='.')
ax.scatter(mu_x_IC[0], mu_x_IC[1], mu_x_IC[2], s = 100, c='y', marker='o')
ax.scatter(traj1_Xee[-1], traj1_Yee[-1], traj1_Zee[-1], s = 100, c='r', marker='o')
#ax.scatter(traj2_Xee, traj2_Yee, traj2_Zee, s = 100, c='b', marker='.')
#ax.scatter(traj2_Xee[-1], traj2_Yee[-1], traj2_Zee[-1], s = 100, c='r', marker='o')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.title('Task space learnt promp1')
plt.show()

print('len(goal)=', len(goal))

#######################################################################################################################################

# Conditioning at 1st goal point ( same performance if cond at tf comes at end)

for i in range(0,len(goal)):  # len(goal)
  mu_x_tf = goal[i] #goal[i]
  sig_x_tf = np.eye(3) * 0.0
  print('mu_x_tf=', mu_x_tf)
  #### cond at t0
  mu_x_t0 = start[0] 
  sig_x_t0 = np.eye(3) * 0.0
  traj_conditioned_t0 = learnedProMP1.taskSpaceConditioning_Sariah(0, mu_x_t0, sig_x_t0)
  traj_Xee_condt0 = traj_conditioned_t0.getTrajectorySamplesX(time_normalised, 1)
  traj_Yee_condt0 = traj_conditioned_t0.getTrajectorySamplesY(time_normalised, 1)
  traj_Zee_condt0 = traj_conditioned_t0.getTrajectorySamplesZ(time_normalised, 1)

  traj_conditioned_tf = traj_conditioned_t0.taskSpaceConditioning_Sariah(1, mu_x_tf, sig_x_tf)
  traj_Xee_condT = traj_conditioned_tf.getTrajectorySamplesX(time_normalised, 1)
  traj_Yee_condT = traj_conditioned_tf.getTrajectorySamplesY(time_normalised, 1)
  traj_Zee_condT = traj_conditioned_tf.getTrajectorySamplesZ(time_normalised, 1)

  trajectories_task_conditioned = np.array([traj_Xee_condT, traj_Yee_condT,traj_Zee_condT])
  with open('GRASPberry_traject{}_task_conditioned.npz'.format(i), 'w') as f:
    np.save(f, trajectories_task_conditioned)
  if i ==0:
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(traj_Xee_condT, traj_Yee_condT, traj_Zee_condT, c='b', marker='.')
    ax.scatter(mu_x_IC[0], mu_x_IC[1], mu_x_IC[2], s = 100, c='y', marker='o')
    ax.scatter(mu_x_tf[0], mu_x_tf[1], mu_x_tf[2], s = 100, c='r', marker='o')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.title('Task space learnt promp 1')
    plt.show()

print('Finished basic framework')

