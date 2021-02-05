#!/usr/bin/env python
from __future__ import absolute_import, division, print_function
import numpy as np
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import random
import PRR_kinematics 
import os


PRR_kin = PRR_kinematics.PRRkinematics()

# Reach EndEff desired point
class ScaraSim_RefControl:

	def __init__(self, model, armR_enabled=True, armL_enabled=True, gripperR_enabled=False, gripperL_enabled=False):
		rospy.init_node('graspberry_scara_pathgen')
		self.rate = rospy.Rate(50)
		self.armR_enabled = armR_enabled
		self.armL_enabled = armL_enabled
		self.gripperR_enabled = gripperR_enabled
		self.gripperL_enabled = gripperL_enabled
		self.nameR = []
		self.current_position_armR = []
		self.current_velocity_armR = []
		self.current_effort_armR = []
		self.nameL = []
		self.current_position_armL = []
		self.current_velocity_armL = []
		self.current_effort_armL = []
		self.model = model
		self.data = []
		self.ndof = 3


	def gazebo_command_publisher(self):
		self.response_L1R = rospy.Publisher("/scara_arm/link1_controller/command", Float64, queue_size=10)
		self.response_L2R = rospy.Publisher("/scara_arm/link2_controller/command", Float64, queue_size=10)
		self.response_L3R = rospy.Publisher("/scara_arm/link3_controller/command", Float64, queue_size=10)

		self.response_L1L = rospy.Publisher("/scara_arm/link1M_controller/command", Float64, queue_size=10)
		self.response_L2L = rospy.Publisher("/scara_arm/link2M_controller/command", Float64, queue_size=10)
		self.response_L3L = rospy.Publisher("/scara_arm/link3M_controller/command", Float64, queue_size=10)



	def joint_states_listener(self):

		if self.armR_enabled:
			rospy.loginfo("Get current joint states of right arm")
			self.has_reached_position_armR = False
			self.has_reached_position_message_armR = None
			self.joints_states_listenerR  = rospy.Subscriber("/scara_arm/joint_states", JointState, self.joint_states_callbackR)


		if self.armL_enabled:
			rospy.loginfo("Get current joint states of left arm")
			self.has_reached_position_armL = False
			self.has_reached_position_message_armL = None
			self.joints_states_listenerL  = rospy.Subscriber("/scara_arm/joint_states", JointState, self.joint_states_callbackL)
        
        
	

	def joint_states_callbackR(self, msg):
		#self.lock.acquire()
		self.nameR = msg.name[0:2:4]
		self.current_position_armR = msg.position[0:2:4]
		self.current_velocity_armR = msg.velocity[0:2:4]
		self.current_effort_armR = msg.effort[0:2:4]
		#self.lock.release()


	def joint_states_callbackL(self, msg):
		self.nameL = msg.name[1:2:5]
		self.current_position_armL = msg.position[1:2:5]
		self.current_velocity_armL = msg.velocity[1:2:5]
		self.current_effort_armL = msg.effort[1:2:5]


	def __arm_do_action(self, q1, q2, q3):
		self.gazebo_command_publisher()
		if self.armR_enabled:
			self.response_L1R.publish(q1)
			self.response_L2R.publish(q2)
			self.response_L3R.publish(q3)
			#print('publishing to arm1')

		if self.armL_enabled:
			self.response_L1L.publish(q1)
			self.response_L2L.publish(q2)
			self.response_L3L.publish(q3)
			#print('publishing to arm2')

	def moveR_to_xyz(self, j0, j1, j2, speed, mode):
		print('about to move right arm')
		#qR = self.model.PRR_IK1(x,y,z)
		j0 = np.squeeze(j0)
		j1 = np.squeeze(j1)
		j2 = np.squeeze(j2)
		#print('length j0=', len(j0))
		#print('qR1=', j0)
		#print('qR2=', j1)
		#print('qR3=', j2)
		#self.__arm_do_action(j0, j1, j2) # uncomment to try single desired goal point
		for J in range(len(j0)):
			if (isinstance(mode, int) and mode == 0):
				return self.__arm_do_action(j0[J], j1[J], j2[J])


	def moveL_to_xyz(self, j0, j1, j2, speed, mode):
		print('about to move left arm')
		#qL = self.model.PRR_IK1(x,y,z)
		j0 = np.squeeze(j0)
		j1 = np.squeeze(j1)
		j2 = np.squeeze(j2)
		#print('qL1=', j0)
		#print('qL2=', j1)
		#print('qL3=', j2)
		self.__arm_do_action(j0, j1, j2) # uncomment to try single desired goal point
		# for J in range(len(j0)):
		# 	if (isinstance(mode, int) and mode == 0):
		# 		return self.__arm_do_action(j0[J], j1[J], j2[J])



def scara_reach2grasp():
	#create an object of the scara robot model
	scara_model = PRR_kinematics.PRRkinematics()

	scara_RefControl = ScaraSim_RefControl(scara_model, armR_enabled=False, armL_enabled=True, gripperR_enabled=False, gripperL_enabled=False)


	path, dirs, files = next(os.walk("./data"))
	file_count = len(files)

	for i in range(1): #file_count-1
		with open('./GRASPberry_traject0_task_conditioned.npz', 'r') as f1:
		#with open('./data/GRASPberry_traject{}_task_conditioned.npz'.format(1), 'r') as f1:
			Q1 = np.load(f1)
			Q1 = np.squeeze(Q1)
			Q1 = Q1.transpose()
    		print('Q1 length:',len(Q1))
    		print('Q1 shape:',Q1.shape)
    		print('Xee=', Q1[:,0])
    		print('Yee=', Q1[:,1])
    		print('Zee=', Q1[:,2])

    		joint_traj = PRR_kin.joint_traj(Q1)
    		J0 = joint_traj[:,0]
    		#J0 = np.expand_dims(J0, axis=0)
    		J1 = joint_traj[:,1]
    		#J1 = np.expand_dims(J1, axis=0)
    		J2 = joint_traj[:,2]
    		#J2 = np.expand_dims(J2, axis=0)
    		#scara_RefControl.data.append(np.array([J0, J1, J2]))
    		#scara_RefControl.data.append(np.array([-0.32, -2.36, 0.9]))  # uncomment to try single desired goal point
    		#scara_RefControl.data.append(np.array([0, -2.5, 2])) # to test collision gripper - tabletop
    		jt = PRR_kin.PRR_IK1(0, 0.5, 0.266)
    		#jt = PRR_kin.PRR_IK1(-0.44, 0, 0)
    		print('jt=', jt)
    		scara_RefControl.data.append(np.array([jt[0], +jt[1]-np.pi, jt[2]]))
		
	print('scara_RefControl len=', len(scara_RefControl.data))
	while not rospy.is_shutdown():
		for j in range(len(scara_RefControl.data)):
			traj_d = scara_RefControl.data[j]
			print('traj des=', traj_d)
			print('traj des shape=', traj_d.shape)

			if scara_RefControl.armR_enabled:
				print('p0R=', traj_d[0,0:])
				print('p1R=', traj_d[1,0:])
				print('p2R=', traj_d[2,0:])
				#print('p0=', p[0,0:].shape)
				#scara_RefControl.moveR_to_xyz(traj_d[0,0:], traj_d[1,0:], traj_d[2,0:], speed =20, mode = 0)
				scara_RefControl.moveR_to_xyz(traj_d[0], traj_d[1], traj_d[2], speed =20, mode = 0) # uncomment to try single desired goal point

			if scara_RefControl.armL_enabled:
				#print('p0L=', -traj_d[0,0:])
				#print('p1L=', -traj_d[1,0:])
				#print('p2L=', traj_d[2,0:])
				#scara_RefControl.moveL_to_xyz(-traj_d[0,0:], -traj_d[1,0:], traj_d[2,0:], speed =20, mode = 0)
				scara_RefControl.moveL_to_xyz(traj_d[0], traj_d[1], traj_d[2], speed =20, mode = 0) # uncomment to try single desired goal point
		scara_RefControl.rate.sleep()
	# 	#rospy.spin()




if __name__ == '__main__':
	try:
		scara_reach2grasp()
	except rospy.ROSInterruptException:
		pass
