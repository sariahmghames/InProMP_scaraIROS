#!/usr/bin/env python
from __future__ import absolute_import, division, print_function
import numpy as np
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import csv
import random
import PRR_kinematics 
#import threading 



class ScaraSim_RefControl:

	def __init__(self, model, armR_enabled=True, armL_enabled=True, gripperR_enabled=False, gripperL_enabled=False):
		rospy.init_node('graspberry_scara_pathgen')
		self.rate = rospy.Rate(20)
		self.armR_enabled = armR_enabled
		self.armL_enabled = armL_enabled
		self.gripperR_enabled = gripperR_enabled
		self.gripperL_enabled = gripperL_enabled
		#self.lock = threading.Lock()
		self.nameR = []
		self.current_position_armR = []
		self.current_velocity_armR = []
		self.current_effort_armR = []
		self.nameL = []
		self.current_position_armL = []
		self.current_velocity_armL = []
		self.current_effort_armL = []
		self.model = model
		self.data = np.array([])

		#self.thread = threading.Thread(target=self.graspberry_scara_pathgen)
		#self.thread.start()


	def gazebo_command_publisher(self):
		self.response_L1R = rospy.Publisher("/scara_arm/link1_controller/command", Float64, queue_size=10)
		self.response_L2R = rospy.Publisher("/scara_arm/link2_controller/command", Float64, queue_size=10)
		self.response_L3R = rospy.Publisher("/scara_arm/link3_controller/command", Float64, queue_size=10)

		self.response_L1L = rospy.Publisher("/scara_arm/link1M_controller/command", Float64, queue_size=10)
		self.response_L2L = rospy.Publisher("/scara_arm/link2M_controller/command", Float64, queue_size=10)
		self.response_L3L = rospy.Publisher("/scara_arm/link3M_controller/command", Float64, queue_size=10)


	#thread function: listen for joint_states messages
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
        
        #rospy.spin()
	

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

	def moveR_to_xyz(self, X, Y, Z, speed, mode):
		x, y, z = X / 1000, Y/1000, Z/1000
		qR = self.model.PRR_IK2(x,y,z)
		print('qR1=', qR[0])
		print('qR2=', qR[1])
		print('qR3=', qR[2])
		if (isinstance(mode, int) and mode == 0):
			#print('started IK1 for arm1')
			return self.__arm_do_action(qR[0], qR[1], qR[2])
			#print('sended command to gazebo armR')


	def moveL_to_xyz(self, X, Y, Z, speed, mode):
		x, y, z = X / 1000, Y/1000, Z/1000
		qL = self.model.PRR_IK2(x,y,z)
		print('qL1=', qL[0])
		print('qL2=', qL[1])
		print('qL3=', qL[2])
		if (isinstance(mode, int) and mode == 0):
			#print('started IK1 for arm2')
			return self.__arm_do_action(qL[0], qL[1], qL[2])
			#print('sended command to gazebo armL')



def scara_reach2grasp():

	#create an object of the scara robot model
	scara_model = PRR_kinematics.PRRkinematics()

	scara_RefControl = ScaraSim_RefControl(scara_model, armR_enabled=True, armL_enabled=True, gripperR_enabled=False, gripperL_enabled=False)

	#points = [[400, 400, 510], [400, 400, 0], [600, 200, 200], [600, 400, 400]]
	with open('./reach_pts.csv', 'r') as csvfile:
		file = csv.reader(csvfile, delimiter = ',')
		row_count = sum(1 for row in file) 
		i = random.randint(0,row_count)
		print('i', i)
		for row in file:
			scara_RefControl.data.append(file(row))

	ldata = len(scara_RefControl.data)
	print('ldata',ldata)
	X_des = scara_RefControl.data[i]
	while not rospy.is_shutdown():
		for p in X_des:
			if scara_RefControl.armR_enabled:
				scara_RefControl.moveR_to_xyz(*p, speed =20, mode = 0)

			if scara_RefControl.armL_enabled:
				scara_RefControl.moveL_to_xyz(*p, speed =20, mode = 0)

        scara_RefControl.rate.sleep()   

if __name__ == '__main__':
	try:
		scara_reach2grasp()
	except rospy.ROSInterruptException:
		pass
