import numpy as np
from operator import itemgetter

def get_goals():

	# This needs to be replaced by pcl output from perception module
	X_goal1 = np.array([0, 0.5, 0.41-0.15-0.016]) # 0.1 is the stem root position, 0.15 is the length of the stem, 0.016 is the radius+safety factor for the straw
	X_g1n1 = np.array([-0.03, 0.5, 0.44-0.15-0.016])  
	X_g1n2 = np.array([0.03, 0.5, 0.44-0.15-0.016])  

	X_goal2 = np.array([0.15, 0.5, 0.44-0.15-0.016]) 
	X_g2n1 = np.array([0.12, 0.5, 0.44-0.15-0.016])  
	X_g2n2 = np.array([0.18, 0.5, 0.44-0.15-0.016])  

	X_goal3 = np.array([-0.15, 0.5, 0.44-0.15-0.016])  
	X_g3n1 = np.array([-0.12, 0.5, 0.41-0.15-0.016])   
	X_g3n2 = np.array([-0.18, 0.5, 0.41-0.15-0.016])  


	all_fruits =  []
	all_fruits.append(X_goal1)
	all_fruits.append(X_goal2)
	all_fruits.append(X_goal3)
	all_fruits.append(X_g1n1)
	all_fruits.append(X_g1n2) 
	all_fruits.append(X_g2n1)
	all_fruits.append(X_g2n2)
	all_fruits.append(X_g3n1)
	all_fruits.append(X_g3n2)

	# ordering thefruits by first in raw
	#goal = sorted(all_fruits, key=itemgetter(0))
	goal = all_fruits
	
	return goal
