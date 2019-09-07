#!/usr/bin/env python

#TODO: import pressure list from file
#TODO: add time column to pressure list and update based on time

import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
import numpy as np
# import time


loop_freq = 1
num_pres_chan = 5 # number of pressure channels

p = 8.0
pres_list = [[0.0, 0.0, 0.0, 0.0, 0.0], # (psi) pressure setpoints
			[0.0, 0.0, 0.0, 0.0, 0.0],
			[p,   0.0, 0.0, 0.0, 0.0],
			[p,   0.0, 0.0, 0.0, 0.0],
			[p,   0.0, 0.0, 0.0, 0.0],
			[p,   0.0, 0.0, 0.0, 0.0],
			[p,   0.0, 0.0, 0.0, 0.0],
			[p,   0.0, 0.0, 0.0, 0.0],
			[p,   0.0, 0.0, 0.0, 0.0],
			[0.0, 0.0, 0.0, 0.0, 0.0]] 


if __name__ == '__main__':
	
	# set up node and publisher
	rospy.init_node('pres_control', anonymous=True) # initialize ROS node
   	pub_tx = rospy.Publisher('mcu_tx', Float32MultiArray, queue_size=10) # publisher of data to transmit to MCU
   	rospy.sleep(1) # required in order to not miss initial published data

   	# set up message
	msg_tx = Float32MultiArray() # create message 
	msg_tx.data = np.zeros(num_pres_chan+1, dtype=np.float32) # initialize with extra int for msg type
	msg_tx.data[0] = 0 # specify this is a pressure setpoint message type
	msg_tx.layout.dim.append(MultiArrayDimension()) # add dimension to message
	msg_tx.layout.dim[0].size = num_pres_chan+1 # extra int for message type

	# loop through pressure list and update pressure setpoints
	i = 0
	rate = rospy.Rate(loop_freq) # publish at 100 Hz
	while(i < len(pres_list)): # this would be control loop
		msg_tx.data[1:] = pres_list[i] # send pressures
		pub_tx.publish(msg_tx) # publish message
		print('pressure setpoints updated: ', pres_list[i])
		i += 1 
		rate.sleep()






