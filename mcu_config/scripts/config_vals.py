#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
import numpy as np
import os
import sys
# import time


num_ctrl_params = 4 # number of parameters per controller


if __name__ == '__main__':

	# set up node and publisher
	rospy.init_node('config_vals', anonymous=True) # initialize ROS node
   	pub_tx = rospy.Publisher('mcu_tx', Float32MultiArray, queue_size=10) # publisher of data to transmit to MCU
   	rospy.sleep(1) # required in order to not miss initial published data

	# set up message
	msg_tx = Float32MultiArray()
	msg_tx.data = np.zeros(num_ctrl_params+1, dtype=np.float32) # clear msg, extra float for msg type
	msg_tx.layout.dim.append(MultiArrayDimension())
	msg_tx.layout.dim[0].size = num_ctrl_params+1 # extra int for message type

	# loop through control parameters and update
	rate = rospy.Rate(10) # publish at 100 Hz
	for ctrllr in rospy.get_param('/pres_ctrl_params/ctrllr'):
		ctrl_params = rospy.get_param(''.join(['/pres_ctrl_params/ctrllr', '/', ctrllr]))

		msg_tx.data[0] = ((int(ctrllr) << 8) | 1) # controller no. in 2nd LSByte, 1 in LSByte, 
		msg_tx.data[1:] = ctrl_params # add control parameters to message
		pub_tx.publish(msg_tx) # publish message
		rate.sleep()

	rospy.loginfo('Control parameters updated.')


	# configure length estimation coefficients TODO:


	# configure force estimation coefficients TODO:



