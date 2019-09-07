#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
import numpy as np

# control parameters
fsr_threshold = 2.5 # (V) FSR threshold for triggering exo assistance
assist_delay = 0.0001 # (s) time delay after FSR trigger before exo assistance
pres_seq = [[0.00, 8.0], # (psi) pressure sequence [time, pressure]
			[0.50, 0.0],
			[1.00, 0.0]] # need last row to be 0 pressure to vent pneumatic actuators

# global variables
num_pres_chan = 5 # number of pressure channels (fixed by microcontroller code)
assist_flag = 0 # flag for exo assistance
t_node_start = 0 # ROS start time of node
global t_fsr_trigger # FSR trigger

class FsrController:
	''' perform preset actuator pressure sequence based on FSR load (voltage) '''
	def __init__(self, publisher_topic, subscriber_topic):
		# set up publisher and subscriber
		self.pub_tx = rospy.Publisher(publisher_topic, Float32MultiArray, queue_size=10) # publisher of data to transmit to MCU
		self.sub_tx = rospy.Subscriber(subscriber_topic, Float32MultiArray, queue_size=1, callback=self.fsrProcess)
		rospy.sleep(1) # required in order to not miss initial published data

		# set up message
		self.msg_tx = Float32MultiArray() # create message 
		self.msg_tx.data = np.zeros(num_pres_chan+1, dtype=np.float32) # initialize with extra int for msg type
		self.msg_tx.data[0] = 0 # specify this is a pressure setpoint message type
		self.msg_tx.layout.dim.append(MultiArrayDimension()) # add dimension to message
		self.msg_tx.layout.dim[0].size = num_pres_chan+1 # extra int for message type

		# set up variables
		self.assist_flag = 0

	def fsrProcess(self, msg_rx):
		''' determine if FSR value is above trigger threshold, trigger pressure sequence '''
		fsr_value = msg_rx.data[2] # get FSR voltage value
		if (fsr_value >= fsr_threshold) and (self.assist_flag == 0): # if above threshold and not already triggered
			global t_fsr_trigger
			t_fsr_trigger = rospy.get_time()
			print('foot strike triggered at: {:0.3f}'.format(t_fsr_trigger - t_node_start))
			rospy.Timer(rospy.Duration(assist_delay), callback=self.runPresSequence, oneshot=True) # start timer to pressure sequence
			self.assist_flag = 1 # set exo assistance flag

	def runPresSequence(self, event):
		''' run pneumatic actuator pressure sequence '''
		t_pres_seq = rospy.get_time()
		print('  running pressure assist sequence at: {:0.3f}, delay: {:0.3f}'.format(\
			t_pres_seq - t_node_start, t_pres_seq - t_fsr_trigger))
		t_start = rospy.get_time() # start time of pressure sequence
		ind_pres_seq = 0 # pressure sequence index
		while ind_pres_seq < len(pres_seq): # go through pressure sequence list
			t = rospy.get_time() - t_start
			if t >= pres_seq[ind_pres_seq][0]: # if at next sequence time
				self.msg_tx.data[1] = pres_seq[ind_pres_seq][1] # add pressure to message
				self.pub_tx.publish(self.msg_tx) # publish message
				# print('pressure setpoint updated: ', pres_seq[ind_pres_seq][1], 'at time: ', t)
				ind_pres_seq = ind_pres_seq + 1 # increment pressure sequence index
		self.assist_flag = 0 # clear exo assistance flag
		print('  sequence completed')


if __name__ == '__main__':  
	rospy.init_node('pres_control', anonymous=True) # initialize ROS node
	fsr_controller = FsrController('mcu_tx', 'mcu_rx') # create & start FSR controller

	try:
		t_node_start = rospy.get_time()
		print('running...')
		rospy.spin()
	except KeyboardInterrupt:
		print("shutting down")
