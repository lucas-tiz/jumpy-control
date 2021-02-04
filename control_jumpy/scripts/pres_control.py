#!/usr/bin/env python
import rospy
from mcu_uart_jumpy.msg import PresTraj
from std_msgs.msg import Time
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
import numpy as np


# pressure sequence [time, p1 KR, p2 HR, p3 HL, p4 KL] (s, psi)
p1 = 30 - 8 # 8 psi overshoot
# p2 = 30.0
# p3 = 10.0
pres_seq = [[00.00, -1.0, -1.0,   20, -1.0], # inflate
			[00.20, -1.0, -1.0,  0.0, -1.0], # vent
			[01.00, -1.0, -1.0, -1.0, -1.0]] # seal

# pres_seq = [[0.00,   -1,    p1,   p1,    -1], # inflate hips
# 			[0.02,   p1,    p1,   p1,    p1], # inflate knees
# 			[0.2,    -1,    -1,   -1,    -1], # seal
# 			[0.6,     0,     0,    0,     0], # vent
# 			[1.00,    -1,   -1,   -1,    -1]] # seal

			# [05.05, -1.0, -1.0, -1.0, -1.0], # seal
			# [05.10,   p2,   p2,   p2,   p2], # inflate 2
			# [5.30,   0.0,  0.0,  0.0,  0.0], # vent at peak
			# [5.40,   0.0, 10.0, 10.0,  0.0], # inflate hips for landing, continue knee vent
			# [5.42,   0.0, -1.0, -1.0,  0.0], # seal	hips, continue knee vent		
			# [6.40,   0.0,  0.0,  0.0,  0.0]] # vent all

# 1.2:
# pres_seq = [[00.000, -1.0, -1.0, -1.0, -1.0], # seal
# 			[05.000,   p1, -1.0, -1.0,   p1], # inflate knees
# 			[05.019,   p1,   p1,   p1,   p1], # inflate hips
# 			[05.062, -1.0,   p1,   p1, -1.0], # seal knees
# 			[05.071, -1.0, -1.0, -1.0, -1.0], # seal hips
# 			[05.7,   -1.0, -1.0, -1.0, -1.0]] # end
# 1.3:
# pres_seq = [[00.000, -1.0, -1.0, -1.0, -1.0], # seal
# 			[05.000, -1.0,   p1,   p1, -1.0], # inflate hips
# 			[05.017,   p1,   p1,   p1,   p1], # inflate knees
# 			[05.083, -1.0, -1.0, -1.0, -1.0], # seal hips & knees
# 			[05.7,   -1.0, -1.0, -1.0, -1.0]] # end
# 1.4:
# pres_seq = [[00.000, -1.0, -1.0, -1.0, -1.0], # seal
# 			[05.000, -1.0,   p1,   p1, -1.0], # inflate hips
# 			[05.010,   p1,   p1,   p1,   p1], # inflate knees
# 			[05.107,   p1, -1.0, -1.0,   p1], # seal hips
# 			[05.112, -1.0, -1.0, -1.0, -1.0], # seal knees
# 			[05.7,   -1.0, -1.0, -1.0, -1.0]] # end
# 1.4:
# pres_seq = [[00.000, -1.0, -1.0, -1.0, -1.0], # seal
# 			[05.000, -1.0,   p1,   p1, -1.0], # inflate hips
# 			[05.018,   p1,   p1,   p1,   p1], # inflate knees
# 			[05.124,   p1, -1.0, -1.0,   p1], # seal hips
# 			[05.134, -1.0, -1.0, -1.0, -1.0], # seal knees
# 			[05.7,   -1.0, -1.0, -1.0, -1.0]] # end

# test_pres_seq = [[00.000,  p1,   -1.0, -1.0,   p1], # seal
# 			     [00.5,    p1,     p2,   p2,   p1],
# 				 [00.6,   0.0,    0.0,  0.0,  0.0],
# 				 [01.2,  -1.0,   -1.0, -1.0, -1.0]]

vent_seq = 	 [[0.00, 00.0, 00.0, 00.0, 00.0], # vent all valves
			  [3.0, -1.0, -1.0, -1.0, -1.0]] # seal all valves

# global variables
num_valves = 4 


class Controller:
	''' perform preset actuator pressure timing sequence '''
	def __init__(self, publisher_topic):
		# set up message & publisher
		self.msg_tx = Float32MultiArray() # create message 
		self.msg_tx.data = np.zeros(num_valves+2, dtype=np.float32) # initialize for max message length
		self.msg_tx.layout.dim.append(MultiArrayDimension()) # add dimension to message
		self.pub_tx = rospy.Publisher(publisher_topic, Float32MultiArray, queue_size=10) # publisher of data to transmit to MCU
		rospy.sleep(2) # required in order to not miss initial published data


	def runPresSequence(self, pres_seq, data_dump=True):
		''' run pneumatic actuator pressure sequence '''
		# write valve time sequence to MCU
		print('  updating valve sequence.')
		self.msg_tx.layout.dim[0].size = num_valves+2 # set array size
		for idx_pres_seq in range(len(pres_seq)): # loop through valve sequence list
			self.msg_tx.data[0] = (idx_pres_seq << 8) | 3 # specify valve sequence update (LSByte) & sequence index (2nd LSByte)
			self.msg_tx.data[1:(num_valves+2)] = pres_seq[idx_pres_seq] # add sequence data to message
			self.pub_tx.publish(self.msg_tx) # publish message
		rospy.sleep(0.1)

		# start valve time sequence
		self.msg_tx.layout.dim[0].size = 1 # set array size
		self.msg_tx.data[0] = 4 # specify this is a valve sequence start 
		self.pub_tx.publish(self.msg_tx) # publish message
		print('  started valve time sequence.')
		t_start = rospy.get_time() # start time of valve sequence
		# while (rospy.get_time() - t_start) < pres_seq[-1][0]*1.5: continue # allow time for sequence to run
		rospy.sleep(pres_seq[-1][0]*1.1) # allow time for sequence to run

		# ask for data back
		if data_dump:
			self.msg_tx.layout.dim[0].size = 1 # set array size
			self.msg_tx.data[0] = 5 # specify this is a trajectory data dump
			self.pub_tx.publish(self.msg_tx) # publish message
			print('  started data dump\n')
			rospy.sleep(pres_seq[-1][0]*2.5) # allow time for dump/plot


	def setPressure(self, pres):
		self.msg_tx.layout.dim[0].size = num_valves+1 # extra int for message type
		self.msg_tx.data[0] = 0 # specify this is a pressure setpoint message type
		self.msg_tx.data[1:(num_valves+1)] = pres # add pressures to message
		self.pub_tx.publish(self.msg_tx) # publish message


if __name__ == '__main__':  
	rospy.init_node('pres_control', anonymous=True) # initialize ROS node
	controller = Controller('mcu_tx') # create controller

	# def shutdown(): # set up venting on shutdown
	# 	print('shutdown - venting:')
	# 	controller.runPresSequence(vent_seq, data_dump=True)
	# rospy.on_shutdown(shutdown)

	rospy.set_param('/file_prefix', rospy.get_param('~file_prefix'))

	rospy.sleep(4) # time to get into position
	controller.runPresSequence(pres_seq) # run pressure sequence

	controller.setPressure([0, 0, 0, 0]) # vent
	rospy.sleep(3) # allow vent time
	controller.setPressure([-1, -1, -1, -1]) # seal


	# controller.runPresSequence(vent_seq, data_dump=False) # run pressure sequence
	print('done')

	rospy.spin()