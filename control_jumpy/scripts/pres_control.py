#!/usr/bin/env python
import rospy
from std_msgs.msg import Time
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
import numpy as np

# pressure sequence [time, p1 KR, p2 HR, p3 HL, p4 KL] (s, psi)
p1 = 25.0
p2 = 30.0
p3 = 10.0
# pres_seq = [[00.00, -1.0, -1.0, -1.0, -1.0], # seal
# 			[01.00,   p1, -1.0, -1.0, -1.0], # inflate
# 			[01.10, -1.0, -1.0, -1.0, -1.0]] # seal
pres_seq = [[00.00, -1.0, -1.0, -1.0, -1.0], # seal
			[05.00,   p1,   p1,   p1,   p1], # inflate 1
			# [05.10, -1.0, -1.0, -1.0, -1.0], # seal
			[05.05, -1.0, -1.0, -1.0, -1.0], # seal
			[05.10,   p2,   p2,   p2,   p2], # inflate 2
			[5.30,   0.0,  0.0,  0.0,  0.0], # vent at peak
			[5.40,   0.0, 10.0, 10.0,  0.0], # inflate hips for landing, continue knee vent
			[5.42,   0.0, -1.0, -1.0,  0.0], # seal	hips, continue knee vent		
			[6.40,   0.0,  0.0,  0.0,  0.0]] # vent all


# global variables
num_valves = 4 


class Controller:
	''' perform preset actuator pressure timing sequence '''
	def __init__(self, publisher_topic):
		# set up message
		self.msg_tx = Float32MultiArray() # create message 
		self.msg_tx.data = np.zeros(num_valves+1, dtype=np.float32) # initialize with extra int for msg type
		self.msg_tx.layout.dim.append(MultiArrayDimension()) # add dimension to message
		self.msg_tx.layout.dim[0].size = num_valves+1 # extra int for message type

		# set up publisher and subscriber
		self.pub_tx = rospy.Publisher(publisher_topic, Float32MultiArray, queue_size=10) # publisher of data to transmit to MCU
		self.pub_test = rospy.Publisher('test_start', Time, queue_size=1)

		self.msg_pres_des = Float32MultiArray()
		self.msg_pres_des.data = np.zeros(num_valves, dtype=np.float32)
		self.pub_pres_des = rospy.Publisher('pres_des', Float32MultiArray, queue_size=10)

		# self.sub_tx = rospy.Subscriber(subscriber_topic, Float32MultiArray, queue_size=1, callback=self.fsrProcess) # subscriber to MCU data
		rospy.sleep(2) # required in order to not miss initial published data


	def runPresSequence(self, pres_seq):
		''' run pneumatic actuator pressure sequence '''
		msg_test = Time()
		t_start = rospy.get_time() # start time of pressure sequence
		msg_test.data = rospy.get_rostime()
		self.pub_test.publish(msg_test)

		# signal LED test start
		self.msg_tx.data[0] = 2
		self.msg_tx.data[1] = 1
		self.pub_tx.publish(self.msg_tx)

		self.msg_tx.data[0] = 0 # specify this is a pressure setpoint message type
		ind_pres_seq = 0 # initialize pressure sequence index
		while ind_pres_seq < len(pres_seq) and not rospy.is_shutdown(): # go through pressure sequence list
			t = rospy.get_time() - t_start # get elapsed time
			if t >= pres_seq[ind_pres_seq][0]: # if at next time in sequence
				self.pub_pres_des.publish(self.msg_pres_des) # publish previous desired pressure

				self.msg_tx.data[1:(num_valves+1)] = pres_seq[ind_pres_seq][1:(num_valves+1)] # add pressures to message
				self.pub_tx.publish(self.msg_tx) # publish message
				
				self.msg_pres_des.data[:] = pres_seq[ind_pres_seq][1:(num_valves+1)] # update message
				self.pub_pres_des.publish(self.msg_pres_des) # publish latest desired pressure

				print '  pressure setpoint updated:', pres_seq[ind_pres_seq][1:(num_valves+1)], 'at time {0:.2f}'.format(t) 
				ind_pres_seq = ind_pres_seq + 1 # increment pressure sequence index
		if ind_pres_seq == len(pres_seq):

			# signal LED test end
			self.msg_tx.data[0] = 2
			self.msg_tx.data[1] = 0
			self.pub_tx.publish(self.msg_tx)

			print('finished pressure sequence')


if __name__ == '__main__':  
	rospy.init_node('pres_control', anonymous=True) # initialize ROS node
	controller = Controller('mcu_tx') # create controller



	def shutdown(): # set up venting on shutdown
		print('shutdown: venting')
		rospy.sleep(0.1)
		vent_seq = 	 [[0.00, 00.0, 00.0, 00.0, 00.0], # vent all valves
			  		  [2.00, -1.0, -1.0, -1.0, -1.0]] # seal all valves
		controller.runPresSequence(vent_seq)
		rospy.sleep(2)
		print('vented!')
	rospy.on_shutdown(shutdown)

	print('running pressure sequence')
	controller.runPresSequence(pres_seq) # run pressure sequence
