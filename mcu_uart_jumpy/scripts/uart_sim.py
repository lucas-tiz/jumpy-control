#!/usr/bin/env python

import rospy
import serial




if __name__ == '__main__':  
    rospy.init_node('uart_sim', anonymous=True) # initialize ROS node


    # ser = serial.Serial('/dev/ttyS1', baudrate=57600, timeout=3.0)
    ser = serial.Serial('/dev/ttyS100')
    print(ser.parity == serial.PARITY_NONE)
    print(ser.stopbits)


    while not rospy.is_shutdown():
        ser.write('test')
        rospy.sleep(1)


