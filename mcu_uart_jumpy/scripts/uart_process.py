#!/usr/bin/env python
import rospy
from mcu_uart_jumpy.msg import Msp
from mcu_uart_jumpy.msg import PresTraj
from std_msgs.msg import Float32
import numpy as np
from datetime import datetime
import matplotlib.pyplot as plt


class UartProcess:
    ''' Process UART data '''
    def __init__(self):
        self.pub_pres_tank = rospy.Publisher('pres_tank', Float32, queue_size=1000)
        self.pub_pres_act1_kr = rospy.Publisher('pres_act1_kr', Float32, queue_size=1000)
        self.pub_pres_act2_hr = rospy.Publisher('pres_act2_hr', Float32, queue_size=1000)
        self.pub_pres_act3_hl = rospy.Publisher('pres_act3_hl', Float32, queue_size=1000)
        self.pub_pres_act4_kr = rospy.Publisher('pres_act4_kl', Float32, queue_size=1000)
        self.pub_traj_dump = rospy.Publisher('traj_dump', Float32, queue_size=1000)

        self.data_traj = np.empty((0,6)) # trajectory data array

        plt.switch_backend('Agg')
        self.fig, self.ax = plt.subplots()


    def processData(self, msg_msp):
        ''' Process data based on UART packet type '''        
        if msg_msp.packet_type == 1: # publish to individual topics
            msg_float32 = Float32()
            msg_float32.data = msg_msp.packet_data[0]
            self.pub_pres_tank.publish(msg_float32)
            msg_float32.data = msg_msp.packet_data[1]
            self.pub_pres_act1_kr.publish(msg_float32)
            msg_float32.data = msg_msp.packet_data[2]
            self.pub_pres_act2_hr.publish(msg_float32)
            msg_float32.data = msg_msp.packet_data[3]
            self.pub_pres_act3_hl.publish(msg_float32)
            msg_float32.data = msg_msp.packet_data[4]
            self.pub_pres_act4_kr.publish(msg_float32)

        elif msg_msp.packet_type == 2: # save data
            self.data_traj = np.append(self.data_traj, [msg_msp.packet_data], axis=0)

        elif msg_msp.packet_type == 3: # export data & publish for plotting
            # export data     
            str_datetime = datetime.now().strftime("%Y-%m-%d %H-%M-%S")
            fname = rospy.get_param('file_prefix', 'jumpy-default') + ' ' + str_datetime
            np.savetxt(fname, self.data_traj, fmt='%.3f', delimiter=', ') # export data #TODO: save to different path, add ROS time to file name

            # export plot
            plt.cla()
            self.ax.plot(self.data_traj[:,0], self.data_traj[:,1], label='Tank')
            self.ax.plot(self.data_traj[:,0], self.data_traj[:,2], label='Act1 (kr)')
            self.ax.plot(self.data_traj[:,0], self.data_traj[:,3], label='Act2 (hr)')
            self.ax.plot(self.data_traj[:,0], self.data_traj[:,4], label='Act3 (hl)')
            self.ax.plot(self.data_traj[:,0], self.data_traj[:,5], label='Act4 (kl)')
            self.ax.set_xlim(0, self.data_traj[-1,0])
            self.ax.set(xlabel='Time (s)', ylabel='Pressure (psi)')
            self.ax.grid()
            self.ax.legend()
            self.fig.savefig(fname + '.png')

            # publish trajectory data for plotting
            msg_float32 = Float32()
            msg_float32.data = 0
            self.pub_traj_dump.publish(msg_float32)

            p_max = np.max(self.data_traj[:,1:-1])
            idx_traj = 0
            t = 0
            t_start = rospy.get_time()
            while idx_traj < self.data_traj.shape[0]:
                t = rospy.get_time() - t_start
                if t >= self.data_traj[idx_traj,0]:
                    msg_float32.data = self.data_traj[idx_traj,1]
                    self.pub_pres_tank.publish(msg_float32)
                    msg_float32.data = self.data_traj[idx_traj,2]
                    self.pub_pres_act1_kr.publish(msg_float32)
                    msg_float32.data = self.data_traj[idx_traj,3]
                    self.pub_pres_act2_hr.publish(msg_float32)
                    msg_float32.data = self.data_traj[idx_traj,4]
                    self.pub_pres_act3_hl.publish(msg_float32)
                    msg_float32.data = self.data_traj[idx_traj,5]
                    self.pub_pres_act4_kr.publish(msg_float32)

                    msg_float32.data = p_max
                    self.pub_traj_dump.publish(msg_float32)
                    idx_traj = idx_traj + 1

            msg_float32.data = 0
            self.pub_traj_dump.publish(msg_float32)

            # reset
            self.data_traj = np.empty((0,6)) 


if __name__ == '__main__':  
    rospy.init_node('uart_process', anonymous=True) # initialize ROS node
    processor = UartProcess()
    sub_rx = rospy.Subscriber('mcu_rx', Msp, queue_size=1, 
        callback=processor.processData) # subscriber to MCU data
    rospy.spin()

