// Receive MCU data from publisher, publish data to transmit to MSP

//TODO: check if this script still works


#include "mcu_uart/device_conversion.hpp"
#include "mcu_uart/byte_stuff.hpp"
#include "mcu_uart/util_lt.hpp" // my utilities

#include <ros/ros.h>
#include <ros/console.h> // ros logging (info stream)
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <vector>


// globals
std::vector<float> vec_float_rx; // create received data vector
int n_float_rx;
int pub_flag = 0; //DEBUG


// callback function called when new message arrives on corresponding topic
void uartCallback(const std_msgs::Float32MultiArray::ConstPtr& msg_rx) {
  vec_float_rx.clear(); // clear receiving vector
  vec_float_rx.insert(vec_float_rx.begin(), msg_rx->data.begin(), msg_rx->data.end()); // add data
  n_float_rx = msg_rx->layout.dim[0].size;
  pub_flag = 1; // set published flag high
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "uart_process");
  ros::NodeHandle n;
  ros::Subscriber sub_rx = n.subscribe("mcu_rx",10, uartCallback); // subscriber to data received from MCU
  ros::Publisher pub_tx = n.advertise<std_msgs::Float32MultiArray>("mcu_tx", 1); // publisher of data to transmit to MCU

  std_msgs::Float32MultiArray msg_tx; // create mutibyte message for publishing data
  msg_tx.layout.dim.push_back(std_msgs::MultiArrayDimension()); // add dimension message
  std::vector<float> vec_float_tx; // vector for concatenated data to transmit

  int count = 0; //DEBUG
  int inc; //DEBUG
  while (ros::ok()) { // while ROS not shutdown

    if (pub_flag == 1) {
      // float enc_angle[1];
      // convert_msmt(enc_angle, &vec16_cat_rx[0], ENC_AMT102V, MSMT_INT_TO_FLOAT, 1);
      // float pres[5];
      // convert_msmt(pres, &vec16_cat_rx[1], PRES_ASDXAVX030PGAA5, MSMT_INT_TO_FLOAT, 4);
      // convert_msmt(pres+4, &vec16_cat_rx[5], PRES_ASDXAVX100PGAA5, MSMT_INT_TO_FLOAT, 1);
      // float opt_sens[8];
      // convert_msmt(opt_sens, &vec16_cat_rx[6], OPT_SENS, MSMT_INT_TO_FLOAT, 8);

      // print_arr_float(vec_float_rx, n_float_rx, "values"); //TODO
      // print_arr_float(pres, 5, "pres");
      // print_arr_float(opt_sens, 8, "opt_sens");
      // ROS_INFO_STREAM(pub_flag << '\n');

      pub_flag = 0; // clear published flag
      count++; //DEBUG
    } 


    if (count == 5) {
    // update pressure setpoint: first uint16_t is 0
      vec_float_tx.clear(); // clear concatenated data vec
      // vec16_cat_tx = {0, 1, 2, 3, 4, 5};
    

    // update pressure controller gains
      uint8_t ctrl_id_arr[4] = {0,0,0,1};
      float ctrl_id_float;
      ConcatArr(ctrl_id_arr, 4, &ctrl_id_float);


      // uint16_t ctrl_id = ConcatData(1, 2);
      ROS_INFO_STREAM(ctrl_id_float);
      // uint16_t msb = ctrl_id >> 8;
      // uint16_t lsb = ctrl_id & 255;
      // ROS_INFO_STREAM(msb);
      // ROS_INFO_STREAM(lsb);
      vec_float_tx = {ctrl_id_float, 11, 12, 13, 14};



      // publish data
      msg_tx.data.clear(); // clear array
      msg_tx.layout.dim[0].size = vec_float_tx.size(); // update message size based on packet size
      msg_tx.data.insert(msg_tx.data.begin(), vec_float_tx.begin(), vec_float_tx.end()); // add data
      pub_tx.publish(msg_tx); // publish message
      count++;
    }


    //DEBUG:
    // if (pub_flag == 1) {
    //   vec16_cat_tx.clear(); // clear concatenated data vec
    //   vec16_cat_tx.insert(vec16_cat_tx.begin(), vec16_cat_rx.begin(), vec16_cat_rx.end()); // add received data
    //   print_vector16(vec16_cat_tx, n16_tx, "mcu_rx sub");

    //   count++;
    //   for(int i = 0; i < n16_tx; i++) {
    //     vec16_cat_tx[i] = vec16_cat_tx[i] + 1;// + count;
    //   }
    //   // print_vector16(vec16_cat_tx, n16_tx, "mcu_tx pub");
    //   // ROS_INFO_STREAM("\n"); 

    //   // publish data
    //   msg_tx.data.clear(); // clear array
    //   msg_tx.layout.dim[0].size = n16_tx; // update message size based on packet size
    //   msg_tx.data.insert(msg_tx.data.end(), vec16_cat_tx.begin(), vec16_cat_tx.end()); // add data
    //   pub_tx.publish(msg_tx); // publish message
    //   pub_flag = 0; // clear publish flag
    // }

    ros::spinOnce(); // call all waiting callbacks 
  }
  return 0;
}




