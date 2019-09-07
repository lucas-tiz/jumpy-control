// Set up serial communication with MSP (publishers & subscriber)

#include "mcu_uart/serial.hpp" // serial communication class
#include <ros/ros.h> // common ROS pieces
#include <ros/console.h> // ros logging (info stream)
#include <std_msgs/Float32MultiArray.h> // multi-byte array message
#include <vector> // std::vector


int main(int argc, char **argv) {
    // set up node
    ros::init(argc, argv, "uart_comm"); // initialize ROS node (used to parse remapping args from cmd line), name node
    ros::NodeHandle n; // create handle to node process, register node with ROS master  

    // set up publishers
    ros::Publisher pub_pres_tank = n.advertise<std_msgs::Float32MultiArray>("pressure_tank", 1);
    ros::Publisher pub_pres_exo = n.advertise<std_msgs::Float32MultiArray>("pressure_exo", 1);
    ros::Publisher pub_fsr = n.advertise<std_msgs::Float32MultiArray>("fsr", 1); 
    ros::Publisher pub_pres_error = n.advertise<std_msgs::Float32MultiArray>("pressure_error", 1);
    ros::Publisher pub_duty = n.advertise<std_msgs::Float32MultiArray>("duty", 1);
    publisher_map pub_map; // map to indicate data indices that correspond to each publisher
    pub_map[&pub_pres_tank] = std::vector<int>({0});
    pub_map[&pub_pres_exo] = std::vector<int>({1});
    pub_map[&pub_fsr] = std::vector<int>({2});
    pub_map[&pub_pres_error] = std::vector<int>({3});
    pub_map[&pub_duty] = std::vector<int>({4});

    // set up serial port
    std::string device = "/dev/ttyACM0";
    Serial msp(device, B115200, pub_map); // set up MSP serial port

    // set up subscriber
    ros::Subscriber sub_tx = n.subscribe("mcu_tx",100, &Serial::transmitData, &msp, ros::TransportHints().tcpNoDelay(true)); // subscriber to data to transmit to MCU

    // loop and transmit/receive data
    while (ros::ok()) { // while ROS not shutdown
        msp.receiveData(); // check for new data
        ros::spinOnce(); // call all waiting callbacks
    }

    // close
    msp.close_port();
    return 0;
}