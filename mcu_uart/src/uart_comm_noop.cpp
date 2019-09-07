// Receive data from MSP, publish MSP data, transmit data to MSP


/* 
Notes
  -make sure user is in "dialout" group: sudo adduser <user> dialout

  -some required flags to clear for raw UART I/O:
    options.c_oflag &= ~OPOST; // all output options disabled (raw output)
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // raw input (noncanonical mode)
    options.c_iflag &= ~(ICRNL | IXON | BRKINT | INPCK | ISTRIP); // ignore carriage return & other stuff

  -can get current UART config with:
    tcgetattr(fd, &options); // get current options for port

  -sources:
    http://man7.org/linux/man-pages/man3/termios.3.html
    https://www.cmrr.umn.edu/~strupp/serial.html#2_5_4
    https://viewsourcecode.org/snaptoken/kilo/02.enteringRawMode.html
    http://tldp.org/HOWTO/Serial-Programming-HOWTO/x115.html
    https://stackoverflow.com/questions/6947413/how-to-open-read-and-write-from-serial-port-in-c
    https://en.wikibooks.org/wiki/Serial_Programming/termios
  */


/* TODO:
  -packet length error checking?
  -where to put uint16 to float conversion? probably in control node
*/


#include "mcu_uart/byte_stuff.hpp" // byte stuffing functions
#include "mcu_uart/util_lt.hpp" // my utilities

#include <ros/ros.h>                  // common ROS pieces
#include <ros/console.h> // ros logging (info stream)
// #include <ros/xmlrpc_manager.h>
#include <std_msgs/Float32MultiArray.h> // multi-byte array message
#include <std_msgs/MultiArrayDimension.h>
#include <termios.h> // POSIX terminal control definitions
#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h> // file contorl definitions
#include <errno.h> // error number definitions
#include <vector> // std::vector
#include <algorithm> // std::copy



#define BAUDRATE B57600
#define DEVICE "/dev/ttyACM0"


// global vars
int fd_mcu; // microcontroller serial port device file descriptor




// open port
int open_port(void) {
  int fd = open(DEVICE, O_RDWR | O_NOCTTY | O_NDELAY); // | O_NONBLOCK); // open read+write, no terminal control, ignore DCD signal
  if (fd == -1) {
    ROS_INFO_STREAM("open_port: Unable to open " << std::string(DEVICE) << " - " << strerror(errno));
    // perror(std::ostringstream"open_port: Unable to open " << std::string(DEVICE) << " - ");
  }
  else {
    ROS_INFO_STREAM("opened port with file descriptor: " << fd); // print data
    fcntl(fd, F_SETFL, FNDELAY); // set non-blocking read
  }
  return fd;
}


// configure port
void configure_port(int fd) {
  struct termios options;
  memset(&options, 0, sizeof(options)); // clear all flags
  cfsetispeed(&options, BAUDRATE); // set baud rate
  cfsetospeed(&options, BAUDRATE); // set baud rate
  options.c_cflag |= (CLOCAL | CREAD); // local mode (don't change port owner), enable receiver
  options.c_cflag |= CS8; // 8 data bits
  int r = tcsetattr(fd, TCSANOW, &options); // update configuration immediately
  ros::Duration(0.01).sleep(); // required for flush to work
  int rf = tcflush(fd, TCIFLUSH); // flush data received but not read

  if (r == 0) {
    ROS_INFO_STREAM("configured port with file descriptor: " << fd); // print data
  }
  else {
    ROS_INFO_STREAM("config_port: Unable to configure " << std::string(DEVICE) << " - " << strerror(errno));
  }
}


// callback for "mcu_in" messages
void uartCallback(const std_msgs::Float32MultiArray::ConstPtr& msg_tx) {
  // set up trasmit data structures
  int n_float_tx = msg_tx->layout.dim[0].size; // number of floats
  int n8_tx = n_float_tx*4; // corresponding number of bytes
  
  float arr_float_tx[n_float_tx]; // array for concatenated data to transmit
  uint8_t arr8_unstuff_tx[n8_tx]; // array for unstuffed data to transmit
  uint8_t arr8_stuff_tx[n8_tx+2]; // array for stuffed data to transmit

  // read data from ROS message and transmit over UART
  std::copy(msg_tx->data.begin(), msg_tx->data.end(), arr_float_tx); // add data from message to array
  SeparateArr(arr_float_tx, n_float_tx, arr8_unstuff_tx);  // separate floats into bytes
  StuffArr(arr8_unstuff_tx, n8_tx, arr8_stuff_tx); // create stuffed byte packet
  int n8_tx_write = write(fd_mcu, &arr8_stuff_tx[0], n8_tx+2); // write data to UART
    //DEBUG: 
    print_arr_float(arr_float_tx, n_float_tx, "mcu_tx");
    // print_arr8(arr8_unstuff_tx, n8_tx, "mcu_tx unstu");
    // print_arr8(arr8_stuff_tx, n8_tx+2, "mcu_tx stuff");
    // ROS_INFO_STREAM("\n"); 
}


// set up node, set up port, loop to get & publish data
int main(int argc, char **argv) {
  // set up node, publisher, and subscriber
  ros::init(argc, argv, "uart_comm"); // initialize ROS node (used to parse remapping args from cmd line), name node
  ros::NodeHandle n; // create handle to node process, register node with ROS master  
  ros::Publisher pub_rx = n.advertise<std_msgs::Float32MultiArray>("mcu_rx", 1); // publisher of data received from MCU
  ros::Subscriber sub_tx = n.subscribe("mcu_tx",100, uartCallback, ros::TransportHints().tcpNoDelay(true)); // subscriber to data to transmit to MCU

  // open and set up serial port
  fd_mcu = open_port(); // open UART port
  configure_port(fd_mcu); // configure UART port

  // set up received UART data structures
  uint8_t buf_rx[1]; // create buffer for UART byte  
  ssize_t n8_rx_read; // number of bytes from each read
  int n8_rx = 0; // number of bytes received in packet 
  int n_float_rx; // number of floats received in packet
  uint8_t arr8_stuff_rx[256]; // array for stuffed data received, reserve space for 256 max bytes in packet
  uint8_t arr8_unstuff_rx[252]; // array for unstuffed data received, reserve space for 254 max data bytes in packet
  float arr_float_rx[63]; // array for concatenated data received, reserve space for 63 max floats
  std_msgs::Float32MultiArray msg_rx; // create mutibyte message for publishing data
  msg_rx.layout.dim.push_back(std_msgs::MultiArrayDimension()); // add dimension message


  //DEBUG:
  // ros::WallTime t1, t2;

  // loop 
  while (ros::ok()) { // while ROS not shutdown
    // t1 = ros::WallTime::now(); //DEBUG

    // read
    n8_rx_read = read(fd_mcu, buf_rx, 1); // read byte
    if (n8_rx_read == 1) { // if byte was read
      if (buf_rx[0] == 0) { // if at end of packet
        arr8_stuff_rx[n8_rx] = buf_rx[0]; // add byte to data array
        n8_rx++; // increment number of bytes in packet
        UnstuffArr(arr8_stuff_rx, n8_rx, arr8_unstuff_rx); // unstuff data in packet
        ConcatArr(arr8_unstuff_rx, n8_rx-2, arr_float_rx); // concatenate bytes into floats
          //DEBUG:
          // print_arr8(arr8_stuff_rx, n8_rx, "mcu_rx stu");
          // print_arr8(arr8_unstuff_rx, (n8_rx-2), "mcu_rx uns");
          print_arr_float(arr_float_rx, (n8_rx-2)/4, "mcu_rx");
          // ROS_INFO_STREAM("\n"); 

        // publish data
        n_float_rx = (n8_rx-2)/4; // number of floats in packet
        msg_rx.data.clear(); // clear array
        msg_rx.layout.dim[0].size = n_float_rx; // update message size based on packet size
        msg_rx.data.insert(msg_rx.data.begin(), &arr_float_rx[0], &arr_float_rx[n_float_rx]); // add data
        pub_rx.publish(msg_rx); // publish message
        n8_rx = 0; // reset packet length
      }
      else {
        arr8_stuff_rx[n8_rx] = buf_rx[0]; // add byte to data array
        n8_rx++; // increment number of bytes in packet
      }
    }

    ros::spinOnce(); // call all waiting callbacks

    //DEBUG:
    // t2 = ros::WallTime::now();
    // ROS_INFO_STREAM("dt: " << t2-t1);

  }
  
  close(fd_mcu);
  return 0;
}

