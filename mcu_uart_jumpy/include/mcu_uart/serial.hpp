// Class for serial communication with microcontroller

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
*/


#include "mcu_uart/byte_stuff.hpp" // byte stuffing functions
#include "mcu_uart/util_lt.hpp" // my utilities

#include <ros/ros.h>                  // common ROS pieces
#include <ros/console.h> // ros logging (info stream)
#include <std_msgs/Float32MultiArray.h> // multi-byte array message
#include <std_msgs/MultiArrayDimension.h>
#include <termios.h> // POSIX terminal control definitions
#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h> // file contorl definitions
#include <errno.h> // error number definitions
#include <vector> // std::vector
#include <algorithm> // std::copy
#include <map>


typedef std::map<ros::Publisher*, std::vector<int>> publisher_map;


class Serial {
    private:
        const std::string device; // device name
        int baud_rate; // baud rate
        int fd_mcu; // microcontroller serial port device file descriptor
        publisher_map pub_map; // map of publishers & associated indices

    public:
        Serial(const std::string& in_device, int in_baud_rate, publisher_map &in_pub_map); // constructor
        void open_port(void); // open serial port
        void configure_port(void); // configure serial port
        void close_port(void); // close serial port
        void transmitData(const std_msgs::Float32MultiArray::ConstPtr& msg_tx); // transmit data to MCU
        void receiveData(void); // receive data from MCU and publish to corresponding topic
};

Serial::Serial(const std::string& in_device, int in_baud_rate, publisher_map &in_pub_map) 
    : device(in_device), baud_rate(in_baud_rate), pub_map(in_pub_map)
    /* constructor */
    {
    open_port(); // open port
    configure_port(); // configure port
}
 
void Serial::open_port(void) {
    /* open serial port */
    fd_mcu = open(device.c_str(), O_RDWR | O_NOCTTY | O_NDELAY); // | O_NONBLOCK); // open read+write, no terminal control, ignore DCD signal
    if (fd_mcu == -1) {
        ROS_ERROR_STREAM("open_port: Unable to open " << std::string(device) << " - " << strerror(errno));
    }
    else {
        ROS_INFO_STREAM("opened port with file descriptor: " << fd_mcu); // print data
        fcntl(fd_mcu, F_SETFL, FNDELAY); // set non-blocking read
    }
}

void Serial::configure_port(void) {
    /* configure serial port */
    struct termios options; // create termios struct
    memset(&options, 0, sizeof(options)); // clear all flags
    cfsetispeed(&options, baud_rate); // set baud rate
    cfsetospeed(&options, baud_rate); // set baud rate
    options.c_cflag |= (CLOCAL | CREAD); // local mode (don't change port owner), enable receiver
    options.c_cflag |= CS8; // 8 data bits
    int r = tcsetattr(fd_mcu, TCSANOW, &options); // update configuration immediately
    ros::Duration(0.01).sleep(); // sleep required for flush to work
    int rf = tcflush(fd_mcu, TCIFLUSH); // flush data received but not read

    if (r == 0) {
        ROS_INFO_STREAM("configured port with file descriptor: " << fd_mcu); // print data
    }
    else {
        ROS_ERROR_STREAM("config_port: Unable to configure " << std::string(device) << " - " << strerror(errno));
    }
}

void Serial::close_port(void) {
      close(fd_mcu);
}

void Serial::transmitData(const std_msgs::Float32MultiArray::ConstPtr& msg_tx) {
    /* transmit data to MCU */
    int n_float_tx = msg_tx->layout.dim[0].size; // number of floats
    int n8_tx = n_float_tx*4; // corresponding number of bytes
    float arr_float_tx[n_float_tx]; // array for concatenated data to transmit
    uint8_t arr8_unstuff_tx[n8_tx]; // array for unstuffed data to transmit
    uint8_t arr8_stuff_tx[n8_tx+2]; // array for stuffed data to transmit

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

void Serial::receiveData(void) {
    /* receive data from MCU */
    static uint8_t buf_rx[1]; // create buffer for UART byte  
    static ssize_t n8_rx_read; // number of bytes from each read
    static int n8_rx = 0; // number of bytes received in packet 
    static uint8_t arr8_stuff_rx[256]; // array for stuffed data received, reserve space for 256 max bytes in packet
    static uint8_t arr8_unstuff_rx[252]; // array for unstuffed data received, reserve space for 254 max data bytes in packet
    static float arr_float_rx[63]; // array for concatenated data received, reserve space for 63 max floats
    static std_msgs::Float32MultiArray msg; // create mutibyte message for publishing data
    static bool initialized;
    if (!initialized) {
        initialized = true;
        msg.layout.dim.push_back(std_msgs::MultiArrayDimension()); // add dimension message
    }

    n8_rx_read = read(fd_mcu, buf_rx, 1); // read byte if available
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

        publisher_map::iterator iter;
        for (iter = pub_map.begin(); iter != pub_map.end(); iter++) { // loop over publishers
            msg.data.clear(); // clear message data vector
            msg.layout.dim[0].size = 0; // clear message size

            for (std::vector<int>::size_type i = 0; i != iter->second.size(); i++) { // loop over indices for publisher
                msg.data.push_back(arr_float_rx[iter->second[i]]); // add data corresponding to index
                msg.layout.dim[0].size++; // increment message size
            }
            iter->first->publish(msg); // publish message
        }

        n8_rx = 0; // reset packet length
      }
      else {
        arr8_stuff_rx[n8_rx] = buf_rx[0]; // add byte to data array
        n8_rx++; // increment number of bytes in packet
      }
    }
}

