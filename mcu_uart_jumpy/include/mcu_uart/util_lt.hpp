#ifndef UTIL_LT_
#define UTIL_LT_

#include <stdint.h>
#include <sstream>
#include <iostream>
#include <ros/console.h> // ros logging (info stream)


void print_arr8(uint8_t *vec, int len, std::string s_pre = "") {
  std::ostringstream ss;
  ss << s_pre << ": ";
  for (int i = 0; i < len; i++) {
    ss << (unsigned int)vec[i] << " ";
  }
  ROS_INFO_STREAM(ss.str());
}

void print_arr16(uint16_t *vec, int len, std::string s_pre = "") {
  std::ostringstream ss;
  ss << s_pre << ": ";
  for (int i = 0; i < len; i++) {
    ss << (int)vec[i] << " ";
  }
  ROS_INFO_STREAM(ss.str());
}

void print_arr_float(float *arr, int len, std::string s_pre = "") {
  std::ostringstream ss;
  ss << s_pre << ": ";
  for (int i = 0; i < len; i++) {
    ss << arr[i] << " ";
  }
  ss << "\n";
  std::cout << ss.str();
  // ROS_DEBUG_STREAM(ss.str());
}

void print_vector8(std::vector<uint8_t> &vec, int len, std::string s_pre = "") {
  std::ostringstream ss;
  ss << s_pre << ": ";
  for (int i = 0; i < len; i++) {
    ss << (int)vec[i] << " ";
  }
  ROS_INFO_STREAM(ss.str());
}

void print_vector16(std::vector<uint16_t> &vec, int len, std::string s_pre = "") {
  std::ostringstream ss;
  ss << s_pre << ": ";
  for (int i = 0; i < len; i++) {
    ss << (int)vec[i] << " ";
  }
  ROS_INFO_STREAM(ss.str());
}

#endif // UTIL_LT_