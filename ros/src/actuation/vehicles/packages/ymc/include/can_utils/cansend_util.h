#ifndef CANSEND_UTIL_H
#define CANSEND_UTIL_H

#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>

namespace mycansend
{

std::string makeCmdArgument(const unsigned char data[], const size_t data_size, const std::string& str_id)
{
  std::vector<std::string> str_data;

  // unsigned char -> string of hex value
  for (size_t i = 0; i < data_size; i++)
  {
    std::stringstream ss;
    ss << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(data[i]);
    str_data.emplace_back(ss.str());
  }
  
  std::string cmd;
  cmd = str_id + "#";
  for (const auto& d : str_data)
  {
    cmd += d + ".";
  }

  // erase last "."
  cmd.pop_back();

  //std::cout << "arg:" << std::endl << cmd << std::endl;

  return cmd;
}

}

#endif
