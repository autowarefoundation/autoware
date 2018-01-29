/*!
 * \file SickLMS5xxUtility.hh
 * \brief Defines simple utility functions for working with the
 *        Sick LMS 5xx laser range finder units.
 *
 * Code by Jason C. Derenick and Christopher R. Mansley.
 * Contact jasonder(at)seas(dot)upenn(dot)edu
 *
 * The Sick LIDAR Matlab/C++ Toolbox
 * Copyright (c) 2009, Jason C. Derenick and Christopher R. Mansley
 * All rights reserved.
 *
 * This software is released under a BSD Open-Source License.
 * See http://sicktoolbox.sourceforge.net
 */

/* Auto-generated header */
#include "SickConfig.hh"

/* Implementation Dependencies */
#include <sstream>

/**
 * \def REVERSE_BYTE_ORDER_16
 * \brief Reverses the byte order of the given 16 bit unsigned integer
 */ 
#define REVERSE_BYTE_ORDER_16( y ) ( ( ( y & 0x00FF ) << 8 ) | ( ( y & 0xFF00 ) >> 8 ) )

/**
 * \def REVERSE_BYTE_ORDER_32
 * \brief Reverses the byte order of the given 32 bit unsigned integer
 */ 
#define REVERSE_BYTE_ORDER_32( y ) ( ( ( y & 0x000000FF ) << 24 ) | ( ( y & 0x0000FF00 ) << 8 ) | ( ( y & 0x00FF0000 ) >> 8 ) | ( ( y & 0xFF000000 ) >> 24 ) )

/* Associate the namespace */
namespace SickToolbox {
  
#ifndef WORDS_BIGENDIAN
  
  /* NOTE: The following functions are necessary since the Sick LD doesn't adopt the
   *       convention of converting from network byte order.
   */
  
  /**
   * \brief Converts host byte order (little-endian) to Sick LMS byte order (little-endian)
   * \param value The 2-byte value to convert to little-endian
   * \return Value in Sick LMS byte order (little-endian)
   */
  inline uint16_t host_to_sick_lms_5xx_byte_order( uint16_t value ) {
    return value;
  }
  
  /**
   * \brief Converts host byte order (little-endian) to Sick LMS byte order (little-endian)
   * \param value The 4-byte value to convert to little-endian
   * \return Value in Sick LMS byte order (little-endian)
   */
  inline uint32_t host_to_sick_lms_5xx_byte_order( uint32_t value ) {
    return value;
  }
  
  /**
   * \brief Converts Sick LMS byte order (little-endian) to host byte order (little-endian)
   * \param value The 2-byte value to convert to little-endian
   * \return Value in host byte order (little-endian)
   */
  inline uint16_t sick_lms_5xx_to_host_byte_order( uint16_t value ) {
    return value;
  }
  
  /**
   * \brief Converts Sick LMS byte order (little-endian) to host byte order (little-endian)
   * \param value The 4-byte value to convert to little-endian
   * \return Value in host byte order (little-endian)
   */
  inline uint32_t sick_lms_5xx_to_host_byte_order( uint32_t value ) {
    return value;
  }
  
#else // The host has a big-endian architecture
  
  /**
   * \brief Converts host byte order (big-endian) to Sick LMS byte order (little-endian)
   * \param value The 2-byte value to convert to little-endian
   * \return Value in Sick LMS byte order (little-endian)
   */
  inline uint16_t host_to_sick_lms_5xx_byte_order( uint16_t value ) {
    return REVERSE_BYTE_ORDER_16(value);
  }
  
  /**
   * \brief Converts host byte order (big-endian) to Sick LMS byte order (little-endian)
   * \param value The 4-byte value to convertto little-endian
   * \return Value in Sick LMS byte order (little-endian)
   */
  inline uint32_t host_to_sick_lms_5xx_byte_order( uint32_t value ) {
    return REVERSE_BYTE_ORDER_32(value);
  }
  
  /**
   * \brief Converts Sick LMS byte order (little-endian) to host byte order (big-endian)
   * \param value The 2-byte value to convert to big-endian
   * \return Value in host byte order (big-endian)
   */
  inline uint16_t sick_lms_5xx_to_host_byte_order( uint16_t value ) {
    return REVERSE_BYTE_ORDER_16(value);
  }
  
  /**
   * \brief Converts Sick LMS byte order (little-endian) to host byte order (big-endian)
   * \param value The 4-byte value to convert to big-endian
   * \return Value in host byte order (big-endian)
   */
  inline uint32_t sick_lms_5xx_to_host_byte_order( uint32_t value ) {
    return REVERSE_BYTE_ORDER_32(value);
  }
  
#endif /* _LITTLE_ENDIAN_HOST */
  
  /*
   * NOTE: Other utility functions can be defined here
   */
  
  /**
   * \brief Utility function for converting int to standard string
   * \param value Integer to be converted
   * \return String representing the given integer
   */
  inline std::string int_to_str( const int value ) {
    std::stringstream ss;
    ss << value;
    return ss.str();
  }
  

  
} //namespace SickToolbox
