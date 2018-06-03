/*!
 * \file SickLMS5xx Message.hh
 * \brief Defines the class SickLMS5xxMessage.
 *
 * Code by Jason C. Derenick and Christopher R. Mansley.
 * Contact jasonder(at)seas(dot)upenn(dot)edu
 *
 * The Sick LIDAR Matlab/C++ Toolbox
 * Copyright (c) 2009, Jason C. Derenick and Christoper R. Mansley
 * All rights reserved.
 *
 * This software is released under a BSD Open-Source License.
 * See http://sicktoolbox.sourceforge.net
 */

#ifndef SICK_LMS_5xx_MESSAGE_HH
#define SICK_LMS_5xx_MESSAGE_HH

/* Macros */
#define SICK_LMS_5xx_MSG_HEADER_LEN             (1)  ///< Sick LMS 5xx message header length in bytes
#define SICK_LMS_5xx_MSG_PAYLOAD_MAX_LEN    (30000)  ///< Sick LMS 5xx maximum payload length
#define SICK_LMS_5xx_MSG_TRAILER_LEN            (1)  ///< Sick LMS 5xx length of the message trailer

/* Definition dependencies */
#include <string.h>
#include <arpa/inet.h>
#include "SickMessage.hh"
#include "SickException.hh"

/* Associate the namespace */
namespace SickToolbox {

  /**
   * \class SickLMS5xxMessage
   * \brief A class to represent all messages sent to and from the Sick LMS 5xx unit.
   */
  class SickLMS5xxMessage : public SickMessage< SICK_LMS_5xx_MSG_HEADER_LEN, SICK_LMS_5xx_MSG_PAYLOAD_MAX_LEN, SICK_LMS_5xx_MSG_TRAILER_LEN > {
  
  public:
    
    /** A standard constructor */
    SickLMS5xxMessage( );
    
    /** Constructs a packet by using BuildMessage */
    SickLMS5xxMessage( const uint8_t * const payload_buffer, const unsigned int payload_length );
    
    /** Constructs a packet using ParseMessage() */
    SickLMS5xxMessage( const uint8_t * const message_buffer );
    
    /** Construct a well-formed raw packet */
    void BuildMessage( const uint8_t * const payload_buffer, const unsigned int payload_length );
    
    /** Populates fields from a (well-formed) raw packet */
    void ParseMessage( const uint8_t * const message_buffer ) throw ( SickIOException );
    
    /** Get the length of the service code associated with the message */
    std::string GetCommandType( ) const { return _command_type; }
    
    /** Get the service sub-code associated with the message */
    std::string GetCommand( ) const { return _command; }

    /** Reset the data associated with this message (for initialization purposes) */
    void Clear( );
    
    /** A debugging function that prints the contents of the frame. */
    void Print( ) const;
    
    /** Destructor */
    ~SickLMS5xxMessage( );

  private:

    /** Command type associated w/ message */
    std::string _command_type;
    
    /** Command associated w/ message */
    std::string _command;
    
  };
  
} /* namespace SickToolbox */

#endif /* SICK_LMS_5xx_MESSAGE_HH */
