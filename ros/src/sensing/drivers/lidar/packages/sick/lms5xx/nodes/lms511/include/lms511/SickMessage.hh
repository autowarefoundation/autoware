/*!
 * \file SickMessage.hh
 * \brief Defines the abstract parent class for all Sick messages.
 *
 * Code by Jason C. Derenick and Thomas H. Miller.
 * Contact derenick(at)lehigh(dot)edu
 *
 * The Sick LIDAR Matlab/C++ Toolbox
 * Copyright (c) 2008, Jason C. Derenick and Thomas H. Miller
 * All rights reserved.
 *
 * This software is released under a BSD Open-Source License.
 * See http://sicktoolbox.sourceforge.net
 */

#ifndef SICK_MESSAGE
#define SICK_MESSAGE

/* Dependencies */
#include <arpa/inet.h>
#include <iomanip>
#include <iostream>

/* Associate the namespace */
namespace SickToolbox {

  /**
   * \class SickMessage
   * \brief Provides an abstract parent for all Sick messages
   */
  template < unsigned int MSG_HEADER_LENGTH, unsigned int MSG_PAYLOAD_MAX_LENGTH, unsigned int MSG_TRAILER_LENGTH >
  class SickMessage {

  public:

    /** Some constants to make things more manageable */
    static const unsigned int MESSAGE_HEADER_LENGTH = MSG_HEADER_LENGTH;
    static const unsigned int MESSAGE_TRAILER_LENGTH = MSG_TRAILER_LENGTH;
    static const unsigned int MESSAGE_PAYLOAD_MAX_LENGTH = MSG_PAYLOAD_MAX_LENGTH;
    static const unsigned int MESSAGE_MAX_LENGTH = MESSAGE_HEADER_LENGTH + MESSAGE_PAYLOAD_MAX_LENGTH + MESSAGE_TRAILER_LENGTH;
        
    /** A standard constructor */
    SickMessage( );

    /** Construct a well-formed Sick message */
    void BuildMessage( const uint8_t * const payload_buffer, const unsigned int payload_length );
    
    /** Populates fields given a sequence of bytes representing a raw message */
    virtual void ParseMessage( const uint8_t * const message_buffer ) = 0;

    /** Returns a copy of the raw message buffer */
    void GetMessage( uint8_t * const message_buffer ) const;

    /** Resturns the total message length in bytes */
    unsigned int GetMessageLength( ) const { return _message_length; }
    
    /** Returns a copy of the raw message payload */
    void GetPayload( uint8_t * const payload_buffer ) const;

    /** Returns a copy of the payload as a C String */
    void GetPayloadAsCStr( char * const payload_str ) const;
    
    /** Returns a subregion of the payload specified by indices */
    void GetPayloadSubregion( uint8_t * const payload_sub_buffer, const unsigned int start_idx,
			      const unsigned int stop_idx ) const;
    
    /** Returns the total payload length in bytes */
    unsigned int GetPayloadLength( ) const { return _payload_length; } 
    
    /** Indicates whether the message container is populated */
    bool IsPopulated( ) const { return _populated; };
    
    /** Clear the contents of the message container/object */
    virtual void Clear( );

    /** Print the contents of the message */
    virtual void Print( ) const;

    /** A virtual destructor */
    virtual ~SickMessage( );

  protected:

    /** The length of the message payload in bytes */
    unsigned int _payload_length;

    /** The length of the message in bytes */
    unsigned int _message_length;

    /** The message as a raw sequence of bytes */
    uint8_t _message_buffer[MESSAGE_MAX_LENGTH];

    /** Indicates whether the message container/object is populated */
    bool _populated;

  };


  /**
   * \brief A default constructor
   */
  template< unsigned int MSG_HEADER_LENGTH, unsigned int MSG_PAYLOAD_MAX_LENGTH, unsigned int MSG_TRAILER_LENGTH >
  SickMessage< MSG_HEADER_LENGTH, MSG_PAYLOAD_MAX_LENGTH, MSG_TRAILER_LENGTH >::SickMessage( ) { }

  /**
   * \brief Constructs a Sick message given the parameter values
   * \param *payload_buffer The payload of the message as an array of bytes
   * \param payload_length The length of the payload in bytes
   */
  template< unsigned int MSG_HEADER_LENGTH, unsigned int MSG_PAYLOAD_MAX_LENGTH, unsigned int MSG_TRAILER_LENGTH >
  void SickMessage< MSG_HEADER_LENGTH, MSG_PAYLOAD_MAX_LENGTH, MSG_TRAILER_LENGTH >::BuildMessage( const uint8_t * const payload_buffer, const unsigned int payload_length ) {
    
    /* Clear the object */
    Clear();
    
    /* Assign the payload and message lengths */
    _payload_length = payload_length;
    _message_length = MESSAGE_HEADER_LENGTH + MESSAGE_TRAILER_LENGTH + _payload_length;

    /* Copy the payload into the message buffer */
    memcpy(&_message_buffer[MESSAGE_HEADER_LENGTH],payload_buffer,_payload_length);

    /* Mark the object container as being populated */
    _populated = true;
    
  }

  /**
   * \brief Parses a sequence of bytes into a Sick message
   * \param *message_buffer A well-formed message to be parsed into the class' fields
   */
  template< unsigned int MSG_HEADER_LENGTH, unsigned int MSG_PAYLOAD_MAX_LENGTH, unsigned int MSG_TRAILER_LENGTH >
  void SickMessage< MSG_HEADER_LENGTH, MSG_PAYLOAD_MAX_LENGTH, MSG_TRAILER_LENGTH >::ParseMessage( const uint8_t * const message_buffer ) {

    /* Clear the message container/object */
    Clear(); 

    /* Mark the object as populated */
    _populated = true;    
  }

  /**
   * \brief Get the message as a sequence of well-formed bytes
   * \param *message_buffer Destination buffer for message contents
   */
  template< unsigned int MSG_HEADER_LENGTH, unsigned int MSG_PAYLOAD_MAX_LENGTH, unsigned int MSG_TRAILER_LENGTH >
  void SickMessage< MSG_HEADER_LENGTH, MSG_PAYLOAD_MAX_LENGTH, MSG_TRAILER_LENGTH >::GetMessage( uint8_t * const message_buffer ) const {
    memcpy(message_buffer,_message_buffer,_message_length);
  }

  /**
   * \brief Get the payload contents as a sequence of well-formed bytes
   * \param *payload_buffer Destination buffer for message payload contents
   */
  template< unsigned int MSG_HEADER_LENGTH, unsigned int MSG_PAYLOAD_MAX_LENGTH, unsigned int MSG_TRAILER_LENGTH >
  void SickMessage< MSG_HEADER_LENGTH, MSG_PAYLOAD_MAX_LENGTH, MSG_TRAILER_LENGTH >::GetPayload( uint8_t * const payload_buffer ) const {
    memcpy(payload_buffer,&_message_buffer[MESSAGE_HEADER_LENGTH],_payload_length);
  }

  /** Returns a copy of the payload as a C String */
  template< unsigned int MSG_HEADER_LENGTH, unsigned int MSG_PAYLOAD_MAX_LENGTH, unsigned int MSG_TRAILER_LENGTH >
  void SickMessage< MSG_HEADER_LENGTH, MSG_PAYLOAD_MAX_LENGTH, MSG_TRAILER_LENGTH >::GetPayloadAsCStr( char * const payload_buffer ) const {
    memcpy(payload_buffer,&_message_buffer[MESSAGE_HEADER_LENGTH],_payload_length);
    payload_buffer[_payload_length] = '\0';
  }
  
  /**
   * \brief Get a specified sub-region of the payload buffer
   * \param *payload_sub_buffer Destination buffer for message payload contents
   * \param *start_idx The 0-indexed starting location for copying
   * \param *stop_idx The 0-indexed stopping location for copying
   */
  template< unsigned int MSG_HEADER_LENGTH, unsigned int MSG_PAYLOAD_MAX_LENGTH, unsigned int MSG_TRAILER_LENGTH >
  void SickMessage< MSG_HEADER_LENGTH, MSG_PAYLOAD_MAX_LENGTH, MSG_TRAILER_LENGTH >::GetPayloadSubregion( uint8_t * const payload_sub_buffer,
													  const unsigned int start_idx,
													  const unsigned int stop_idx ) const {
    /* Extract the subregion */
    memcpy(payload_sub_buffer,&_message_buffer[MESSAGE_HEADER_LENGTH+start_idx],stop_idx+1);
  }
  
  /**
   * \brief Reset all internal fields and buffers
   */
  template< unsigned int MSG_HEADER_LENGTH, unsigned int MSG_PAYLOAD_MAX_LENGTH, unsigned int MSG_TRAILER_LENGTH >
  void SickMessage< MSG_HEADER_LENGTH, MSG_PAYLOAD_MAX_LENGTH, MSG_TRAILER_LENGTH >::Clear( ) {

    /* Reset the parent integer variables */
    _message_length = _payload_length = 0;

    /* Clear the message buffer */
    memset(_message_buffer,0,MESSAGE_MAX_LENGTH);

    /* Set the flag indicating this message object/container is empty */
    _populated = false;
  }
  
  /**
   * \brief Print data about this object
   */
  template< unsigned int MSG_HEADER_LENGTH, unsigned int MSG_PAYLOAD_MAX_LENGTH, unsigned int MSG_TRAILER_LENGTH >
  void SickMessage< MSG_HEADER_LENGTH, MSG_PAYLOAD_MAX_LENGTH, MSG_TRAILER_LENGTH >::Print( ) const {

    std::cout << "Payload length: " << GetPayloadLength() << std::endl;
    std::cout << "Message length: " << GetMessageLength() << std::endl;
    std::cout << std::flush;

    std::cout << "Message (hex):" << std::endl;
    std::cout.setf(std::ios::hex,std::ios::basefield);
    for (unsigned int i = 0; i < _message_length; i++) {
      std::cout << (int)_message_buffer[i] << " ";
    }
    std::cout << std::endl << std::flush;

    std::cout << "Message (ASCII):" << std::endl;
    std::cout.setf(std::ios::dec,std::ios::basefield);
    for (unsigned int i = 0; i < _message_length; i++) {
      std::cout << _message_buffer[i] << " ";
    }
    std::cout << std::endl << std::flush;    
  }

  /**
   * \brief A destructor
   */
  template< unsigned int MSG_HEADER_LENGTH, unsigned int MSG_PAYLOAD_MAX_LENGTH, unsigned int MSG_TRAILER_LENGTH >
  SickMessage< MSG_HEADER_LENGTH, MSG_PAYLOAD_MAX_LENGTH, MSG_TRAILER_LENGTH >::~SickMessage() { }
  
} /* namespace SickToolbox */

#endif /* SICK_MESSAGE */
