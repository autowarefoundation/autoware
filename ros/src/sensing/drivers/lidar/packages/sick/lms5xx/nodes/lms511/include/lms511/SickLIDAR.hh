/*!
 * \file SickLIDAR.hh
 * \brief Defines the abstract parent class for defining
 *        a Sick LIDAR device driver.
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

/**
 * \mainpage The Sick LIDAR Matlab/C++ Toolbox 
 * \author Jason C. Derenick and Thomas H. Miller.
 * \version 1.0
 *
 * The Sick LIDAR Matlab/C++ Toolbox
 * Copyright (c) 2008, Jason C. Derenick and Thomas H. Miller
 * All rights reserved.
 *
 * This software is released under a BSD Open-Source License.
 * See http://sicktoolbox.sourceforge.net
 */

#ifndef SICK_LIDAR
#define SICK_LIDAR

/* Definition dependencies */
#include <new>
#include <string>
#include <iomanip>
#include <iostream>
#include <fcntl.h>
#include <pthread.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/time.h>
#include "SickException.hh"

/* Associate the namespace */
namespace SickToolbox {

  /**
   * \class SickLIDAR
   * \brief Provides an abstract parent for all Sick LIDAR devices
   */
  template < class SICK_MONITOR_CLASS, class SICK_MSG_CLASS >
  class SickLIDAR {

  public:
    
    /** The primary constructor */
    SickLIDAR( );

    /** Indicates whether device is initialized */
    bool IsInitialized() { return _sick_initialized; }
    
    /** A virtual destructor */
    virtual ~SickLIDAR( );

  protected:

    /** Sick device file descriptor */
    int _sick_fd;

    /** A flag to indicated whether the device is properly initialized */
    bool _sick_initialized;
    
    /** A pointer to the driver's buffer monitor */
    SICK_MONITOR_CLASS *_sick_buffer_monitor;        

    /** Indicates whether the Sick buffer monitor is running */
    bool _sick_monitor_running;

    /** A method for setting up a general connection */
    virtual void _setupConnection( ) = 0;
    
    /** A method for tearing down a connection to the Sick */
    virtual void _teardownConnection( ) = 0;

    /** Starts the driver listening for messages */
    void _startListening( ) throw( SickThreadException );

    /** Stops the driver from listening */
    void _stopListening( )  throw( SickThreadException );

    /** Indicates whether there is a monitor currently running */
    bool _monitorRunning( ) const { return _sick_monitor_running; }
    
    /** Make the associated file descriptor non blocking */
    void _setBlockingIO( ) const throw ( SickIOException );
    
    /** Make the associated file descriptor non blocking */
    void _setNonBlockingIO( ) const throw ( SickIOException );

    /** Send a message to the Sick LD (allows specifying min time between transmitted bytes) */
    void _sendMessage( const SICK_MSG_CLASS &sick_message, const unsigned int byte_interval ) const
      throw( SickIOException );
    
    /** Acquire the next message from the message container */
    void _recvMessage( SICK_MSG_CLASS &sick_message, const unsigned int timeout_value ) const throw ( SickTimeoutException );

    /** Search the stream for a payload with a particular "header" byte string */
    void _recvMessage( SICK_MSG_CLASS &sick_message,
                       const uint8_t * const byte_sequence,
                       const unsigned int byte_sequence_length,
                       const unsigned int timeout_value ) const throw ( SickTimeoutException );
    
    /** An inline function for computing elapsed time */
    double _computeElapsedTime( const struct timeval &beg_time, const struct timeval &end_time ) const { return ((end_time.tv_sec*1e6)+(end_time.tv_usec))-((beg_time.tv_sec*1e6)+beg_time.tv_usec); }
    
    /** Sends a request to the Sick and acquires looks for the reply */
    virtual void _sendMessageAndGetReply( const SICK_MSG_CLASS &send_message,
                                          SICK_MSG_CLASS &recv_message,
                                          const uint8_t * const byte_sequence,
                                          const unsigned int byte_sequence_length,
                                          const unsigned int byte_interval,
                                          const unsigned int timeout_value,
                                          const unsigned int num_tries ) throw( SickTimeoutException, SickIOException);
    
  };

  /**
   * \brief Initializes the buffer monitor
   */
  template< class SICK_MONITOR_CLASS, class SICK_MSG_CLASS >
  SickLIDAR< SICK_MONITOR_CLASS, SICK_MSG_CLASS >::SickLIDAR( ) :
    _sick_fd(0), _sick_initialized(false), _sick_buffer_monitor(NULL), _sick_monitor_running(false) {

    try {
      /* Attempt to instantiate a new SickBufferMonitor for the device */
      _sick_buffer_monitor = new SICK_MONITOR_CLASS;
    }
    catch ( std::bad_alloc &allocation_exception ) {
      std::cerr << "SickLIDAR::SickLIDAR: Allocation error - " << allocation_exception.what() << std::endl;
    }
    
  }

  /**
   * \brief Destructor tears down buffer monitor
   */
  template< class SICK_MONITOR_CLASS, class SICK_MSG_CLASS >
  SickLIDAR< SICK_MONITOR_CLASS, SICK_MSG_CLASS >::~SickLIDAR( ) {
    
    /* Deallocate the buffer monitor */
    if (_sick_buffer_monitor) {
      delete _sick_buffer_monitor;
    }
    
  }

  /**
   * \brief Activates the buffer monitor for the driver
   */
  template< class SICK_MONITOR_CLASS, class SICK_MSG_CLASS >
  void SickLIDAR< SICK_MONITOR_CLASS, SICK_MSG_CLASS >::_startListening( ) throw( SickThreadException ) {

    /* Try to start the monitor */
    try {
      _sick_buffer_monitor->StartMonitor(_sick_fd);
    }

    /* Handle a thread exception */
    catch(SickThreadException &sick_thread_exception) {
      std::cerr << sick_thread_exception.what() << std::endl;
      throw;
    }

    /* Handle a thread exception */
    catch(...) {
      std::cerr << "SickLIDAR::_startListening: Unknown exception!!!" << std::endl;
      throw;
    }    

    /* Set the flag */
    _sick_monitor_running = true;
    
  }

  /**
   * \brief Activates the buffer monitor for the driver
   */
  template< class SICK_MONITOR_CLASS, class SICK_MSG_CLASS >
  void SickLIDAR< SICK_MONITOR_CLASS, SICK_MSG_CLASS >::_stopListening( ) throw( SickThreadException ) {

    /* Try to start the monitor */
    try {
      _sick_buffer_monitor->StopMonitor();
    }

    /* Handle a thread exception */
    catch(SickThreadException &sick_thread_exception) {
      std::cerr << sick_thread_exception.what() << std::endl;
      throw;
    }

    /* Handle a thread exception */
    catch(...) {
      std::cerr << "SickLIDAR::_stopListening: Unknown exception!!!" << std::endl;
      throw;
    }

    /* Reset the flag */
    _sick_monitor_running = false;
    
  }

  /**
   * \brief A simple method for setting blocking I/O
   */
  template< class SICK_MONITOR_CLASS, class SICK_MSG_CLASS >
  void SickLIDAR< SICK_MONITOR_CLASS, SICK_MSG_CLASS >::_setBlockingIO( ) const throw( SickIOException ) {

    /* Read the flags */
    int fd_flags = 0;
    if((fd_flags = fcntl(_sick_fd,F_GETFL)) < 0) {
      throw SickIOException("SickLIDAR::_setNonBlocking: fcntl failed!");
    }
    
    /* Set the new flags  */
    if(fcntl(_sick_fd,F_SETFL,fd_flags & (~O_NONBLOCK)) < 0) {
      throw SickIOException("SickLIDAR::_setNonBlocking: fcntl failed!");
    }

  }
  
  /**
   * \brief A simple method for setting non-blocking I/O
   */
  template< class SICK_MONITOR_CLASS, class SICK_MSG_CLASS >
  void SickLIDAR< SICK_MONITOR_CLASS, SICK_MSG_CLASS >::_setNonBlockingIO( ) const throw ( SickIOException ) {

    /* Read the flags */
    int fd_flags = 0;
    if((fd_flags = fcntl(_sick_fd,F_GETFL)) < 0) {
      throw SickIOException("SickLIDAR::_setNonBlockingIO: fcntl failed!");
    }
    
    /* Set the new flags */
    if(fcntl(_sick_fd,F_SETFL,fd_flags | O_NONBLOCK) < 0) {
      throw SickIOException("SickLIDAR::_setNonBlockingIO: fcntl failed!");
    }

  }
  
  /**
   * \brief Sends a message to the Sick device
   * \param &sick_message A reference to the well-formed message that is to be sent to the Sick
   * \param byte_interval Minimum time in microseconds between transmitted bytes
   */
  template< class SICK_MONITOR_CLASS, class SICK_MSG_CLASS >
  void SickLIDAR< SICK_MONITOR_CLASS, SICK_MSG_CLASS >::_sendMessage( const SICK_MSG_CLASS &sick_message, const unsigned int byte_interval ) const
    throw( SickIOException ) {

    uint8_t message_buffer[SICK_MSG_CLASS::MESSAGE_MAX_LENGTH] = {0};

    /* Copy the given message and get the message length */
    sick_message.GetMessage(message_buffer);
    unsigned int message_length = sick_message.GetMessageLength();

    /* Check whether a transmission delay between bytes is requested */
    if (byte_interval == 0) {
      
      /* Write the message to the stream */
      if ((unsigned int)write(_sick_fd,message_buffer,message_length) != message_length) {      
        throw SickIOException("SickLIDAR::_sendMessage: write() failed!");
      }

    }
    else {
      
      /* Write the message to the unit one byte at a time */
      for (unsigned int i = 0; i < message_length; i++) {

        /* Write a single byte to the stream */
        if (write(_sick_fd,&message_buffer[i],1) != 1) {
          throw SickIOException("SickLIDAR::_sendMessage: write() failed!");
        }

        /* Some time between bytes (Sick LMS 2xx likes this) */
        usleep(byte_interval);
      }

    }    

  }

  /**
   * \brief Attempt to acquire the latest available message from the device
   * \param &sick_message A reference to the container that will hold the most recent message
   * \param timeout_value The time in secs to wait before throwing a timeout error
   * \return True if a new message was received, False otherwise
   */
  template< class SICK_MONITOR_CLASS, class SICK_MSG_CLASS >
  void SickLIDAR< SICK_MONITOR_CLASS, SICK_MSG_CLASS >::_recvMessage( SICK_MSG_CLASS &sick_message,
                                                                      const unsigned int timeout_value ) const throw ( SickTimeoutException ) {

    /* Timeval structs for handling timeouts */
    struct timeval beg_time, end_time;

    /* Acquire the elapsed time since epoch */
    gettimeofday(&beg_time,NULL);
    
    /* Check the shared object */
    while(!_sick_buffer_monitor->GetNextMessageFromMonitor(sick_message)) {    
      
      /* Sleep a little bit */
      usleep(1000);
    
      /* Check whether the allowed time has expired */
      gettimeofday(&end_time,NULL);    
      if (_computeElapsedTime(beg_time,end_time) > timeout_value) {
        throw SickTimeoutException("SickLIDAR::_recvMessage: Timeout occurred!");
      }
      
    }
    
  }

  /**
   * \brief Attempt to acquire a message having a payload beginning w/ the given byte sequence
   * \param &sick_message A reference to the container that will hold the most recent message
   * \param *byte_sequence The byte sequence that is expected to lead off the payload in the packet (e.g. service codes, etc...)
   * \param byte_sequence_length The number of bytes in the given byte_sequence
   * \param timeout_value The time in usecs to wait before throwing a timeout error
   * \return True if a new message was received, False otherwise
   *
   * NOTE: This method is intended to be a helper for _sendMessageAndGetReply
   */
  template< class SICK_MONITOR_CLASS, class SICK_MSG_CLASS >
  void SickLIDAR< SICK_MONITOR_CLASS, SICK_MSG_CLASS >::_recvMessage( SICK_MSG_CLASS &sick_message,
                                                                      const uint8_t * const byte_sequence,
                                                                      const unsigned int byte_sequence_length,
                                                                      const unsigned int timeout_value ) const throw( SickTimeoutException ) {

    /* Define a buffer */
    uint8_t payload_buffer[SICK_MSG_CLASS::MESSAGE_PAYLOAD_MAX_LENGTH];
    
    /* Timeval structs for handling timeouts */
    struct timeval beg_time, end_time;    
    
    /* A container for the message */
    SICK_MSG_CLASS curr_message;
    
    /* Get the elapsed time since epoch */
    gettimeofday(&beg_time,NULL);

    /* Check until it is found or a timeout */
    for(;;) {
      
      /* Attempt to acquire the message */
      unsigned int i = 0;
      if (_sick_buffer_monitor->GetNextMessageFromMonitor(curr_message)) {

        /* Extract the payload subregion */
        curr_message.GetPayloadSubregion(payload_buffer,0,byte_sequence_length-1);

        /* Match the byte sequence */
        for (i=0; (i < byte_sequence_length) && (payload_buffer[i] == byte_sequence[i]); i++);

        /* Our message was found! */
        if (i == byte_sequence_length) {
          sick_message = curr_message;
          break;
        }

        std::cout << "WARNING: Found unexpected message:" << std::endl;
        curr_message.Print();
      }
      
      /* Sleep a little bit */
      usleep(1000);     

      /* Check whether the allowed time has expired */
      gettimeofday(&end_time,NULL);        
      if (_computeElapsedTime(beg_time,end_time) > timeout_value) {
              throw SickTimeoutException();
      }      
      
    }

  }

  /**
   * \param sick_send_frame A sick frame to be sent to the LMS
   * \param sick_receive_frame A sick frame to hold the response (expected or unexpected) of the LMS
   * \param num_tries The number of times to send the frame in the event the LMS fails to reply
   * \param timeout The epoch to wait before considering a sent frame lost
   * \return True if the message was sent and the expected reply was received
   */
  template< class SICK_MONITOR_CLASS, class SICK_MSG_CLASS >
  void SickLIDAR< SICK_MONITOR_CLASS, SICK_MSG_CLASS >::_sendMessageAndGetReply( const SICK_MSG_CLASS &send_message,
                                                                                 SICK_MSG_CLASS &recv_message,
                                                                                 const uint8_t * const byte_sequence,
                                                                                 const unsigned int byte_sequence_length,
                                                                                 const unsigned int byte_interval,
                                                                                 const unsigned int timeout_value,
                                                                                 const unsigned int num_tries )
                                                                                 throw( SickTimeoutException, SickIOException ) {
    
    /* Send the message for at most num_tries number of times */
    for(unsigned int i = 0; i < num_tries; i++) {

      try {

        /* Send the frame to the unit */
        _sendMessage(send_message,byte_interval);

        /* Wait for the reply! */
        _recvMessage(recv_message,byte_sequence,byte_sequence_length,timeout_value);

        /* message was found! */
        break;

      }

      /* Handle a timeout! */
      catch (SickTimeoutException &sick_timeout) {

        /* Check if it was found! */
        if (i == num_tries - 1) {
          throw SickTimeoutException("SickLIDAR::_sendMessageAndGetReply: Attempted max number of tries w/o success!");
        }

        /* Display the number of tries remaining! */
        std::cerr << sick_timeout.what() << " " << num_tries - i - 1  << " tries remaining" <<  std::endl;

      }
      
      /* Handle write buffer exceptions */
      catch (SickIOException &sick_io_error) {
        std::cerr << sick_io_error.what() << std::endl;
        throw;
      }
      
      /* A safety net */
      catch (...) {
        std::cerr << "SickLIDAR::_sendMessageAndGetReply: Unknown exception!!!" << std::endl;
        throw;
      }
      
    }
    
  }
  
} /* namespace SickToolbox */

#endif /* SICK_LIDAR */
