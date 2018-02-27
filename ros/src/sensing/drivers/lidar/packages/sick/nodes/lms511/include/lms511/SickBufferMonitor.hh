/*!
 * \file SickBufferMonitor.hh
 * \brief Defines an abstract class for interfacing with a Sick LIDAR.
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

#ifndef SICK_BUFFER_MONITOR
#define SICK_BUFFER_MONITOR

/* Dependencies */
#include <iostream>
#include <pthread.h>
#include "SickException.hh"
#include <unistd.h>

/* Associate the namespace */
namespace SickToolbox {

  /**
   * \class SickBufferMonitor
   */
  template < class SICK_MONITOR_CLASS, class SICK_MSG_CLASS >
  class SickBufferMonitor {

  public:

    /** A standard constructor */
    SickBufferMonitor( SICK_MONITOR_CLASS * const monitor_instance ) throw( SickThreadException );

    /** A method for setting the target data stream */
    void SetDataStream( const unsigned int sick_fd ) throw( SickThreadException );
    
    /** Start the buffer monitor for the device */
    void StartMonitor( const unsigned int sick_fd ) throw( SickThreadException );

    /** Acquire the most recent message buffered by the monitor */
    bool GetNextMessageFromMonitor( SICK_MSG_CLASS &sick_message ) throw( SickThreadException );
    
    /** Stop the buffer monitor for the device */
    void StopMonitor( ) throw( SickThreadException );

    /** Locks access to the data stream */
    void AcquireDataStream( ) throw( SickThreadException );

    /** Acquire the next message from raw byte stream */
    void GetNextMessageFromDataStream( SICK_MSG_CLASS &sick_message );
    
    /** Unlock access to the data stream */
    void ReleaseDataStream( ) throw( SickThreadException );

    /** A standard destructor */
    ~SickBufferMonitor( ) throw( SickThreadException );

  protected:

    /** Sick data stream file descriptor */
    unsigned int _sick_fd;   
    
    /** Reads n bytes into the destination buffer */
    void _readBytes( uint8_t * const dest_buffer, const int num_bytes_to_read, const unsigned int timeout_value = 0 ) const throw ( SickTimeoutException, SickIOException );       
    
  private:

    /** The current monitor instance */
    SICK_MONITOR_CLASS *_sick_monitor_instance;

    /** A flag to indicate the monitor should continue running */
    bool _continue_grabbing;
    
    /** Buffer monitor thread ID */
    pthread_t _monitor_thread_id;

    /** A mutex for guarding the message container */
    pthread_mutex_t _container_mutex;

    /** A mutex for locking the data stream */
    pthread_mutex_t _stream_mutex;
    
    /** A container to hold the most recent message */
    SICK_MSG_CLASS _recv_msg_container;      

    /** Locks access to the message container */
    void _acquireMessageContainer( ) throw( SickThreadException );

    /** Unlocks access to the message container */
    void _releaseMessageContainer( ) throw( SickThreadException );   

    /** Entry point for the monitor thread */
    static void * _bufferMonitorThread( void * thread_args );    
    
  };

  /**
   * \brief Primary constructor
   * \param device_instance A pointer to the current driver instance
   */
  template < class SICK_MONITOR_CLASS, class SICK_MSG_CLASS >
  SickBufferMonitor< SICK_MONITOR_CLASS, SICK_MSG_CLASS >::SickBufferMonitor( SICK_MONITOR_CLASS * const monitor_instance ) throw( SickThreadException ) :
    _sick_monitor_instance(monitor_instance), _continue_grabbing(true), _monitor_thread_id(0) {
    
    /* Initialize the shared message buffer mutex */
    if (pthread_mutex_init(&_container_mutex,NULL) != 0) {
      throw SickThreadException("SickBufferMonitor::SickBufferMonitor: pthread_mutex_init() failed!");
    }

    /* Initialize the shared data stream mutex */
    if (pthread_mutex_init(&_stream_mutex,NULL) != 0) {
      throw SickThreadException("SickBufferMonitor::SickBufferMonitor: pthread_mutex_init() failed!");
    }
    
  }

  /**
   * \brief A method for setting/changing the current data stream
   * \param sick_fd The data stream file descriptor
   */
  template < class SICK_MONITOR_CLASS, class SICK_MSG_CLASS >
  void SickBufferMonitor< SICK_MONITOR_CLASS, SICK_MSG_CLASS >::SetDataStream( const unsigned int sick_fd ) throw ( SickThreadException ) {

    try {
    
      /* Attempt to acquire the data stream */
      AcquireDataStream();
      
      /* Assign the data stream fd */
      _sick_fd = sick_fd;
      
      /* Attempt to release the data stream */
      ReleaseDataStream();
      
    }

    /* Handle thread exception */
    catch(SickThreadException &sick_thread_exception) {
      std::cerr << sick_thread_exception.what() << std::endl;
    }
    
    /* A safety net */
    catch(...) {
      std::cerr << "SickBufferMonitor::SetDataStream: Unknown exception!" << std::endl;
      throw;
    }
    
  }
  
  /**
   * \brief Creates and starts the buffer monitor thread
   * \return True upon success, False otherwise
   */
  template < class SICK_MONITOR_CLASS, class SICK_MSG_CLASS >
  void SickBufferMonitor< SICK_MONITOR_CLASS, SICK_MSG_CLASS >::StartMonitor( const unsigned int sick_fd ) throw ( SickThreadException ) {

    /* Assign the fd associated with the data stream */
    _sick_fd = sick_fd;
    
    /* Start the buffer monitor */
    if (pthread_create(&_monitor_thread_id,NULL,SickBufferMonitor< SICK_MONITOR_CLASS, SICK_MSG_CLASS >::_bufferMonitorThread,_sick_monitor_instance) != 0) {
      throw SickThreadException("SickBufferMonitor::StartMonitor: pthread_create() failed!");
    }

    /* Set the flag to continue grabbing data */
    _continue_grabbing = true;
    
  }

  /**
   * \brief Checks the message container for the next available Sick message
   * \param &sick_message The message object that is to be populated with the results
   * \return True if the current contents were acquired, false otherwise
   */
  template < class SICK_MONITOR_CLASS, class SICK_MSG_CLASS >
  bool SickBufferMonitor< SICK_MONITOR_CLASS, SICK_MSG_CLASS >::GetNextMessageFromMonitor( SICK_MSG_CLASS &sick_message ) throw( SickThreadException ) {

    bool acquired_message = false;

    try {
    
      /* Acquire a lock on the message buffer */
      _acquireMessageContainer();

      /* Check whether the object is populated */
      if (_recv_msg_container.IsPopulated()) {

	/* Copy the shared message */
	sick_message = _recv_msg_container;
	_recv_msg_container.Clear();
	
	/* Set the flag indicating success */
	acquired_message = true;      
      }

      /* Release message container */
      _releaseMessageContainer();      

    }

    /* Handle a thread exception */
    catch(SickThreadException &sick_thread_exception) {
      std::cerr << sick_thread_exception.what() << std::endl;
      throw;
    }

    /* Handle an unknown exception */
    catch(...) {
      std::cerr << "SickBufferMonitor::CheckMessageContainer: Unknown exception!" << std::endl;
      throw;
    }
    
    /* Return the flag */
    return acquired_message;    
  }
  
  /**
   * \brief Cancels the buffer monitor thread
   * \return True if the thread was properly canceled, false otherwise
   */
  template < class SICK_MONITOR_CLASS, class SICK_MSG_CLASS >
  void SickBufferMonitor< SICK_MONITOR_CLASS, SICK_MSG_CLASS >::StopMonitor( ) throw ( SickThreadException ) {

    try {

      /* Return results from the thread */
      void *monitor_result = NULL;     
      
      /* Tell the thread to quit working */
      AcquireDataStream();      
      _continue_grabbing = false;
      ReleaseDataStream();

      /* Wait for the buffer monitor to exit */
      if (pthread_join(_monitor_thread_id,&monitor_result) != 0) {
      	throw SickThreadException("SickBufferMonitor::StopMonitor: pthread_join() failed!");      
      }

    }

    /* Handle thread exception */
    catch(SickThreadException &sick_thread_exception) {
      std::cerr << sick_thread_exception.what() << std::endl;
    }
    
    /* A safety net */
    catch(...) {
      std::cerr << "SickBufferMonitor::StopMonitor: Unknown exception!" << std::endl;
      throw;
    }
    
  }

  /**
   * \brief Acquires a lock on the data stream
   */ 
  template < class SICK_MONITOR_CLASS, class SICK_MSG_CLASS >
  void SickBufferMonitor< SICK_MONITOR_CLASS, SICK_MSG_CLASS >::AcquireDataStream( ) throw( SickThreadException ) {

    /* Attempt to lock the stream mutex */
    if (pthread_mutex_lock(&_stream_mutex) != 0) {
      throw SickThreadException("SickBufferMonitor::AcquireDataStream: pthread_mutex_lock() failed!");
    }
    
  }

  /**
   * \brief Releases a lock on the data stream
   */ 
  template < class SICK_MONITOR_CLASS, class SICK_MSG_CLASS >
  void SickBufferMonitor< SICK_MONITOR_CLASS, SICK_MSG_CLASS >::ReleaseDataStream( ) throw( SickThreadException ) {

    /* Attempt to lock the stream mutex */
    if (pthread_mutex_unlock(&_stream_mutex) != 0) {
      throw SickThreadException("SickBufferMonitor::ReleaseDataStream: pthread_mutex_unlock() failed!");
    }
    
  }
  
  /**
   * \brief The destructor (kills the mutex)
   */
  template < class SICK_MONITOR_CLASS, class SICK_MSG_CLASS >
  SickBufferMonitor< SICK_MONITOR_CLASS, SICK_MSG_CLASS >::~SickBufferMonitor( ) throw( SickThreadException ) {

    /* Destroy the message container mutex */
    if (pthread_mutex_destroy(&_container_mutex) != 0) {
      throw SickThreadException("SickBufferMonitor::~SickBufferMonitor: pthread_mutex_destroy() failed!");
    }

    /* Destroy the data stream container mutex */
    if (pthread_mutex_destroy(&_stream_mutex) != 0) {
      throw SickThreadException("SickBufferMonitor::~SickBufferMonitor: pthread_mutex_destroy() failed!");
    }
    
  }

  /**
   * \brief Locks access to the message container
   */
  template < class SICK_MONITOR_CLASS, class SICK_MSG_CLASS >
  void SickBufferMonitor< SICK_MONITOR_CLASS, SICK_MSG_CLASS >::_acquireMessageContainer( ) throw( SickThreadException ) {

    /* Lock the mutex */
    if (pthread_mutex_lock(&_container_mutex) != 0) {
      throw SickThreadException("SickBufferMonitor::_acquireMessageContainer: pthread_mutex_lock() failed!");
    }	
  
  }

  /**
   * \brief Unlocks access to the message container
   */
  template < class SICK_MONITOR_CLASS, class SICK_MSG_CLASS >
  void SickBufferMonitor< SICK_MONITOR_CLASS, SICK_MSG_CLASS >::_releaseMessageContainer( ) throw( SickThreadException ) {

    /* Unlock the mutex */
    if (pthread_mutex_unlock(&_container_mutex) != 0) {
      throw SickThreadException("SickBufferMonitor::_releaseMessageContainer: pthread_mutex_unlock() failed!");
    }	
    
  }

  /**
   * \brief Attempt to read a certain number of bytes from the stream
   * \param *dest_buffer A pointer to the destination buffer
   * \param num_bytes_to_read The number of bytes to read into the buffer
   * \param timeout_value The number of microseconds allowed between subsequent bytes in a message
   * \return True if the number of requested bytes were successfully read
   */
  template< class SICK_MONITOR_CLASS, class SICK_MSG_CLASS >
  void SickBufferMonitor< SICK_MONITOR_CLASS, SICK_MSG_CLASS >::_readBytes( uint8_t * const dest_buffer, const int num_bytes_to_read, const unsigned int timeout_value ) const
    throw ( SickTimeoutException, SickIOException ) {
    
    /* Some helpful variables */
    int num_bytes_read = 0;
    int total_num_bytes_read = 0;
    int num_active_files = 0;
    
    struct timeval timeout_val;                     // This structure will be used for setting our timeout values
    fd_set file_desc_set;                           // File descriptor set for monitoring I/O    
    
    /* Attempt to fetch the bytes */
    while ( total_num_bytes_read < num_bytes_to_read ) {
      
      /* Initialize and set the file descriptor set for select */
      FD_ZERO(&file_desc_set);
      FD_SET(_sick_fd,&file_desc_set);
      
      /* Setup the timeout structure */
      memset(&timeout_val,0,sizeof(timeout_val));   // Initialize the buffer
      timeout_val.tv_usec = timeout_value;          // Wait for specified time before throwing a timeout

      /* Wait for the OS to tell us that data is waiting! */
      num_active_files = select(getdtablesize(),&file_desc_set,0,0,(timeout_value > 0) ? &timeout_val : 0);
      
      /* Figure out what to do based on the output of select */
      if (num_active_files > 0) {
	
  	/* A file is ready for reading!
  	 *
  	 * NOTE: The following conditional is just a sanity check. Since
  	 *       the file descriptor set only contains the sick device fd,
  	 *       it likely unnecessary to use FD_ISSET
  	 */
  	if (FD_ISSET(_sick_fd,&file_desc_set)) {
	  
  	  /* Read a single byte from the stream! */
  	  num_bytes_read = read(_sick_fd,&dest_buffer[total_num_bytes_read],1);

  	  /* Decide what to do based on the output of read */
  	  if (num_bytes_read > 0) { //Update the number of bytes read so far
  	    total_num_bytes_read += num_bytes_read;
  	  }
  	  else {
  	    /* If this happens, something is wrong */
  	    throw SickIOException("SickBufferMonitor::_readBytes: read() failed!");
  	  }	  
	  
  	}
	
      }
      else if (num_active_files == 0) {
	
	/* A timeout has occurred! */
	throw SickTimeoutException("SickBufferMonitor::_readBytes: select() timeout!");	

      }
      else {
	
	/* An error has occurred! */
	throw SickIOException("SickBufferMonitor::_readBytes: select() failed!");	

      }
      
    }
    
  }
  
  /**
   * \brief The monitor thread
   * \param *args The thread arguments
   */
  template < class SICK_MONITOR_CLASS, class SICK_MSG_CLASS >
  void * SickBufferMonitor< SICK_MONITOR_CLASS, SICK_MSG_CLASS >::_bufferMonitorThread( void * thread_args ) {
    
    /* Declare a Sick LD receive object */
    SICK_MSG_CLASS curr_message;
    
    /* Acquire the Sick device instance */
    SICK_MONITOR_CLASS *buffer_monitor = (SICK_MONITOR_CLASS *)thread_args;

    /* The main thread control loop */
    for (;;) {

      try {

	/* Reset the sick message object */
 	curr_message.Clear();	

 	/* Acquire the most recent message */
	buffer_monitor->AcquireDataStream();	  
	
	if (!buffer_monitor->_continue_grabbing) { // should the thread continue grabbing
	  buffer_monitor->ReleaseDataStream();
	  break;
	}

	buffer_monitor->GetNextMessageFromDataStream(curr_message);
	buffer_monitor->ReleaseDataStream();
	
	/* Update message container contents */
	buffer_monitor->_acquireMessageContainer();	
	buffer_monitor->_recv_msg_container = curr_message;
 	buffer_monitor->_releaseMessageContainer();

      }

      /* Make sure there wasn't a serious error reading from the buffer */
      catch(SickIOException &sick_io_exception) {
	std::cerr << sick_io_exception.what() << std::endl;
      }

      /* Catch any thread exceptions */
      catch(SickThreadException &sick_thread_exception) {
	std::cerr << sick_thread_exception.what() << std::endl;
      }
      
      /* A failsafe */
      catch(...) {
	std::cerr << "SickBufferMonitor::_bufferMonitorThread: Unknown exception!" << std::endl;
      }

      /* sleep a bit! */
      usleep(1000);
      
    }    

    /* Thread is done */
    return NULL;
    
  }
  
} /* namespace SickToolbox */

#endif /* SICK_BUFFER_MONITOR */
