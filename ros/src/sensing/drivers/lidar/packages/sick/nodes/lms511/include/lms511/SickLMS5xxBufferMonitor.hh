/*!
 * \file SickLMS5xxBufferMonitor.hh
 * \brief Defines a class for monitoring the receive
 *        buffer when interfacing w/ a Sick LMS 5xx
 *        laser range finder unit.
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

#ifndef SICK_LMS_5xx_BUFFER_MONITOR_HH
#define SICK_LMS_5xx_BUFFER_MONITOR_HH

// Note: In contrast to the LMS2xx code, this value is used for receives only, not for sending.
#define DEFAULT_SICK_LMS_5XX_BYTE_TIMEOUT         (100000)  ///< Max allowable time between consecutive bytes

/* Definition dependencies */
#include "SickLMS5xxMessage.hh"
#include "SickBufferMonitor.hh"
#include "SickException.hh"

/* Associate the namespace */
namespace SickToolbox {

  /*!
   * \brief A class for monitoring the receive buffer when interfacing with a Sick LD LIDAR
   */
  class SickLMS5xxBufferMonitor : public SickBufferMonitor< SickLMS5xxBufferMonitor, SickLMS5xxMessage > {

  public:

    /** A standard constructor */
    SickLMS5xxBufferMonitor( );

    /** A method for extracting a single message from the stream */
    void GetNextMessageFromDataStream( SickLMS5xxMessage &sick_message ) throw( SickIOException );

    /** A standard destructor */
    ~SickLMS5xxBufferMonitor( );

  private:

    /* A utility function for flushing the receive buffer */
    void _flushTCPRecvBuffer( ) const throw ( SickIOException );
    
  };
    
} /* namespace SickToolbox */

#endif /* SICK_LMS_5xx_BUFFER_MONITOR_HH */
