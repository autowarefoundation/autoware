/*!
 * \file SickException.hh
 * \brief Contains some simple exception classes.
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

#ifndef SICK_EXCEPTION
#define SICK_EXCEPTION

/* Definition dependencies */
#include <string>
#include <exception>

/* Associate the namespace */
namespace SickToolbox {

  /** \class SickException
   *  \brief Provides a base exception class from
   *         which to derive other Sick exceptions
   */
  class SickException : std::exception {

  public:

    /**
     * \brief A standard constructor
     * \param general_str A descriptive "general" string
     */
    SickException( const std::string general_str ) { 
      _detailed_msg = general_str;
    }

    /**
     * \brief A standard constructor
     * \param general_str A descriptive "general" string
     * \param detailed_str A more detailed description
     */
    SickException( const std::string general_str, const std::string detailed_str ) {
      _detailed_msg = general_str + " " + detailed_str;
    }
    
    /**
     * \brief From the standard exception library 
     */
    virtual const char* what( ) const throw() {
      return _detailed_msg.c_str();
    }

    /**
     * \brief A destructor
     */
    ~SickException() throw() {}

  private:

    /** The string identifier */
    std::string _detailed_msg;
    
  };

  /**
   * \class SickTimeoutException
   * \brief Makes handling timeouts much easier
   */
  class SickTimeoutException : public SickException {

  public:

    /**
     * \brief A constructor
     */
    SickTimeoutException() :
      SickException("A Timeout Occurred!") { }

    /**
     * \brief A constructor
     * \param detailed_str A more detailed description
     */
    SickTimeoutException( const std::string detailed_str ) :
      SickException("A Timeout Occurred -",detailed_str) { }
    
    /**
     * \brief A destructor
     */
    ~SickTimeoutException() throw() { }
    
  };
  
  /**
   * \class SickIOException
   * \brief Thrown instance where the driver can't
   *        read,write,drain,flush,... the buffers
   */
  class SickIOException : public SickException {

  public:

    /**
     * \brief A constructor
     */
    SickIOException() :
      SickException("ERROR: I/O exception!") { }

    /**
     * \brief Another constructor
     * \param detailed_str A more detailed description
     */
    SickIOException( const std::string detailed_str ) :
      SickException("ERROR: I/O exception -",detailed_str) { }
    
    /**
     * \brief A destructor
     */
    ~SickIOException() throw() { }
    
  };

  /**
   * \class SickBadChecksumException
   * \brief Thrown when a received message has an
   *        invalid checksum
   */
  class SickBadChecksumException : public SickException {

  public:

    /**
     * \brief A constructor
     */
    SickBadChecksumException() :
      SickException("ERROR: Bad Checksum!") { }

    /**
     * \brief Another constructor
     * \param detailed_str A more detailed description
     */
    SickBadChecksumException( const std::string detailed_str ) :
      SickException("ERROR: Bad Checksum -",detailed_str) { }
    
    /**
     * \brief A destructor
     */
    ~SickBadChecksumException() throw() { }
    
  };

  /**
   * \class SickThreadException
   * \brief Thrown when error occurs during thread
   *        initialization, and uninitialization
   */
  class SickThreadException : public SickException {

  public:

    /**
     * \brief A constructor
     */
    SickThreadException() :
      SickException("ERROR: Sick thread exception!") { }

    /**
     * \brief Another constructor
     * \param detailed_str A more detailed description
     */
    SickThreadException( const std::string detailed_str ) :
      SickException("ERROR: Sick thread exception -",detailed_str) { }
    
    /**
     * \brief A destructor
     */
    ~SickThreadException() throw() { }
    
  };
  
  /**
   * \class SickConfigException
   * \brief Thrown when the driver detects (or the Sick reports)
   *        an invalid config
   */
  class SickConfigException : public SickException {

  public:

    /**
     * \brief A constructor
     */
    SickConfigException() :
      SickException("ERROR: Config exception!") { }

    /**
     * \brief Another constructor
     * \param detailed_str A more detailed description
     */
    SickConfigException( const std::string detailed_str ) :
      SickException("ERROR: Config exception -",detailed_str) { }
    
    /**
     * \brief A destructor
     */
    ~SickConfigException() throw() { }
    
  };

  /**
   * \class SickErrorException
   * \brief Thrown when Sick returns an error code
   *        or an unexpected response.
   */
  class SickErrorException : public SickException {
    
    public:

    /**
     * \brief A constructor
     */
    SickErrorException() :
      SickException("ERROR: Sick returned error code!") { };

    /**
     * \brief Another constructor
     * \param detailed_str A more detailed description
     */
    SickErrorException( const std::string detailed_str ) :
      SickException("ERROR: Sick error -", detailed_str) { } 

    /**
     * \brief A destructor
     */
    ~SickErrorException() throw() { }

  };
} /* namespace SickToolbox */
  
#endif /* SICK_EXCEPTION */
