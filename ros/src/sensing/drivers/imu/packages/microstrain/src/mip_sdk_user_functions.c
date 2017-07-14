/**
 * @file    my_user_functions.c 
 * @author  Brian S. Bingham
 * @version 0.1
 *
 * @brief Target(Linux)-Specific Functions Required by the MIP SDK
 *
 * Implementation of mip_sdk_user_functions.c prototypes for Linux
 *
 * External dependencies:
 * mip.h
 * 
 */

/* -- Includes -- */
#include "mip_sdk_user_functions.h"


/**
 *
 * @fn
 * u16 purge(ComPortHandle comPortHandle)
 *
 * @section DESCRIPTION
 * Target-Specific communications port purge. 
 *
 * @section DETAILS
 *
 * @param [in] int comPortHandle  - port number (as recognized by the 
 *                                                          operating system.)
 *
 * @retval MIP_USER_FUNCTION_ERROR  When there is a problem purging the 
 *                                                                     port.\n
 * @retval MIP_USER_FUNCTION_OK     Purge was successful.\n
 *
 */

u16 purge(ComPortHandle comPortHandle){

 if (tcflush(comPortHandle,TCIOFLUSH)==-1){

  printf("flush failed\n");
  return MIP_USER_FUNCTION_ERROR;

 }

 return MIP_USER_FUNCTION_OK;

}

/**
 * @fn
 * u16 mip_sdk_port_open(void *port_handle, int port_num, int baudrate)
 *
 * @section DESCRIPTION
 * Target-Specific communications port open. 
 *
 * @section DETAILS
 *
 * @param [out] void *port_handle - target-specific port handle pointer (user needs to allocate memory for this)
 * @param [in] string port_num       - port number (as recognized by the operating system.)
 * @param [in] int baudrate       - baudrate of the com port.
 *
 * @retval MIP_USER_FUNCTION_ERROR  When there is a problem opening the port.\n
 * @retval MIP_USER_FUNCTION_OK     The open was successful.\n
 */

u16 mip_sdk_port_open(void **port_handle, const char *portstr, int baudrate)
{
 char port_name[100]  = {0};
 static ComPortHandle local_port_handle;
 int hardware_bit_baud, status;
 struct termios options;

 // Copy portstr argument to port_name
 strcat(port_name,portstr);
 //printf("Attempting to open port: %s\n",port_name);
 //Attempt to open the specified port
 local_port_handle = open(port_name, O_RDWR | O_NOCTTY);

 //Check for an invalid handle
 if(local_port_handle == -1)
 {
   printf("Unable to open com Port %s\n Errno = %i\n", port_name, errno);

  return MIP_USER_FUNCTION_ERROR;
 }
 //printf("Port: %s opened successfully.\n",port_name);
 
 //Convert specified baud to hardware specific value
 switch (baudrate)
 {
 case 0:
  hardware_bit_baud = B0;
  break;

 case 50:
  hardware_bit_baud = B50;
  break;

 case 75:
  hardware_bit_baud = B75;
  break;

 case 110:
  hardware_bit_baud = B110;
  break;

 case 134:
  hardware_bit_baud = B134;
  break;

 case 150:
  hardware_bit_baud = B150;
  break;

 case 200:
  hardware_bit_baud = B200;
  break;

 case 300:
  hardware_bit_baud = B300;
  break;

 case 600:
  hardware_bit_baud = B600;
  break;

 case 1200:
  hardware_bit_baud = B1200;
  break;

 case 1800:
  hardware_bit_baud = B1800;
  break;

 case 2400:
  hardware_bit_baud = B2400;
  break;

 case 4800:
  hardware_bit_baud = B4800;
  break;

 case 9600:
  hardware_bit_baud = B9600;
  break;

 case 19200:
  hardware_bit_baud = B19200;
  break;

 case 38400:
  hardware_bit_baud = B38400;
  break;

# ifdef B7200
 case 7200:
  hardware_bit_baud = B7200;
  break;

# endif

# ifdef B14400
 case 14400:
  hardware_bit_baud = B14400;
  break;

# endif

# ifdef B57600
 case 57600:
  hardware_bit_baud = B57600;
  break;

# endif

# ifdef B115200
 case 115200:
  hardware_bit_baud = B115200;
  break;

# endif

# ifdef B230400
 case 230400:
  hardware_bit_baud = B230400;
  break;

# endif

# ifdef B460800
 case 460800:
  hardware_bit_baud = B460800;
  break;

# endif

# ifdef B500000
 case 500000:
  hardware_bit_baud = B500000;
  break;

# endif

# ifdef B576000
 case 576000:
  hardware_bit_baud = B576000;
  break;

# endif

# ifdef B921600
 case 921600:
  hardware_bit_baud = B921600;
  break;

# endif

# ifdef B1000000
 case 1000000:
  hardware_bit_baud = B1000000;
  break;

# endif

# ifdef B1152000
 case 1152000:
  hardware_bit_baud = B1152000;
  break;

# endif

# ifdef B2000000
 case 2000000:
  hardware_bit_baud = B2000000;
  break;

# endif

# ifdef B3000000
 case 3000000:
  hardware_bit_baud = B3000000;
  break;

# endif

# ifdef B3500000
 case 3500000:
  hardware_bit_baud = B3500000;
  break;

# endif

# ifdef B4000000
 case 4000000:
  hardware_bit_baud = B4000000;
  break;

# endif
 //Unsupported baud specified
 default:
  printf("Unsupported baud specified\n");
  return MIP_USER_FUNCTION_ERROR;

 }

 //Get the current settings for the port...
 tcgetattr(local_port_handle, &options);

 //set the baud rate
 cfsetospeed(&options, hardware_bit_baud);
 cfsetispeed(&options, hardware_bit_baud);

 //set the number of data bits.
 options.c_cflag &= ~CSIZE; // Mask the character size bits
 options.c_cflag |= CS8;

 //set the number of stop bits to 1
 options.c_cflag &= ~CSTOPB;

 //Set parity to None
 options.c_cflag &=~PARENB;

 //set for non-canonical (raw processing, no echo, etc.)
 options.c_iflag = IGNPAR; // ignore parity check close_port(int
 options.c_oflag = 0; // raw output
 options.c_lflag = 0; // raw input

 //Time-Outs -- won't work with NDELAY option in the call to open
 options.c_cc[VMIN] = 0;  // block reading until RX x characers. If x = 0, 
               // it is non-blocking.
 options.c_cc[VTIME] = 1;  // Inter-Character Timer -- i.e. timeout= x*.1 s

 //Set local mode and enable the receiver
 options.c_cflag |= (CLOCAL | CREAD);

 //purge serial port buffers
 if(purge(local_port_handle) != MIP_USER_FUNCTION_OK){
  printf("Flushing old serial buffer data failed\n");
  return MIP_USER_FUNCTION_ERROR;
 }
 //attempt to apply settings
 status=tcsetattr(local_port_handle, TCSANOW, &options); 

 if (status != 0){ //For error message

  printf("Configuring comport failed\n");
  return MIP_USER_FUNCTION_ERROR;

 }

 //Purge serial port buffers
 if(purge(local_port_handle) != MIP_USER_FUNCTION_OK){
  printf("Post configuration serial buffer flush failed\n");
  return MIP_USER_FUNCTION_ERROR;
 }

 //assign external pointer to port handle pointer
 *port_handle = &local_port_handle;

 return MIP_USER_FUNCTION_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_sdk_port_close(void *port_handle)
//
//! @section DESCRIPTION
//! Target-Specific port close function 
//
//! @section DETAILS
//!
//! @param [in] void *port_handle - target-specific port handle pointer (user needs to allocate memory for this)
//
//! @retval MIP_USER_FUNCTION_ERROR  When there is a problem closing the port.\n
//! @retval MIP_USER_FUNCTION_OK     The close was successful.\n
//
//! @section NOTES
//! 
//! The user should copy the \c mip_sdk_user_functions.c file to their project directory and\n
//! edit it as needed to support their target operating system.
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_sdk_port_close(void *port_handle)
{
 int local_port_handle = *((int *)port_handle);
 
 if(port_handle == NULL)
  return MIP_USER_FUNCTION_ERROR;
 
 //Close the serial port
 close(local_port_handle);
 
 return MIP_USER_FUNCTION_OK; 
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_sdk_port_write(void *port_handle, u8 *buffer, u32 num_bytes, u32 *bytes_written, u32 timeout_ms)
//
//! @section DESCRIPTION
//! Target-Specific Port Write Function. 
//
//! @section DETAILS
//!
//! @param [in]  void *port_handle  - target-specific port handle pointer (user needs to allocate memory for this)
//! @param [in]  u8 *buffer         - buffer containing num_bytes of data
//! @param [in]  u32 num_bytes      - the number of bytes to write to the port
//! @param [out] u32 *bytes_written - the number of bytes actually written to the port
//! @param [in]  u32 timeout_ms     - the write timeout
//
//! @retval MIP_USER_FUNCTION_ERROR  When there is a problem communicating with the port.\n
//! @retval MIP_USER_FUNCTION_OK     The write was successful.\n
//
//! @section NOTES
//! 
//! The user should copy the \c mip_sdk_user_functions.c file to their project directory and\n
//! edit it as needed to support their target operating system.
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_sdk_port_write(void *port_handle, u8 *buffer, u32 num_bytes, u32 *bytes_written, u32 timeout_ms)
{
 int local_port_handle = *((int *)port_handle);
 static char print_once = 0;

 int local_bytes_written = write(local_port_handle, buffer, num_bytes); 

 if(local_bytes_written == -1)
  return MIP_USER_FUNCTION_ERROR;

 *bytes_written = local_bytes_written;
 if(*bytes_written == num_bytes)
  return MIP_USER_FUNCTION_OK;
 else
  return MIP_USER_FUNCTION_ERROR;
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_sdk_port_read(void *port_handle, u8 *buffer, u32 num_bytes, u32 *bytes_read, u32 timeout_ms)
//
//! @section DESCRIPTION
//! Target-Specific Port Write Function. 
//
//! @section DETAILS
//!
//! @param [in]  void *port_handle  - target-specific port handle pointer (user needs to allocate memory for this)
//! @param [in]  u8 *buffer         - buffer containing num_bytes of data
//! @param [in]  u32 num_bytes      - the number of bytes to write to the port
//! @param [out] u32 *bytes_read    - the number of bytes actually read from the device
//! @param [in]  u32 timeout_ms     - the read timeout
//
//! @retval MIP_USER_FUNCTION_ERROR  When there is a problem communicating with the port.\n
//! @retval MIP_USER_FUNCTION_OK     The read was successful.\n
//
//! @section NOTES
//! 
//! The user should copy the \c mip_sdk_user_functions.c file to their project directory and\n
//! edit it as needed to support their target operating system.
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_sdk_port_read(void *port_handle, u8 *buffer, u32 num_bytes, u32 *bytes_read, u32 timeout_ms)
{
 int local_port_handle = *((int *)port_handle);

 int local_bytes_read = read(local_port_handle, buffer, num_bytes);

 *bytes_read = local_bytes_read;
 if(*bytes_read == num_bytes)
  return MIP_USER_FUNCTION_OK;
 else
  return MIP_USER_FUNCTION_ERROR;
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u32 mip_sdk_port_read_count(void *port_handle)
//
//! @section DESCRIPTION
//! Target-Specific Function to Get the Number of Bytes Waiting on the Port.
//
//! @section DETAILS
//!
//! @param [in]  void *port_handle  - target-specific port handle pointer (user needs to allocate memory for this)
//
//! @returns  Number of bytes waiting on the port,\n
//!           0, if there is an error.
//
//! @section NOTES
//! 
//! The user should copy the \c mip_sdk_user_functions.c file to their project directory and\n
//! edit it as needed to support their target operating system.
//!
//
/////////////////////////////////////////////////////////////////////////////

u32 mip_sdk_port_read_count(void *port_handle)
{
 int local_port_handle = *((int *)port_handle);

 int bytes_available;
 ioctl(local_port_handle, FIONREAD, &bytes_available);
 return (u32)bytes_available;
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u32 mip_sdk_get_time_ms()
//
//! @section DESCRIPTION
//! Target-Specific Call to get the current time in milliseconds.
//
//! @section DETAILS
//!
//! @param [in]  void *port_handle  - target-specific port handle pointer (user needs to allocate memory for this)
//
//! @returns  Current time in milliseconds.
//
//! @section NOTES
//! 
//!   1) This value should no roll-over in short periods of time (e.g. minutes)\n
//!   2) Most systems have a millisecond counter that rolls-over every 32 bits\n
//!      (e.g. 49.71 days roll-over period, with 1 millisecond LSB)\n
//!   3) An absolute reference is not required since this function is\n
//!      used for relative time-outs.\n
//!   4) The user should copy the \c mip_sdk_user_functions.c file to their project directory and\n
//!      edit it as needed to support their target operating system.
//!
//
/////////////////////////////////////////////////////////////////////////////

u32 mip_sdk_get_time_ms()
{

 struct timespec ts;                                                            
                                                                                
 if(clock_gettime(CLOCK_MONOTONIC,&ts) != 0) {                                  
  return -1;                                                                    
 }                                                                              
                                                                                
 return (u32)(ts.tv_sec* 1000ll + ((ts.tv_nsec / 1000000)));
}
