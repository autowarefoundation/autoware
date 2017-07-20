/**
 * @file    my_user_functions.h
 * @author  Brian S. Bingham
 * @version 0.1
 *
 * @brief Target(Linux)-Specific Functions Required by the MIP SDK
 *
 * Implementation of mip_sdk_user_functions.c prototypes for Linux
 *
 */

#ifndef _MIP_SDK_USER_FUNCTIONS_H
#define _MIP_SDK_USER_FUNCTIONS_H

//Include Files
#include "mip.h"
#include <time.h>
#include <sys/ioctl.h>
#include <termios.h> // terminal io (serial port) interface
#include <fcntl.h>  // File control definitions
#include <errno.h>  // Error number definitions
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

// Defines
//! @def 

typedef int ComPortHandle;
typedef unsigned char Byte;

#define MIP_USER_FUNCTION_OK    0
#define MIP_USER_FUNCTION_ERROR 1

#define MIP_COM_PORT_BUFFER_SIZE 0x200


// Function Prototypes
u16 purge(ComPortHandle comPortHandle);

u16 mip_sdk_port_open(void **port_handle, const char *portstr, int baudrate);

u16 mip_sdk_port_close(void *port_handle);

u16 mip_sdk_port_write(void *port_handle, u8 *buffer, u32 num_bytes, u32 *bytes_written, u32 timeout_ms);

u16 mip_sdk_port_read(void *port_handle, u8 *buffer, u32 num_bytes, u32 *bytes_read, u32 timeout_ms);

u32 mip_sdk_port_read_count(void *port_handle);

u32 mip_sdk_get_time_ms();

#endif
