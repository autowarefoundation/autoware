/////////////////////////////////////////////////////////////////////////////
//
//! @file    ring_buffer.h 
//! @author  Nathan Miller
//! @version 1.1
//
//! @description Implements a Ring Buffer in C. 
//
// External dependencies:
//
//  
// 
//!@copyright 2014 Lord Microstrain Sensing Systems. 
//
//!@section CHANGES
//! 
//
//!@section LICENSE
//!
//! THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING 
//! CUSTOMERS WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER 
//! FOR THEM TO SAVE TIME. AS A RESULT, LORD MICROSTRAIN SENSING SYSTEMS
//! SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES 
//! WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT OF SUCH SOFTWARE AND/OR 
//! THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION CONTAINED HEREIN IN CONNECTION 
//! WITH THEIR PRODUCTS.
//
/////////////////////////////////////////////////////////////////////////////

#ifndef _RING_BUFFER_H
#define _RING_BUFFER_H
   
////////////////////////////////////////////////////////////////////////////////
//
//Include Files
//
////////////////////////////////////////////////////////////////////////////////

#include "mip_types.h"
#include <string.h>
#include <stdlib.h>


#ifdef __cplusplus
 extern "C" {
#endif

////////////////////////////////////////////////////////////////////////////////
//
// Defines
//
////////////////////////////////////////////////////////////////////////////////
//! @def 

#define RING_BUFFER_UNINITIALIZED 0
#define RING_BUFFER_INITIALIZED   1

#define RING_BUFFER_OK            0
#define RING_BUFFER_ERROR         1
#define RING_BUFFER_MEMORY_ERROR  2
#define RING_BUFFER_SIZE_MISMATCH 3
#define RING_BUFFER_EMPTY         4
#define RING_BUFFER_FULL          5

#define RING_BUFFER_STATIC_TYPE 0
#define RING_BUFFER_MALLOC_TYPE 1

////////////////////////////////////////////////////////////////////////////////
//
// Structures
//
////////////////////////////////////////////////////////////////////////////////

typedef struct _ring_buffer
{
 //State
 u8 state; 
 u8 type;
  
 //Ring buffer variables
 u8 *entries;
 u32 max_entries;
 u32 entry_size;
 
 u32 volatile position;
 u32 volatile current_count;
 
 //Stats
 u32 volatile total_entries_written;
 u32 volatile total_entries_read;
 u32 volatile total_overruns; 
}ring_buffer;


////////////////////////////////////////////////////////////////////////////////
//
// Function Prototypes
//
////////////////////////////////////////////////////////////////////////////////


//External

u16 ring_buffer_init_static(ring_buffer *buffer, u8 *data_buffer_ptr, u32 max_entries, u32 entry_size);
u16 ring_buffer_init_malloc(ring_buffer *buffer, u32 max_entries, u32 entry_size);

u32 ring_buffer_count(ring_buffer *buffer);
u32 ring_buffer_remaining_entries(ring_buffer *buffer);
u16 ring_buffer_flush(ring_buffer *buffer);

u16 ring_buffer_write(ring_buffer *buffer, u8 *entry, u32 num_bytes);
u16 ring_buffer_write_multi(ring_buffer *buffer, u8 *entry_buff, u32 num_entries, u32 *num_written);

u16 ring_buffer_read(ring_buffer *buffer, u8 *entry, u32 max_bytes);
u16 ring_buffer_read_multi(ring_buffer *buffer, u8 *entry_buff, u32 entry_buff_size, u32 num_requested, u32 *num_read);

u16 ring_buffer_lookahead_read(ring_buffer *buffer, u32 offset, u8 *entry, u32 max_bytes);
u16 ring_buffer_lookahead_read_multi(ring_buffer *buffer, u32 offset, u8 *entry_buff, u32 entry_buff_size, u32 num_requested, u32 *num_read);

u32 ring_buffer_copy(ring_buffer *to, ring_buffer *from, u32 num_entries);
u32 ring_buffer_lookahead_copy(ring_buffer *to, u32 offset, ring_buffer *from, u32 num_entries);
u16 ring_buffer_consume_entries(ring_buffer *buffer, u32 num_entries);

u8 *ring_buffer_get_first_element_ptr(ring_buffer *buffer); 
u16 ring_buffer_remove_first_element(ring_buffer *buffer); 

u8 *ring_buffer_get_available_element_ptr(ring_buffer *buffer); 
u16 ring_buffer_increment_count(ring_buffer *buffer); 

u16 ring_buffer_malloc_free(ring_buffer *buffer);

//Internal

void __ring_buffer_reset(ring_buffer *buffer);


#ifdef __cplusplus
}
#endif

#endif