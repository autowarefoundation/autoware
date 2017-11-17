/////////////////////////////////////////////////////////////////////////////
//
//! @file    ring_buffer.c 
//! @author  Nathan Miller
//! @version 1.1
//
//! @description Implements a Ring Buffer in C.
//
// External dependencies:
//
//  mip.h
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


////////////////////////////////////////////////////////////////////////////////
//
//Include Files
//
////////////////////////////////////////////////////////////////////////////////

#include "ring_buffer.h"



/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 ring_buffer_init_static(ring_buffer *buffer, u8 *data_buffer_ptr, 
//!                             u32 max_entries, u8 entry_size)
//
//! @section DESCRIPTION
//! Initialize a ring buffer with a static memory location. 
//
//! @section DETAILS
//!
//! @param [out] ring_buffer *buffer - pointer to a ring buffer structure.
//! @param [in]  u8 *data_buffer_ptr - data pointer (must be allocated and size = \c max_entries * \c entry_size).
//! @param [in]  u32 max_entries     - maximum entries the ring buffer may contain.
//! @param [in]  u32 entry_size      - size of each entry (bytes).
//
//! @retval RING_BUFFER_ERROR Buffer not initialized.\n
//! @retval RING_BUFFER_OK    Buffer initialized.\n
//
//! @section NOTES
//! 
//! None
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 ring_buffer_init_static(ring_buffer *buffer, u8 *data_buffer_ptr, u32 max_entries, u32 entry_size)
{
  
 //Null check the ring buffer pointer
 if(buffer == 0)
   return RING_BUFFER_ERROR;

 //Set the buffer to the uninitialized state
 buffer->state = RING_BUFFER_UNINITIALIZED;
 
 //Null check the data buffer pointer
 if(data_buffer_ptr == 0)
   return RING_BUFFER_ERROR;
 
 //Reset buffer variables and stats
 __ring_buffer_reset(buffer);
 
 buffer->entries     = data_buffer_ptr;
 buffer->max_entries = max_entries;
 buffer->entry_size  = entry_size;
 buffer->type        = RING_BUFFER_STATIC_TYPE;
 
 //Set the buffer to the initialized state
 buffer->state = RING_BUFFER_INITIALIZED;
 
 return RING_BUFFER_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 ring_buffer_init_malloc(ring_buffer *buffer, u32 max_entries, u8 entry_size)
//
//! @section DESCRIPTION
//! Initialize a ring buffer using malloc to allocate the buffer from the heap. 
//
//! @section DETAILS
//!
//! @param [out] ring_buffer *buffer - pointer to a ring buffer structure.
//! @param [in]  u32 max_entries     - maximum entries the ring buffer may contain.
//! @param [in]  u32 entry_size      - size of each entry (bytes).
//
//! @retval RING_BUFFER_ERROR Buffer not initialized.\n
//! @retval RING_BUFFER_OK    Buffer initialized.\n
//
//! @section NOTES
//! 
//! None
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 ring_buffer_init_malloc(ring_buffer *buffer, u32 max_entries, u32 entry_size)
{
 
 //Null check the ring buffer pointer
 if(buffer == 0)
   return RING_BUFFER_ERROR;
 
 //Set the buffer to the uninitialized state
 buffer->state = RING_BUFFER_UNINITIALIZED;
 
 //Attempt to allocate the needed space
 buffer->entries = (u8 *)malloc(max_entries*entry_size);
 
 //If the pointer is null, malloc failed
 if(buffer->entries == 0)
  return RING_BUFFER_MEMORY_ERROR;
 
 //Reset buffer variables and stats
 __ring_buffer_reset(buffer);
 
 buffer->max_entries = max_entries;
 buffer->entry_size  = entry_size;
 buffer->type        = RING_BUFFER_MALLOC_TYPE;
 
 //Set the buffer to the initialized state
 buffer->state = RING_BUFFER_INITIALIZED;
 
 return RING_BUFFER_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u32 ring_buffer_count(ring_buffer *buffer)
//
//! @section DESCRIPTION
//! Returns the number of entries in the ring buffer. 
//
//! @section DETAILS
//!
//! @param [in] ring_buffer *buffer - pointer to a ring buffer structure.
//
//! @returns Number of entries in the buffer, if initialized.\n
//!          0 Otherwise.
//
//! @section NOTES
//! 
//! None
//!
//
/////////////////////////////////////////////////////////////////////////////

u32 ring_buffer_count(ring_buffer *buffer)
{
 //Null check the ring buffer pointer
 if(buffer == 0)
   return 0;
  
 //Check that the buffer was initialized
 if(buffer->state != RING_BUFFER_INITIALIZED)
   return 0;
 
 return buffer->current_count;
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u32 ring_buffer_remaining_entries(ring_buffer *buffer)
//
//! @section DESCRIPTION
//! Returns the number of empty entries in the ring buffer. 
//
//! @section DETAILS
//!
//! @param [in] ring_buffer *buffer - pointer to a ring buffer structure.
//
//! @returns Number of empty entries in the buffer, if initialized.\n
//!          0 Otherwise.
//
//! @section NOTES
//! 
//! None
//!
//
/////////////////////////////////////////////////////////////////////////////

u32 ring_buffer_remaining_entries(ring_buffer *buffer)
{
 //Null check the ring buffer pointer
 if(buffer == 0)
   return 0;
  
 //Check that the buffer was initialized
 if(buffer->state != RING_BUFFER_INITIALIZED)
   return 0;
 
 return buffer->max_entries - buffer->current_count; 
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 ring_buffer_flush(ring_buffer *buffer)
//
//! @section DESCRIPTION
//! Flushes the ring buffer. 
//
//! @section DETAILS
//!
//! @param [in] ring_buffer *buffer - pointer to a ring buffer structure.
//
//! @retval RING_BUFFER_ERROR Buffer not initialized.\n
//! @retval RING_BUFFER_OK    Buffer flushed.\n
//
//! @section NOTES
//! 
//! None
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 ring_buffer_flush(ring_buffer *buffer)
{
 //Null check the ring buffer pointer
 if(buffer == 0)
   return RING_BUFFER_ERROR;
  
 //Check that the buffer was initialized
 if(buffer->state != RING_BUFFER_INITIALIZED)
   return RING_BUFFER_ERROR;

 //Flush the buffer
 buffer->position      = 0;
 buffer->current_count = 0;

 return RING_BUFFER_OK;
}

/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 ring_buffer_write(ring_buffer *buffer, u8 *entry, u32 num_bytes)
//
//! @section DESCRIPTION
//! Write \c num_bytes of a single entry to the ring buffer. 
//
//! @section DETAILS
//!
//! @param [out] ring_buffer *buffer - pointer to a ring buffer structure.
//! @param [in] u8 *entry            - pointer to memory buffer holding the entry.
//! @param [in] u32 num_bytes        - number of bytes to write.
//
//! @retval RING_BUFFER_ERROR        Entry not written, buffer not initialized.\n
//! @retval RING_BUFFER_FULL         Entry not written, the buffer is full.\n
//! @retval RING_BUFFER_MEMORY_ERROR Entry not written, num_bytes is larger than the ring buffer entry size.\n
//! @retval RING_BUFFER_OK           Entry written.\n
//
//! @section NOTES
//! 
//! \c num_bytes included for ring buffers with variable entry sizes (e.g. strings).
//! \c num_bytes must be less than the max entry size.
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 ring_buffer_write(ring_buffer *buffer, u8 *entry, u32 num_bytes)
{
 u32 entry_pos = 0, i;
  
 //Null check the pointer
 if(buffer == 0)
   return RING_BUFFER_ERROR;

 //Check for a full ring buffer
 if(buffer->current_count >= buffer->max_entries)
 {
  buffer->total_overruns++;
  return RING_BUFFER_FULL; 
 }
 
 //Zero byte short circuit
 if(num_bytes == 0)
  return RING_BUFFER_OK;


 //Check entry size
 if(num_bytes > buffer->entry_size)
  return RING_BUFFER_MEMORY_ERROR;  
 
 //Calculate the write position
 entry_pos = buffer->position + buffer->current_count;
 
 if(entry_pos >= buffer->max_entries)
  entry_pos -= buffer->max_entries;
 
 //Write the entry
 for(i=0;i<num_bytes;i++)
   buffer->entries[entry_pos*buffer->entry_size + i] = entry[i];
 
 //Update the ring buffer
 buffer->current_count++;
 buffer->total_entries_written++;
 
 return RING_BUFFER_OK;  
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 ring_buffer_write_multi(ring_buffer *buffer, u8 *entry_buff, u32 num_entries)
//
//! @section DESCRIPTION
//! Write \c num_entries to the ring buffer. 
//
//! @section DETAILS
//!
//! @param [out] ring_buffer *buffer - pointer to a ring buffer structure.
//! @param [in] u8 *entry_buff       - pointer to memory buffer holding the entries.
//! @param [in] u32 num_entries      - number of entries to write.
//
//! @retval RING_BUFFER_ERROR        Entries not written, buffer not initialized.\n
//! @retval RING_BUFFER_FULL         Entries not written, the buffer is full.\n
//! @retval RING_BUFFER_MEMORY_ERROR Entries not written, num_bytes is larger than the ring buffer entry size.\n
//! @retval RING_BUFFER_OK           Entries written.\n
//
//! @section NOTES
//! 
//! All entries are assumed to be of size \c entry_size, which is set by the ring_buffer_init_xxxxxx call.
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 ring_buffer_write_multi(ring_buffer *buffer, u8 *entry_buff, u32 num_entries, u32 *num_written)
{
 u32 entry_pos = 0, i, j;
  
 *num_written = 0;
 
 //Null check the pointer
 if(buffer == 0)
   return RING_BUFFER_ERROR;

 //Zero entry short circuit
 if(num_entries == 0)
  return RING_BUFFER_OK;

 //Loop through the requested entries
 for(i=0; i<num_entries; i++)
 {

  //Check for a ring buffer space
  if(buffer->current_count >= buffer->max_entries)
  {
   buffer->total_overruns += num_entries - i;
   return RING_BUFFER_FULL; 
  }
   
  //Calculate the write position
  entry_pos = buffer->position + buffer->current_count;
 
  if(entry_pos >= buffer->max_entries)
   entry_pos -= buffer->max_entries;
 
   //Write the entry
   for(j=0;j<buffer->entry_size;j++)
     buffer->entries[entry_pos*buffer->entry_size + j] = entry_buff[i*buffer->entry_size + j];
 
  //Update the ring buffer
  buffer->current_count++;
  buffer->total_entries_written++;
  (*num_written)++;
 }

 return RING_BUFFER_OK; 
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 ring_buffer_read(ring_buffer *buffer, u8 *entry, u32 max_bytes)
//
//! @section DESCRIPTION
//! Read a single entry from the ring buffer. 
//
//! @section DETAILS
//!
//! @param [in] ring_buffer *buffer - pointer to a ring buffer structure.
//! @param [out] u8 *entry          - pointer to data buffer where the entry will be stored.
//! @param [in] u32 max_bytes       - maximum size of the data buffer.
//
//! @retval RING_BUFFER_ERROR        Entry not read, buffer not initialized.\n
//! @retval RING_BUFFER_FULL         Entry not read, the buffer is empty.\n
//! @retval RING_BUFFER_MEMORY_ERROR Entry not read, max_bytes is smaller than the ring buffer entry size.\n
//! @retval RING_BUFFER_OK           Entry read.\n
//
//! @section NOTES
//! 
//! None.
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 ring_buffer_read(ring_buffer *buffer, u8 *entry, u32 max_bytes)
{
 u32 i; 
  
 //Check for an empty buffer
 if(buffer->current_count == 0)
  return RING_BUFFER_EMPTY; 
  
 //Copy the entry
 for(i=0;i<buffer->entry_size;i++)
   entry[i] = buffer->entries[buffer->position*buffer->entry_size + i];
 
 //Update the ring buffer
 if(++buffer->position >= buffer->max_entries)
   buffer->position -= buffer->max_entries;
 
 buffer->current_count--;
 buffer->total_entries_read++;
 
 return RING_BUFFER_OK;  
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 ring_buffer_lookahead_read(ring_buffer *buffer, u32 offset, u8 *entry, u32 max_bytes)
//
//! @section DESCRIPTION
//! Read an entry from the ring buffer without consuming it. 
//
//! @section DETAILS
//!
//! @param [in] ring_buffer *buffer - pointer to a ring buffer structure.
//! @param [in] u32 offset          - offset from the head of the ring buffer to read.
//! @param [out] u8 *entry          - pointer to data buffer where the entry will be stored.
//! @param [in] u32 max_bytes       - maximum size of the data buffer.
//
//! @retval RING_BUFFER_ERROR        Entry not read, buffer not initialized.\n
//! @retval RING_BUFFER_FULL         Entry not read, the buffer is empty.\n
//! @retval RING_BUFFER_MEMORY_ERROR Entry not read, max_bytes is smaller than the ring buffer entry size.\n
//! @retval RING_BUFFER_OK           Entries read.\n
//
//! @section NOTES
//! 
//! None.
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 ring_buffer_lookahead_read(ring_buffer *buffer, u32 offset, u8 *entry, u32 max_bytes)
{
 u32 local_position, i;
  
 //Null check the pointer
 if(buffer == 0)
   return RING_BUFFER_ERROR;

 //Check if the ring buffer is initialized
 if(buffer->state != RING_BUFFER_INITIALIZED)
   return RING_BUFFER_ERROR;
 
 //Check for an empty buffer
 if(buffer->current_count == 0)
  return RING_BUFFER_EMPTY; 

 //Check for buffer size
 if(max_bytes < buffer->entry_size)
  return RING_BUFFER_MEMORY_ERROR;  
  
 //Check that there are the expected number of entries (that count > offset)
 if(buffer->current_count <= offset)
  return RING_BUFFER_ERROR;
 
 //Assign the temporary position
 local_position = buffer->position + offset;
 
 //Temporary position loop logic the ring buffer
 if(local_position >= buffer->max_entries)
   local_position -= buffer->max_entries;
 
 //Copy the entry
 for(i=0;i<buffer->entry_size;i++)
   entry[i] = buffer->entries[local_position*buffer->entry_size + i];
 
 return RING_BUFFER_OK;  
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 ring_buffer_read_multi(ring_buffer *buffer, u8 *entry_buff, u32 entry_buff_size, 
//!                            u32 num_requested, u32 *num_read)
//
//! @section DESCRIPTION
//! Read \c num_requested entries from the ring buffer. 
//
//! @section DETAILS
//!
//! @param [in]  ring_buffer *buffer - pointer to a ring buffer structure.
//! @param [out] u8 *entry           - pointer to data buffer where the entries will be stored.
//! @param [in]  u32 entry_buff_size - maximum size of the data buffer in bytes.
//! @param [in]  u32 num_requested   - the number of entries requested.
//! @param [out] u32 num_read        - the number of entries actually read.
//
//! @retval RING_BUFFER_ERROR        Entry not read, buffer not initialized.\n
//! @retval RING_BUFFER_EMPTY        Entry not read, the buffer is empty.\n
//! @retval RING_BUFFER_MEMORY_ERROR Entry not read, max_bytes is smaller than the ring buffer entry size.\n
//! @retval RING_BUFFER_OK           Entries read.\n
//
//! @section NOTES
//! 
//! None.
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 ring_buffer_read_multi(ring_buffer *buffer, u8 *entry_buff, u32 entry_buff_size, u32 num_requested, u32 *num_read)
{
 u32 i, j; 
  
 //Reset number of entries read
 *num_read = 0;
  
 //Check the buffer size
 if(entry_buff_size < num_requested*buffer->entry_size)
  return RING_BUFFER_MEMORY_ERROR;

 //Copy the entry
 for(i=0; i<num_requested; i++)
 {
  //Check for an empty buffer
  if(buffer->current_count == 0)
   return RING_BUFFER_EMPTY; 

  for(j=0;j<buffer->entry_size;j++)
    entry_buff[i*buffer->entry_size + j] = buffer->entries[buffer->position*buffer->entry_size + j];
 
  //Update the ring buffer
  if(++buffer->position >= buffer->max_entries)
    buffer->position -= buffer->max_entries;
 
  buffer->current_count--;
  buffer->total_entries_read++;

  (*num_read) += 1;
 }

 return RING_BUFFER_OK;  
}



/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 ring_buffer_lookahead_read_multi(ring_buffer *buffer, u32 offset, u8 *entry_buff, 
//!                                      u32 entry_buff_size, u32 num_requested, u32 *num_read)
//
//! @section DESCRIPTION
//! Read \c num_requested entries from the ring buffer without consuming them. 
//
//! @section DETAILS
//!
//! @param [in]  ring_buffer *buffer - pointer to a ring buffer structure.
//! @param [in]  u32 offset          - offset from the head of the ring buffer to read.
//! @param [out] u8 *entry_buff      - pointer to data buffer where the entries will be stored.
//! @param [in]  u32 entry_buff_size - maximum size of the data buffer in bytes.
//! @param [in]  u32 num_requested   - the number of entries requested.
//! @param [out] u32 num_read        - the number of entries actually read.
//
//! @retval RING_BUFFER_ERROR        Entry not read, buffer not initialized.\n
//! @retval RING_BUFFER_EMPTY        Entry not read, the buffer is empty.\n
//! @retval RING_BUFFER_MEMORY_ERROR Entry not read, max_bytes is smaller than the ring buffer entry size.\n
//! @retval RING_BUFFER_OK           Entries read.\n
//
//! @section NOTES
//! 
//! None.
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 ring_buffer_lookahead_read_multi(ring_buffer *buffer, u32 offset, u8 *entry_buff, u32 entry_buff_size, u32 num_requested, u32 *num_read)
{
 u32 local_position, i, j;
  
 //Reset number of entries read
 *num_read = 0;

 //Null check the pointer
 if(buffer == 0)
   return RING_BUFFER_ERROR;

 //Check if the ring buffer is initialized
 if(buffer->state != RING_BUFFER_INITIALIZED)
   return RING_BUFFER_ERROR;
 
 //Check for an empty buffer
 if(buffer->current_count == 0)
  return RING_BUFFER_EMPTY; 

 //Check for buffer size
 if(entry_buff_size < num_requested*buffer->entry_size)
  return RING_BUFFER_MEMORY_ERROR;  

 //Check that there are the expected number of entries (that count > offset)
 if(buffer->current_count < offset + num_requested)
  return RING_BUFFER_ERROR;

 for(i=0; i<num_requested; i++)
 {
  //Assign the temporary position
  local_position = buffer->position + offset + i;

  //Temporary position loop logic the ring buffer
  if(local_position >= buffer->max_entries)
   local_position -= buffer->max_entries;

  //Copy the entry
  for(j=0; j<buffer->entry_size; j++)
   entry_buff[i*buffer->entry_size + j] = buffer->entries[local_position*buffer->entry_size + j];

  //Increment number read
  (*num_read) += 1;
 }
 return RING_BUFFER_OK;  
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 ring_buffer_copy(ring_buffer *to, ring_buffer *from, u32 num_entries)
//
//! @section DESCRIPTION
//! Copy \c num_entries between two ring buffers. 
//
//! @section DETAILS
//!
//! @param [out] ring_buffer *to   - pointer to a ring buffer structure to copy to.
//! @param [in]  ring_buffer *from - pointer to a ring buffer structure to copy from.
//! @param [in]  num_entries       - the number of entries to copy.
//
//! @returns Number of entries copied
//
//! @section NOTES
//! 
//! None.
//!
//
/////////////////////////////////////////////////////////////////////////////

u32 ring_buffer_copy(ring_buffer *to, ring_buffer *from, u32 num_entries)
{
 u32 i, j, write_pos, num_copied = 0;

 //Check that the entry size is the same
 if(from->entry_size != to->entry_size)
   return 0;
 
 //Loop through num_entries, copying each 
 for(i=0; i<num_entries; i++)
 {
  //Check if from buffer is empty
  if(from->current_count == 0)
    return num_copied;
  
  //Check if to buffer is full
  if(to->current_count >= to->max_entries)
    return num_copied;
 
  //Calculate the write position
  write_pos = to->position + to->current_count;
 
  if(write_pos >= to->max_entries)
   write_pos -= to->max_entries;
  
  //Copy the entry
  for(j=0; j < to->entry_size; j++)
    to->entries[write_pos*to->entry_size + j] = from->entries[from->position*from->entry_size +j];
  
  //Update the 'to' ring buffer
  to->current_count++;
  to->total_entries_written++;
  
  //Update the 'from' buffer
  if(++from->position >= from->max_entries)
   from->position -= from->max_entries;
 
  from->current_count--;
  from->total_entries_read++;
  
  num_copied++;
 }
 
 return num_copied;
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 ring_buffer_lookahead_copy(ring_buffer *to, u32 offset, ring_buffer *from, u32 num_entries)
//
//! @section DESCRIPTION
//! Copy \c num_entries between two ring buffers without consuming the entries. 
//
//! @section DETAILS
//!
//! @param [out] ring_buffer *to   - pointer to a ring buffer structure to copy to.
//! @param [in]  u32 offset        - offset from the first element to start the copy.
//! @param [in]  ring_buffer *from - pointer to a ring buffer structure to copy from.
//! @param [in]  num_entries       - the number of entries to copy.
//
//! @returns Number of entries copied
//
//! @section NOTES
//! 
//! None.
//!
//
/////////////////////////////////////////////////////////////////////////////

u32 ring_buffer_lookahead_copy(ring_buffer *to, u32 offset, ring_buffer *from, u32 num_entries)
{
 u32 i, j, write_pos, read_pos;
 
 //Null check the 'to' buffer
 if(to == 0)
   return 0;

 //Null check the 'from' buffer
 if(from == 0)
   return 0;

 //Check if the 'to' ring buffer is initialized
 if(to->state != RING_BUFFER_INITIALIZED)
   return 0;
 
 //Check if the 'from' ring buffer is initialized
 if(from->state != RING_BUFFER_INITIALIZED)
   return 0;
 
 //Check that the entry size is the same
 if(from->entry_size != to->entry_size)
   return 0;
 
 //Check the offset and number of requested entries
 if(num_entries + offset > ring_buffer_count(from))
   return 0;
 
 //Loop through num_entries, copying each 
 for(i=0; i<num_entries; i++)
 {
  //Check if from buffer is empty
  if(from->current_count == 0)
    return i;
  
  //Check if to buffer is full
  if(to->current_count >= to->max_entries)
    return i;
 
  //Calculate the write position
  write_pos = to->position + to->current_count;
 
  if(write_pos >= to->max_entries)
   write_pos -= to->max_entries;

  //Calculate the read position
  read_pos = from->position + offset + i;
 
  if(read_pos >= from->max_entries)
   read_pos -= from->max_entries;
  
  //Copy the entry
  for(j=0;j<to->entry_size;j++)
   to->entries[write_pos*to->entry_size +j] = from->entries[read_pos*from->entry_size + j];
  
  
  //Update the 'to' ring buffer
  to->current_count++;
  to->total_entries_written++;
 }
 
 return i;  
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 ring_buffer_consume_entries(ring_buffer *buffer, u32 num_entries)
//
//! @section DESCRIPTION
//! Removes \c num_entries from the buffer if they exist. 
//
//! @section DETAILS
//!
//! @param [in] ring_buffer *buffer - pointer to a ring buffer structure.
//! @param [in] num_entries         - the number of entries to remove.
//
//! @returns Number of entries removed
//
//! @section NOTES
//! 
//! None.
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 ring_buffer_consume_entries(ring_buffer *buffer, u32 num_entries)
{
 u32 num_consume;
 
 //Calculate the total number of entries to consume
 //(cannot consume more than current number of entries)
 
 if(num_entries > buffer->current_count)
   num_consume = buffer->current_count;
 else
   num_consume = num_entries;
 
 
 //Remove the entries
 buffer->position += num_consume;
 
 if(buffer->position >= buffer->max_entries)
   buffer->position -= buffer->max_entries;
 
 buffer->current_count -= num_consume;
 
 //Update the stats
 buffer->total_entries_read += num_consume;
 
 return num_consume;  
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 ring_buffer_malloc_free(ring_buffer *buffer)
//
//! @section DESCRIPTION
//! Return the allocated memory to the heap. 
//
//! @section DETAILS
//!
//! @param [in]  ring_buffer *buffer - pointer to a ring buffer structure.
//
//! @retval RING_BUFFER_ERROR        Buffer not initialized or not malloc'd.\n
//! @retval RING_BUFFER_OK           Buffer freed.\n
//
//! @section NOTES
//! 
//! None.
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 ring_buffer_malloc_free(ring_buffer *buffer)
{
 //Null check the ring buffer pointer
 if(buffer == 0)
   return RING_BUFFER_ERROR;

 //Check that the buffer did use malloc
 if(buffer->type != RING_BUFFER_MALLOC_TYPE)
   return RING_BUFFER_ERROR;
 
 free(buffer->entries);
 
 return RING_BUFFER_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u8 *ring_buffer_get_first_element_ptr(ring_buffer *buffer)
//
//! @section DESCRIPTION
//! Return a pointer to the first element in the ring buffer. 
//
//! @section DETAILS
//!
//! @param [in] ring_buffer *buffer - pointer to a ring buffer structure.
//
//! @returns NULL (0) If the ring buffer pointer is invalid or there are no elements.\n
//!          Otherwise, the pointer to first element.
//
//! @section NOTES
//! 
//! This routine should only be used for single-element access!!!\n
//! Attempting to access multiple elements using this can cause access\n
//! outside of the array boundaries!
//!
//
/////////////////////////////////////////////////////////////////////////////

u8 *ring_buffer_get_first_element_ptr(ring_buffer *buffer)
{
 //Null check the ring buffer pointer
 if(buffer == 0)
   return 0;
 
 //Check if the ring buffer is initialized
 if(buffer->state != RING_BUFFER_INITIALIZED)
   return 0;
  
 //Check for an empty buffer
 if(buffer->current_count == 0)
  return 0; 
 
 return &buffer->entries[buffer->position*buffer->entry_size];
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 ring_buffer_remove_first_element(ring_buffer *buffer)
//
//! @section DESCRIPTION
//! Removes the first element of the ring buffer if it exists. 
//
//! @section DETAILS
//!
//! @param [in] ring_buffer *buffer - pointer to a ring buffer structure.
//
//! @retval RING_BUFFER_ERROR Ring buffer invalid.\n
//! @retval RING_BUFFER_EMPTY Ring buffer empty.\n
//! @retval RING_BUFFER_OK    Element removed.\n
//
//! @section NOTES
//! 
//! Used in conjunction with ring_buffer_get_first_element_ptr\n
//! for pointer read access to ring buffer elements.
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 ring_buffer_remove_first_element(ring_buffer *buffer)
{
 //Null check the ring buffer pointer
 if(buffer == 0)
   return RING_BUFFER_ERROR;
 
 //Check if the ring buffer is initialized
 if(buffer->state != RING_BUFFER_INITIALIZED)
   return RING_BUFFER_ERROR;
  
 //Check for an empty buffer
 if(buffer->current_count == 0)
  return RING_BUFFER_EMPTY; 
 
 //Remove the first element
 if(++buffer->position >= buffer->max_entries)
   buffer->position -= buffer->max_entries;
 
 buffer->current_count--;
  
 //Update the stats
 buffer->total_entries_read++;
 
 return RING_BUFFER_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u8 *ring_buffer_get_available_element_ptr(ring_buffer *buffer)
//
//! @section DESCRIPTION
//! Return a pointer to the next available element in the ring buffer. 
//
//! @section DETAILS
//!
//! @param [in] ring_buffer *buffer - pointer to a ring buffer structure.
//
//! @returns NULL (0) If the ring buffer pointer is invalid or there are no elements.\n
//!          Otherwise, the pointer to first available element.
//
//! @section NOTES
//! 
//! This routine should only be used for single-element access!!!\n
//! Attempting to access multiple elements using this can cause access\n
//! outside of the array boundaries!
//!
//
/////////////////////////////////////////////////////////////////////////////

u8 *ring_buffer_get_available_element_ptr(ring_buffer *buffer)
{
 u32 entry_pos;
 
 //Null check the ring buffer pointer
 if(buffer == 0)
   return 0;
 
 //Check if the ring buffer is initialized
 if(buffer->state != RING_BUFFER_INITIALIZED)
   return 0;
  
 //Check for an full buffer
 if(buffer->current_count >= buffer->max_entries)
  return 0; 
 
 //Calculate the write position
 entry_pos = buffer->position + buffer->current_count;
 
 if(entry_pos >= buffer->max_entries)
  entry_pos -= buffer->max_entries;
 
 return &buffer->entries[entry_pos*buffer->entry_size];
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 ring_buffer_increment_count(ring_buffer *buffer)
//
//! @section DESCRIPTION
//! Increments the number of elements in the ring buffer. 
//
//! @section DETAILS
//!
//! @param [in] ring_buffer *buffer - pointer to a ring buffer structure.
//
//! @retval RING_BUFFER_ERROR Ring buffer invalid.\n
//! @retval RING_BUFFER_EMPTY Ring buffer full.\n
//! @retval RING_BUFFER_OK    Element added.\n
//
//! @section NOTES
//! 
//! Used in conjunction with ring_buffer_get_available_element_ptr\n
//! for pointer write access to ring buffer elements.
//!
//
/////////////////////////////////////////////////////////////////////////////


u16 ring_buffer_increment_count(ring_buffer *buffer)
{
 //Null check the ring buffer pointer
 if(buffer == 0)
   return RING_BUFFER_ERROR;
 
 //Check if the ring buffer is initialized
 if(buffer->state != RING_BUFFER_INITIALIZED)
   return RING_BUFFER_ERROR;
  
 //Check for a full buffer
 if(buffer->current_count >= buffer->max_entries)
  return RING_BUFFER_FULL; 
 
 //Add the next element
 buffer->current_count++;
 
 //Update the stats
 buffer->total_entries_written++;
 
 return RING_BUFFER_OK;   
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void __ring_buffer_reset(ring_buffer *buffer)
//
//! @section DESCRIPTION
//! Reset the ring buffer state, stats, and buffer. 
//
//! @section DETAILS
//!
//! @param [in] ring_buffer *buffer - pointer to a ring buffer structure.
//
//! @section NOTES
//! 
//! Internal Function.
//!
//
/////////////////////////////////////////////////////////////////////////////

void __ring_buffer_reset(ring_buffer *buffer)
{
 //State
 buffer->state = RING_BUFFER_UNINITIALIZED;  
  
 //Ring buffer variables
 buffer->position      = 0;
 buffer->current_count = 0;
 
 //Stats
 buffer->total_entries_written = 0;
 buffer->total_entries_read    = 0;
 buffer->total_overruns        = 0;   
}

