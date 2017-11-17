/////////////////////////////////////////////////////////////////////////////
//
//! @file    mip.c 
//! @author  Nathan Miller
//! @version 1.1
//
//! @description MIP Definition, Packet Parsing, and Construction Functions
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

#include "mip.h"


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_init(u8 *mip_buffer, u16 buffer_size, u8 descriptor_set)
//
//! @section DESCRIPTION
//! Initialize a MIP packet header.  
//
//! @section DETAILS
//!
//! @param [out] u8 *mip_buffer    - pointer to memory used to construct the mip packet.
//! @param [in] u16 buffer_size    - size of the mip memory buffer.
//! @param [in] u8 descriptor_set  - descriptor set field value.
//
//! @retval MIP_OK           - MIP packet initialized.\n
//! @retval MIP_ERROR        - The pointer is NULL.\n
//! @retval MIP_MEMORY_ERROR - Not enough room in the mip buffer.\n
//
//! @section NOTES
//! 
//! None
//!
//
/////////////////////////////////////////////////////////////////////////////


u16 mip_init(u8 *mip_buffer, u16 buffer_size, u8 descriptor_set)
{
 mip_header *header_ptr = (mip_header*)mip_buffer; 
  
 //Null check mip buffer
 if(mip_buffer == 0)
   return MIP_ERROR;
 
 //The buffer must contain at least enough bytes for the header and checksum
 if(buffer_size < MIP_HEADER_SIZE + MIP_CHECKSUM_SIZE)
  return MIP_MEMORY_ERROR;
 
 //Initialize the header
 header_ptr->sync1          = MIP_SYNC_BYTE1;
 header_ptr->sync2          = MIP_SYNC_BYTE2;
 header_ptr->descriptor_set = descriptor_set;
 header_ptr->payload_size   = 0;
 
 return MIP_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_is_initialized(u8 *mip_buffer, u8 descriptor_set)
//
//! @section DESCRIPTION
//! Check if a MIP packet is initialized.  
//
//! @section DETAILS
//!
//! @param [in] u8 *mip_buffer    - pointer to memory used to construct the mip packet.
//! @param [in] u8 descriptor_set - expected descriptor set field value.
//
//! @retval MIP_OK           - MIP packet initialized.\n
//! @retval MIP_ERROR        - The pointer is NULL or the MIP packet is not intialized.\n
//
//! @section NOTES
//! 
//! None
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_is_initialized(u8 *mip_buffer,  u8 descriptor_set)
{
 mip_header *header_ptr = (mip_header*)mip_buffer; 
 
 //Null check mip buffer
 if(mip_buffer == 0)
   return MIP_ERROR;
 
 //MIP packet is initialized if sync bytes are correct and the class matches.
 if((header_ptr->sync1          == MIP_SYNC_BYTE1) && 
    (header_ptr->sync2          == MIP_SYNC_BYTE2) && 
    (header_ptr->descriptor_set == descriptor_set))
   return MIP_OK;
 
 return MIP_ERROR;
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_add_field(u8 *mip_buffer, u16 buffer_size, void *field_data, u32 data_size, u16 data_descriptor)
//
//! @section DESCRIPTION
//! Adds a field to an initialized MIP packet buffer.  
//
//! @section DETAILS
//!
//! @param [in,out] u8 *mip_buffer     - pointer to memory used to construct the mip packet.
//! @param [in] u16 buffer_size        - size of the mip mempory buffer.
//! @param [in] void *field_data       - buffer containing the field data.
//! @param [in] u8 u16 data_size       - size of the field data (exlcuding the field header).
//! @param [in] u8 u16 data_descriptor - the data descriptor for the field.
//
//! @retval MIP_OK           - field added to MIP packet.\n
//! @retval MIP_ERROR        - A pointer is NULL.\n
//! @retval MIP_MEMORY_ERROR - Not enough room in the mip buffer to add the field or\n
//!                            the packet will be too large for a MIP.
//
//! @section NOTES
//! 
//! None
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_add_field(u8 *mip_buffer, u16 buffer_size, void *field_data, u16 data_size, u16 data_descriptor)
{
 mip_header *header_ptr = (mip_header*)mip_buffer; 
 mip_field_header *field_header_ptr; 
 u8               *field_data_ptr;
 u16              new_packet_size; 
 
 //Null check info
 if(mip_buffer == 0)
   return MIP_ERROR;
 
 //Null check field_data
 if((field_data == 0) && (data_size > 0))
   return MIP_ERROR;
   
 new_packet_size = sizeof(mip_header) + header_ptr->payload_size + data_size + sizeof(mip_field_header) + MIP_CHECKSUM_SIZE;
 
 //Check if there is enough room left to store the field
 if((new_packet_size > MIP_MAX_PACKET_SIZE) || (new_packet_size > buffer_size))
  return MIP_MEMORY_ERROR;
 
 //Jump to the end of the packet
 field_header_ptr = (mip_field_header*)(mip_buffer + sizeof(mip_header) + header_ptr->payload_size);
 
 //Fill-in the field header
 field_header_ptr->size       = data_size + sizeof(mip_field_header);
 field_header_ptr->descriptor = (u8)data_descriptor;
 
 //Add the field data
 if(field_data != 0)
 {
  field_data_ptr = ((u8*)field_header_ptr + sizeof(mip_field_header));
  memcpy(field_data_ptr, field_data, data_size);
 }
 
 //Update the header
 header_ptr->payload_size += data_size + sizeof(mip_field_header);
 
 return MIP_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_add_formatted_field(u8 *mip_buffer, u16 buffer_size, void *field)
//
//! @section DESCRIPTION
//! Adds a pre-formatted field (header and data already assembled) to an initialized MIP packet buffer.  
//
//! @section DETAILS
//!
//! @param [in,out] u8 *mip_buffer  - pointer to memory used to construct the mip packet.
//! @param [in] u16 buffer_size     - size of the mip mempory buffer.
//! @param [in] void *field         - buffer containing the field (must have header and data already assembled).
//
//! @retval MIP_OK           - field added to MIP packet.\n
//! @retval MIP_ERROR        - A pointer is NULL.\n
//! @retval MIP_MEMORY_ERROR - Not enough room in the mip buffer to add the field or\n
//!                            the packet will be too large for a MIP.
//
//! @section NOTES
//! 
//! None
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_add_formatted_field(u8 *mip_buffer, u16 buffer_size, void *field)
{
 mip_header       *header_ptr       = (mip_header*)mip_buffer; 
 mip_field_header *field_header_ptr = (mip_field_header *)field; 
 u8               *field_location;
 u16              new_packet_size; 
 
 //Null check info
 if(mip_buffer == 0)
   return MIP_ERROR;
 
 //Null check field
 if(field == 0)
   return MIP_ERROR;
   
 new_packet_size = sizeof(mip_header) + header_ptr->payload_size + field_header_ptr->size + MIP_CHECKSUM_SIZE;
 
 //Check if there is enough room left to store the field
 if((new_packet_size > MIP_MAX_PACKET_SIZE) || (new_packet_size > buffer_size))
  return MIP_MEMORY_ERROR;
 
 //Jump to the end of the packet
 field_location = mip_buffer + sizeof(mip_header) + header_ptr->payload_size;
 
 //Add the field
 memcpy(field_location, field, field_header_ptr->size);
 
 //Update the header
 header_ptr->payload_size += field_header_ptr->size;
 
 return MIP_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_finalize(u8 *mip_buffer)
//
//! @section DESCRIPTION
//! Calculates the MIP checksum and updates the checksum field in the buffer.  
//
//! @section DETAILS
//!
//! @param [in,out] u8 *mip_buffer  - pointer to memory used to construct the mip packet.
//
//! @returns Size of the MIP Packet in bytes.
//
//! @section NOTES
//! 
//! None
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_finalize(u8 *mip_buffer)
{
 u16 checksum, checksum_offset;
 mip_header *header_ptr = (mip_header*)mip_buffer;   

 //Null check info
 if(mip_buffer == 0)
   return 0;
 
 //Calculate the checksum offset
 checksum_offset = header_ptr->payload_size + MIP_HEADER_SIZE;
 
 //Check that the offset is valid
 if(checksum_offset > MIP_MAX_PACKET_SIZE - MIP_CHECKSUM_SIZE) 
  return 0;

 checksum = mip_calculate_checksum(mip_buffer);

 //Put the checksum in the packet 
 mip_buffer[checksum_offset]     = (checksum >> 8) & 0xFF;
 mip_buffer[checksum_offset + 1] = checksum & 0xFF;
 
 return header_ptr->payload_size + MIP_HEADER_SIZE + MIP_CHECKSUM_SIZE;
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_is_mip_packet(u8 *mip_buffer)
//
//! @section DESCRIPTION
//! Checks if the buffer contains a valid MIP header.  
//
//! @section DETAILS
//!
//! @param [in,out] u8 *mip_buffer  - pointer to memory containing the mip packet.
//
//! @retval MIP_OK           - Header is valid.\n
//! @retval MIP_ERROR        - The pointer is NULL.\n
//! @retval MIP_MEMORY_ERROR - The buffer does not contain a valid MIP packet header.
//
//! @section NOTES
//! 
//! None
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_is_mip_packet(u8 *mip_buffer)
{
 mip_header *header_ptr = (mip_header*)mip_buffer; 

 //Null check info
 if(mip_buffer == 0)
   return MIP_ERROR;

 //If the header contains valid parameters, return OK
 if((header_ptr->sync1 == MIP_SYNC_BYTE1) && (header_ptr->sync2 == MIP_SYNC_BYTE2) && 
    (header_ptr->payload_size <= MIP_MAX_PAYLOAD_SIZE))
   return MIP_OK;
 
 //Not a MIP packet header
 return MIP_INVALID_PACKET;
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_get_packet_size(u8 *mip_buffer)
//
//! @section DESCRIPTION
//! Gets the size of the MIP packet.  
//
//! @section DETAILS
//!
//! @param [in] u8 *mip_buffer  - pointer to memory containing the mip packet.
//
//! @returns  0 - An error occurred\n
//!           Size of the MIP packet, otherwise
//
//! @section NOTES
//! 
//! None
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_get_packet_size(u8 *mip_buffer)
{
 mip_header *header_ptr = (mip_header*)mip_buffer; 
 
 //Check the validity of the packet
 if(mip_is_mip_packet(mip_buffer) != MIP_OK)
   return 0;
 
 //Return the packet size
 return header_ptr->payload_size + MIP_HEADER_SIZE + MIP_CHECKSUM_SIZE;
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_get_first_field(u8 *mip_buffer, mip_field_header **field_header,
//!                         u8 **field_data, u16 *field_offset)
//
//! @section DESCRIPTION
//! Gets the first MIP field (a wrapper for mip_get_next_field that makes it easier to use).  
//
//! @section DETAILS
//!
//! @param [in]  u8 *mip_buffer  - pointer to memory containing the mip packet.
//! @param [out] mip_field_header **field_header - pointer to the first field header.
//! @param [out] u8               **field_data   - pointer to the first field data.
//! @param [out] u16              *field_offset  - integer offset from start of packet payload of first field.
//
//! @returns See mip_get_next_field definition.
//
//! @section NOTES
//! 
//! None
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_get_first_field(u8 *mip_buffer, mip_field_header **field_header, u8 **field_data, u16 *field_offset)
{
 //Null check field_offset
 if(field_offset == 0)
  return MIP_ERROR;
  
 //Start at the first field
 *field_offset = 0;
  
 //The "next field" function is used to calculate the offset
 return mip_get_next_field(mip_buffer, field_header, field_data, field_offset);
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_get_next_field(u8 *mip_buffer, mip_field_header **field_header, 
//!                        u8 **field_data, u16 *field_offset)
//
//! @section DESCRIPTION
//! Gets the next data field at \c field_offset.  
//
//! @section DETAILS
//!
//! @param [in]     u8 *mip_buffer                  - pointer to memory containing the mip packet.
//! @param [out]    mip_field_header **field_header - pointer to the current field header.
//! @param [out]    u8  **field_data                - pointer to the current field data.
//! @param [in,out] u16 *field_offset               - integer offset from start of packet payload of next field.
//
//! @retval MIP_OK                  - c\ field_header and \c field_data contain the parsed information.\n
//! @retval MIP_ERROR               - A pointer is NULL.\n
//! @retval MIP_INVALID_PACKET      - The buffer does not contain a valid MIP packet.
//! @retval MIP_FIELD_NOT_AVAILABLE - The requested field does not exist.
//
//! @section NOTES
//! 
//! None
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_get_next_field(u8 *mip_buffer, mip_field_header **field_header, u8 **field_data, u16 *field_offset)
{
 mip_header *header_ptr = (mip_header *)mip_buffer; 
 u8 *field_ptr;
 
 //Null check info
 if(mip_buffer == 0)
   return MIP_ERROR;

 //Check that the header is valid
 if(mip_is_mip_packet(mip_buffer) != MIP_OK)
   return MIP_INVALID_PACKET;
 
 //Check that the field exists
 if(*field_offset >= header_ptr->payload_size) 
   return MIP_FIELD_NOT_AVAILABLE;
 
 //Assign the pointers
 field_ptr     =  mip_get_payload_ptr(mip_buffer) + *field_offset;
 *field_header = (mip_field_header*)field_ptr;
 *field_data   =  field_ptr + sizeof(mip_field_header);
 
 //0 size fields are unallowed!
 if((*field_header)->size == 0)
  return MIP_ERROR;
 
 //Update field_offset
 *field_offset += (*field_header)->size;
 
 return MIP_OK;
}


////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u8 mip_get_packet_descriptor_set(u8 *mip_buffer)
//
//! @section DESCRIPTION
//! Returns the MIP packet descriptor set.  
//
//! @section DETAILS
//!
//! @param [in] u8 *mip_buffer  - pointer to memory containing the mip packet.
//
//! @returns  The MIP descriptor set byte if the buffer points to a valid packet.\n
//!           0x00 otherwise.
//
//! @section NOTES
//! 
//! None
//!
//
/////////////////////////////////////////////////////////////////////////////

u8 mip_get_packet_descriptor_set(u8 *mip_buffer)
{
 mip_header *header_ptr = (mip_header *)mip_buffer; 
  
 //Null check info
 if(mip_buffer == 0)
   return 0;

 //Check that the header is valid
 if(mip_is_mip_packet(mip_buffer) != MIP_OK)
   return 0;
 
 return header_ptr->descriptor_set;
}


////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u8 mip_get_payload_size(u8 *mip_buffer)
//
//! @section DESCRIPTION
//! Returns the MIP payload size.  
//
//! @section DETAILS
//!
//! @param [in] u8 *mip_buffer  - pointer to memory containing the mip packet.
//
//! @returns  The MIP payload size if the buffer points to a valid packet.\n
//!           0x00 otherwise.
//
//! @section NOTES
//! 
//! None
//!
//
/////////////////////////////////////////////////////////////////////////////

u8 mip_get_payload_size(u8 *mip_buffer)
{
 mip_header *header_ptr = (mip_header *)mip_buffer; 
  
 //Null check info
 if(mip_buffer == 0)
   return 0;

 //Check that the header is valid
 if(mip_is_mip_packet(mip_buffer) != MIP_OK)
   return 0;
  
 return header_ptr->payload_size;  
}


////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u8 *mip_get_payload_ptr(u8 *mip_buffer)
//
//! @section DESCRIPTION
//! Returns a pointer to the start of the MIP payload data.  
//
//! @section DETAILS
//!
//! @param [in] u8 *mip_buffer  - pointer to memory containing the mip packet.
//
//! @returns  A pointer to the MIP payload data.\n
//!           NULL otherwise.
//
//! @section NOTES
//! 
//! None
//!
//
/////////////////////////////////////////////////////////////////////////////

u8 *mip_get_payload_ptr(u8 *mip_buffer)
{  
 //Null check info
 if(mip_buffer == 0)
   return 0;

 //Check that the header is valid
 if(mip_is_mip_packet(mip_buffer) != MIP_OK)
   return 0;
  
 return mip_buffer + sizeof(mip_header);  
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_is_checksum_valid(u8 *mip_buffer, u16 expected_checksum)
//
//! @section DESCRIPTION
//! Returns the state of the MIP packet checksum.  
//
//! @section DETAILS
//!
//! @param [in]     u8 *mip_buffer                  - pointer to memory containing the mip packet.
//
//! @retval MIP_OK                  - The checksum is valid.\n
//! @retval MIP_ERROR               - The pointer is NULL.\n
//! @retval MIP_INVALID_PACKET      - The buffer does not contain a valid MIP packet.
//
//! @section NOTES
//! 
//! None
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_is_checksum_valid(u8 *mip_buffer)
{
 u16 packet_checksum, expected_checksum, checksum_offset = 0; 
 mip_header *header_ptr = (mip_header *)mip_buffer; 

 
 //Null check info
 if(mip_buffer == 0)
   return MIP_ERROR;
 
 //Check that the header is valid
 if(mip_is_mip_packet(mip_buffer) != MIP_OK)
   return MIP_INVALID_PACKET;
 
 checksum_offset = header_ptr->payload_size + sizeof(mip_header);
 
 //Form the packet checksum 
 packet_checksum = ((u16)mip_buffer[checksum_offset]<<8) + (u16)mip_buffer[checksum_offset + 1];
 
 //Calculate the expected checksum
 expected_checksum = mip_calculate_checksum(mip_buffer);
 
 //Evaluate checksums
 if(packet_checksum == expected_checksum)
  return MIP_OK;
 else
  return MIP_CHECKSUM_ERROR;
}


////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_calculate_checksum(u8 *mip_buffer)
//
//! @section DESCRIPTION
//! Calculates the 16-bit Fletcher checksum for the MIP packet.  
//
//! @section DETAILS
//!
//! @param [in] u8 *mip_buffer  - pointer to memory containing the mip packet.
//
//! @returns  The MIP packet checksum, if mip_buffer contains a valid MIP packet.\n
//!           0x00 otherwise.
//
//! @section NOTES
//! 
//! None
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_calculate_checksum(u8 *mip_buffer)
{
 mip_header *header_ptr = (mip_header*)mip_buffer;   
 u16 i, checksum, checksum_range_size;
 u8 checksum_byte1 = 0, checksum_byte2 = 0; 

 
 //Null check info
 if(mip_buffer == 0)
  return 0;
 
 //Calculate the size of the checksum data range
 checksum_range_size = header_ptr->payload_size + MIP_HEADER_SIZE;
 
 //Check that the size is valid
 if(checksum_range_size > MIP_MAX_PACKET_SIZE - MIP_CHECKSUM_SIZE) 
  return 0;
   
 //Calculate the fletcher checksum
 for(i=0; i<checksum_range_size; i++)
 {
  checksum_byte1 += mip_buffer[i];
  checksum_byte2 += checksum_byte1;
 }
   
 //Form the packet checksum 
 checksum = ((u16)checksum_byte1<<8) + (u16)checksum_byte2;

 return checksum;
}

