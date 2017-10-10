/////////////////////////////////////////////////////////////////////////////
//
//! @file    byteswap_utilities.c 
//! @author  Nathan Miller
//! @version 1.1
//
//! @description Utility functions for byteswapping
//
// External dependencies:
//
//  mip_types.h
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


//
//Include Files
//

#include "byteswap_utilities.h"



/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void byteswap(void *in, void *out, u16 size)
//
//! @section DESCRIPTION
//! Byteswap data \c in and store it in location \c out.  
//
//! @section DETAILS
//!
//! @param [in]  void *in  - byte pointer to the data to be byteswapped.
//! @param [out] void *out - byte pointer to the memory to store the result.
//! @param [in]  u16 size  - the size of the data to byteswap in bytes.
//
//! @section NOTES
//! 
//! None
//!
//
/////////////////////////////////////////////////////////////////////////////

void byteswap(void *in, void *out, u16 size)
{
 u16 i;
 u8 *byte_ptr_in  = (u8*)in;
 u8 *byte_ptr_out = (u8*)out;
 
 //Null check in
 if(byte_ptr_in == 0)
   return;
  
 //Null check out
 if(byte_ptr_out == 0)
   return;
  
 for(i=0;i<size;i++)
 {
  byte_ptr_out[i] = byte_ptr_in[size - i - 1]; 
 }
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void byteswap_inplace(void *data, u16 size)
//
//! @section DESCRIPTION
//! Byteswap a memory location in place.  
//
//! @section DETAILS
//!
//! @param [in,out] void *data - byte pointer to the data to byteswap.
//! @param [in]     u16 size   - the size of the data to byteswap in bytes.
//
//! @section NOTES
//! 
//! None
//!
//
/////////////////////////////////////////////////////////////////////////////

void byteswap_inplace(void *data, u16 size)
{
 u8  temp;
 u8 *start = (u8*)data, *end = (u8*)data + size - 1;
 
 while(end > start)
 {
  temp   = *start;
  *start = *end;
  *end   = temp;
  
  start++;
  end--;
 }  
}