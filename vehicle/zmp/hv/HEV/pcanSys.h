/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef __LIBPCAN_H__
#define __LIBPCAN_H__
#include <pcan.h>

#if defined(LPSTR) || defined(HANDLE)
#error "double define for LPSTR, HANDLE found"
#endif

#define LPSTR  char *
#define HANDLE void *

#define CAN_BAUD_1M     0x0014  //   1 MBit/s
#define CAN_BAUD_500K   0x001C  // 500 kBit/s
#define CAN_BAUD_250K   0x011C  // 250 kBit/s
#define CAN_BAUD_125K   0x031C  // 125 kBit/s
#define CAN_BAUD_100K   0x432F  // 100 kBit/s
#define CAN_BAUD_50K    0x472F  //  50 kBit/s
#define CAN_BAUD_20K    0x532F  //  20 kBit/s
#define CAN_BAUD_10K    0x672F  //  10 kBit/s
#define CAN_BAUD_5K     0x7F7F  //   5 kBit/s

// parameter nCANMsgType
#define CAN_INIT_TYPE_EX		0x01	//Extended Frame
#define CAN_INIT_TYPE_ST		0x00	//Standart Frame

#define CAN_ERR_ANYBUSERR (CAN_ERR_BUSLIGHT | CAN_ERR_BUSHEAVY | CAN_ERR_BUSOFF)

#ifdef __cplusplus
  extern "C"
{
#endif

//HANDLE CAN_Open(WORD wHardwareType, ...);
//DWORD CAN_Init(HANDLE hHandle, WORD wBTR0BTR1, int nCANMsgType);
//DWORD CAN_Close(HANDLE hHandle);
//DWORD CAN_Status(HANDLE hHandle);
//DWORD CAN_Write(HANDLE hHandle, TPCANMsg* pMsgBuff);
//DWORD LINUX_CAN_Write_Timeout(HANDLE hHandle, TPCANMsg* pMsgBuff, int nMicroSeconds);
//DWORD CAN_Read(HANDLE hHandle, TPCANMsg* pMsgBuff);
//DWORD LINUX_CAN_Read(HANDLE hHandle, TPCANRdMsg* pMsgBuff);
//DWORD LINUX_CAN_Read_Timeout(HANDLE hHandle, TPCANRdMsg* pMsgBuff, int nMicroSeconds);
//DWORD CAN_ResetFilter(HANDLE hHandle);
//DWORD CAN_MsgFilter(HANDLE hHandle, DWORD FromID, DWORD ToID, int nCANMsgType);
//int LINUX_CAN_FileHandle(HANDLE hHandle);
//DWORD LINUX_CAN_Extended_Status(HANDLE hHandle, int *nPendingReads, int *nPendingWrites);
//DWORD CAN_VersionInfo(HANDLE hHandle, LPSTR lpszTextBuff);
int nGetLastError(void);
//HANDLE LINUX_CAN_Open(const char *szDeviceName, int nFlag);
//DWORD LINUX_CAN_Statistics(HANDLE hHandle, TPDIAG *diag);
//WORD LINUX_CAN_BTR0BTR1(HANDLE hHandle, DWORD dwBitRate);

#ifdef __cplusplus
}
#endif
#endif // __LIBPCAN_H__
