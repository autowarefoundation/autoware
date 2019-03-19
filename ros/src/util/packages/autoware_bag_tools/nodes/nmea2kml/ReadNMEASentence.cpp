//============================================================================
// Name        : ReadNMEASentence.cpp
// Author      : pino
// Version     :
// Copyright   : free
// Description :
//============================================================================

#include <iostream>
#include <cstdlib>
#include <string.h>
#include <stdio.h>
#include "ReadNMEASentence.h"

namespace NMEA_PARSER
{
using namespace std;
typedef unsigned int	uint;
typedef unsigned char 	uchar;

void ReadNMEASentence::Parse(char letter)
{
	switch (Stat)
	{
	case waitFirstChar :
		if (letter == '$')
		{
			Stat = command;
			calculatedCheckSum = 0;
			receivedCheckSum = 0;
			nmeaCmd = "";
			dataString = "";
		}
		break;

	case command :
		if (letter != ',')
		{
			nmeaCmd += letter;
		}
		else
		{
			Stat = data;
		}
		calculatedCheckSum ^= letter;
		break;

	case data :
		if (letter != '*')
		{
			dataString += letter;
			calculatedCheckSum ^= letter;
		}
		else
		{
			Stat = checkSum1;
		}
		break;

	case checkSum1 :
		if (letter >= '0' && letter <= '9')
		{
			receivedCheckSum = ((letter - '0')&0x0f)<<4;
		}
		else if (letter >= 'A' && letter <= 'F')
		{
			receivedCheckSum = ((letter - 'A' + 10)&0x0f)<<4;
		}
		else receivedCheckSum = 0;
		Stat = checkSum2;
		break;

	case checkSum2 :
		if (letter >= '0' && letter <= '9')
		{
			receivedCheckSum |= (letter - '0')&0x0f;
		}
		else if (letter >= 'A' && letter <= 'F')
		{
			receivedCheckSum |= ((letter - 'A' + 10)&0x0f);
		}
		else receivedCheckSum = 0;
		Stat = waitFirstChar;

		if (receivedCheckSum == calculatedCheckSum)
		{
			commandCount++;
			if (nmeaCmd == "GPRMC" || nmeaCmd == "GNRMC")
			{
				GxRMC();
			}
			else if (nmeaCmd == "HEHDT" || nmeaCmd == "GPHDT")
			{
				HEHDT();
			}
			else if (nmeaCmd == "GPGGA")
			{
				GPGGA();
			}
		}

		break;

	default :
		break;
	}
}
ReadNMEASentence::ReadNMEASentence()
{
	Stat = waitFirstChar;
	receivedCheckSum = 0;
	calculatedCheckSum = 0;

	memset(&gxrmc, 0, sizeof(gxrmc));
	memset(&gpgga, 0, sizeof(gpgga));
	memset(&hehdt, 0, sizeof(hehdt));

	commandCount = 0;
}
ReadNMEASentence::~ReadNMEASentence()
{

}
string ReadNMEASentence::GetField(const string dataString, uint *pos)
{
	int cur_pos = *pos;
	int cut_pos = dataString.find(',', cur_pos);
	string result = "";

	if (cut_pos != (int)string::npos)
	{
		result = dataString.substr(cur_pos, cut_pos - 1);
		cur_pos = cut_pos + 1;
	}
	else {
		cur_pos = 0;
	}
	*pos = cur_pos;
	return result;
}
void ReadNMEASentence::GxRMC()
{
	uint currentPos = 0;
	string strBuf;
	char pBuf[32] = { 0 };

	// time
	strBuf = GetField(dataString, &currentPos);
	if (currentPos == 0) return;
	pBuf[0] = strBuf[0];
	pBuf[1] = strBuf[1];
	pBuf[2] = '\0';
	gxrmc.hour = atoi(pBuf);
	pBuf[0] = strBuf[2];
	pBuf[1] = strBuf[3];
	gxrmc.min = atoi(pBuf);
	pBuf[0] = strBuf[4];
	pBuf[1] = strBuf[5];
	gxrmc.sec = atoi(pBuf);

	// status
	strBuf = GetField(dataString, &currentPos);
	if (currentPos == 0) return;
	gxrmc.status = strBuf[0];

	// latitude
	strBuf = GetField(dataString, &currentPos);
	if (currentPos == 0) return;
	pBuf[0] = strBuf[0];
	pBuf[1] = strBuf[1];
	pBuf[2] = '\0';
	gxrmc.latitude = atof(pBuf);
	strBuf = strBuf.substr(2, strBuf.find(',') - 2);
	gxrmc.latitude += atof(strBuf.c_str()) / 60.0;

	strBuf = GetField(dataString, &currentPos);
	if (currentPos == 0) return;
	if (strBuf[0] == 'S') gxrmc.latitude = -gxrmc.latitude;

	// longitude
	strBuf = GetField(dataString, &currentPos);
	if (currentPos == 0) return;
	pBuf[0] = strBuf[0];
	pBuf[1] = strBuf[1];
	pBuf[2] = strBuf[2];
	pBuf[3] = '\0';
	gxrmc.longitude = atof(pBuf);
	strBuf = strBuf.substr(3, strBuf.find(',') - 3);
	gxrmc.longitude += atof(strBuf.c_str()) / 60.0;

	strBuf = GetField(dataString, &currentPos);
	if (currentPos == 0) return;
	if (strBuf[0] == 'W') gxrmc.longitude = -gxrmc.longitude;

	// ground speed
	strBuf = GetField(dataString, &currentPos);
	if (currentPos == 0) return;
	gxrmc.groundSpeed = atof(strBuf.c_str());

	// direction
	strBuf = GetField(dataString, &currentPos);
	if (currentPos == 0) return;
	gxrmc.direction = atof(strBuf.c_str());

	// date
	strBuf = GetField(dataString, &currentPos);
	if (currentPos == 0) return;
	pBuf[0] = strBuf[0];
	pBuf[1] = strBuf[1];
	pBuf[2] = '\0';
	gxrmc.day = atoi(pBuf);
	pBuf[0] = strBuf[2];
	pBuf[1] = strBuf[3];
	gxrmc.month = atoi(pBuf);
	pBuf[0] = strBuf[4];
	pBuf[1] = strBuf[5];
	gxrmc.year = atoi(pBuf) + 2000;

	// magnetic Variation
	strBuf = GetField(dataString, &currentPos);
	if (currentPos == 0) return;
	gxrmc.magVariation = atof(strBuf.c_str());

	strBuf = GetField(dataString, &currentPos);
	if (currentPos == 0) return;
	if (strBuf[0] == 'W') gxrmc.magVariation = -gxrmc.magVariation;

	gxrmc.count++;
}

void ReadNMEASentence::GPGGA()
{
	uint currentPos = 0;
	string strBuf;
	char pBuf[32] = { 0 };

	// time
	strBuf = GetField(dataString, &currentPos);
	if (currentPos == 0) return;
	pBuf[0] = strBuf[0];
	pBuf[1] = strBuf[1];
	pBuf[2] = '\0';
	gpgga.hour = atoi(pBuf);
	pBuf[0] = strBuf[2];
	pBuf[1] = strBuf[3];
	gpgga.min = atoi(pBuf);
	pBuf[0] = strBuf[4];
	pBuf[1] = strBuf[5];
	gpgga.sec = atoi(pBuf);

	// latitude
	strBuf = GetField(dataString, &currentPos);
	if (currentPos == 0 || strBuf.size()==0) return;
	pBuf[0] = strBuf[0];
	pBuf[1] = strBuf[1];
	pBuf[2] = '\0';
	gpgga.latitude = atof(pBuf);
	strBuf = strBuf.substr(2, strBuf.find(',') - 2);
	gpgga.latitude += atof(strBuf.c_str()) / 60.0;

	strBuf = GetField(dataString, &currentPos);
	if (currentPos == 0) return;
	if (strBuf[0] == 'S') gpgga.latitude = -gpgga.latitude;

	// longitude
	strBuf = GetField(dataString, &currentPos);
	if (currentPos == 0) return;
	pBuf[0] = strBuf[0];
	pBuf[1] = strBuf[1];
	pBuf[2] = strBuf[2];
	pBuf[3] = '\0';
	gpgga.longitude = atof(pBuf);
	strBuf = strBuf.substr(3, strBuf.find(',') - 3);
	gpgga.longitude += atof(strBuf.c_str()) / 60.0;

	strBuf = GetField(dataString, &currentPos);
	if (currentPos == 0) return;
	if (strBuf[0] == 'W') gpgga.longitude = -gpgga.longitude;

	strBuf = GetField(dataString, &currentPos);
	if (currentPos == 0) return;
	gpgga.mode = strBuf[0];

	strBuf = GetField(dataString, &currentPos);
	if (currentPos == 0) return;
	gpgga.satellites = atoi(strBuf.c_str());

	strBuf = GetField(dataString, &currentPos);
	if (currentPos == 0) return;
	gpgga.hdop = atof(strBuf.c_str());

	strBuf = GetField(dataString, &currentPos);
	if (currentPos == 0) return;
	gpgga.altitude = atof(strBuf.c_str());

	gpgga.count++;
}

void ReadNMEASentence::HEHDT()
{
	uint currentPos = 0;
	string strBuf;
	// True heading of the vessel (only Hemisphere GPS)
	strBuf = GetField(dataString, &currentPos);
	if (currentPos == 0) return;
	hehdt.trueHeading = atof(strBuf.c_str());
	hehdt.count++;
}

}

