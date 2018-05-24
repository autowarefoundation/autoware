//
// colab.cpp
//
// (c) 2010 SICK AG, Hamburg, Germany

#include "colab.hpp"
#include <cstring>
#include <cassert>
#include <stdexcept>
#include <stdlib.h>
#include <limits>

namespace colab
{

void addStringToBuffer(UINT8* buffer, UINT16& pos, const std::string& stringValue)
{
	UINT16 length = stringValue.length();
	strcpy((char*) &buffer[pos], stringValue.c_str());
	pos += length;
}



void addStringToBuffer(BYTE* buffer, const std::string& stringValue)
{
//	UINT16 length = stringValue.length();
	strcpy((char*) buffer, stringValue.c_str());
//	buffer += length;
}



std::string getStringFromBuffer(UINT8* buffer, UINT16& pos, UINT16 length)
{
	UINT16 start = pos;
	pos += length;
	return std::string((char *) &buffer[start], length);
}



std::string getStringFromBuffer(BYTE*& buffer, UINT16 length)
{
	std::string str((char *) &buffer[0], length);
	buffer += length;
	return str;
}



std::string getCommandStringFromBuffer(UINT8* buffer)
{
	return std::string((char*) &buffer[9], 2);
}



std::string getIdentifierFromBuffer(UINT8* buffer, UINT16& nextData, UINT16 bufferLength)
{
	UINT16 start;
	UINT16 length;

	if (buffer[11] == 0x20)
	{
		start = 12;
	}
	else
	{
		start = 11;
	}

	int i = start;
	do
	{
		if (i == bufferLength - 2)
		{
			// found checksum field -> end of buffer reached.
			nextData = 0; // indicates that there is no more data
			break;
		}
		if (buffer[i] == 0x20)
		{
			// found identifier delimiter
			nextData = i + 1; // points to next data field
			break;
		}
		i++;
	}
	while (true);

	length = i - start; // last byte of identifier

	return std::string((char*) &buffer[start], length);
}



void addFrameToBuffer(UINT8* sendBuffer, UINT8* cmdBuffer, UINT16* len)
{
	UINT16 pos = 0;
	UINT32 length = *len;

	// write header
	sendBuffer[pos++] = 0x02;
	sendBuffer[pos++] = 0x02;
	sendBuffer[pos++] = 0x02;
	sendBuffer[pos++] = 0x02;
	// Write payload length to buffer
	colab::addIntegerToBuffer<UINT32>(sendBuffer, pos, length + 1); // s counts to the payload length
	sendBuffer[pos++] = 's';

	// write telegram
	memcpy(&(sendBuffer[pos]), cmdBuffer, length);
	pos += length;

	// write checksum (of payload)
	UINT8 checksum = sendBuffer[8];
	for (int i = 9; i < pos; i++)
	{
		checksum = checksum ^ sendBuffer[i]; // XOR
	}
	colab::addIntegerToBuffer<UINT8>(sendBuffer, pos, checksum);

	*len = pos;
}

//
//  Returns the requested value. pos points then to the first byte of the next data field.
//
double getDoubleFromBuffer(UINT8* buffer, UINT16& pos)
{
	UINT16 width = sizeof(double);	// 8
//	UINT8* buffer2 = buffer;
//	T floatValue = memread<T>(buffer2);
	double* valuePtr = (double*)buffer;
	double value = *valuePtr;
	
	pos += width;
	return value;
}


UINT16 decodeUINT16(BYTE* buffer)
{
	UINT16 value = (((UINT16)buffer[0]) << 8) +
					((UINT16)buffer[1]);
	return value;
}


} // END namespace colab
