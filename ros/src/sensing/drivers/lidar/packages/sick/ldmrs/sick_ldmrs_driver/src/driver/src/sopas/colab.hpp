//
// colab.hpp
//
// (c) 2011 SICK AG, Hamburg, Germany
//

#ifndef COLAB_HPP
#define COLAB_HPP

#include "../BasicDatatypes.hpp"
#include <memory.h>	// for memread<>

//
// Parser functions for a partly implementation of the CoLa-B
// protocol, needed for communication with SICK sensors.
//
namespace colab
{

/**
 *  Stores the given stringValue to the given buffer. The string is inserted at position pos,
 *  and pos is increased by the number of bytes that were written.
 *  If the length of stringValue is longer than the remaining space of the buffer,
 *  than filling stopps at the end of buffer.
 *
 *  
 */
void addStringToBuffer(UINT8* buffer, UINT16& pos, const std::string& stringValue);


/**
 *  Stores the given stringValue to the given buffer. The buffer pointer will be
 *  increased
 */
void addStringToBuffer(BYTE* buffer, const std::string& stringValue);



/**
 *	Returns bytes from pos to pos+length as string.
 *          pos points to the next byte after (pos+length)
 */
std::string getStringFromBuffer(UINT8* buffer, UINT16& pos, UINT16 length);



/**
 *	Returns bytes from 0 to length as string.
 *          buffer will be shifted to be able to read the next value
 */
std::string getStringFromBuffer(BYTE*& buffer, UINT16 length);



/**
 *  Returns the Sopas command as string from buffer
 */
std::string getCommandStringFromBuffer(UINT8* buffer);




/**
 * 	buffer should point to the BEGIN of the buffer (incl. magic word),
 *  bufferLength is the length of the full buffer (incl. magic word and checksum).
 *
 *	Returns the variable or method identifier from buffer.
 *          pos points to the begin of the next data after identifier.
 *          is pos == 0, there is no more data
 */
std::string getIdentifierFromBuffer(UINT8* buffer, UINT16& nextData, UINT16 bufferLength);



/**
*   Stores the given cmdBuffer (sopas command) to the given sendBuffer (with header and checksum).
*/
void addFrameToBuffer(UINT8* sendBuffer, UINT8* cmdBuffer, UINT16* len);


//  Returns the requested value. pos points then to the first byte of the next data field.
double getDoubleFromBuffer(UINT8* buffer, UINT16& pos);

//
UINT16 decodeUINT16(BYTE* buffer);



// -----------------------------------------------------------------------------------------------------------------
//                                               TEMPLATE FUNCTIONS
// -----------------------------------------------------------------------------------------------------------------


// FIXME: use template functions from types.hpp instead !!! (memread/memwrite)

/**
 *  Accepts all integer types (any bit width and sign).
 *  If width of intValue is greater than given type T, the higher bytes are truncated.
 *
 *  The intValue will be stored in BigEndian byte order (as required for CoLaB).
 * 	pos points then to the first byte of the next data field..
 */
template<typename T>
void addIntegerToBuffer(UINT8* buffer, UINT16& pos, T intValue)
{
	UINT16 width = sizeof(T);

	for (int i = 0; i < width; i++)
	{
		buffer[pos+width-1-i] = (intValue >> (8 * i)) & 0xff; // BIG ENDIAN: width-1-i
	}

	pos += width;
}



/**
 *  Accepts all integer types (any bit width and sign).
 *
 *  Returns the requested intValue. pos points then to the first byte of the next data field.
 */
template<typename T>
T getIntegerFromBuffer(UINT8* buffer, UINT16& pos)
{
	UINT16 width = sizeof(T);
//		UINT8* buffer2 = buffer;
//		T intValue = memread<T>(buffer2);

	T intValue = 0;

	for (int i = 0; i < width; i++)
	{
		intValue += buffer[pos+width-1-i] << (8 * i);
	}

	pos += width;
	return intValue;
}



/**
 *  NOT IMPLEMENTED YET!!! It's only a skeleton.
 *
 *  Accepts float and double.
 *
 *  The floatValue will be stored in BigEndian byte order (as required for CoLaB).
 * 	pos points then to the first byte of the next data field.
 */
template<typename T>
void addFloatToBuffer(UINT8* buffer, UINT16& pos, T floatValue)
{
	UINT16 width = sizeof(T);



	pos += width;
}


} // END namespace colab
#endif
