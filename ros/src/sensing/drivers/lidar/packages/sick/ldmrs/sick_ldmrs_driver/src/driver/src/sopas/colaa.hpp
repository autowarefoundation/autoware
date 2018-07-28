//
// colaa.hpp
//
// (c) 2011 SICK AG, Hamburg, Germany
//

#ifndef COLAA_HPP
#define COLAA_HPP

#include "../BasicDatatypes.hpp"
// #include <boost/tokenizer.hpp>

/**
 * Parser functions for a partly implementation of the CoLa-A
 * protocol, needed for communication with SICK sensors.
 */
namespace colaa
{

// typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
// typedef boost::char_separator<char> separator_type;

UINT16 getValueOfChar(UINT8 c);
UINT8 nibbleToAscii(UINT8 value);

void addFrameToBuffer(UINT8* sendBuffer, UINT8* cmdBuffer, UINT16* len);
UINT16 addUINT8ToBuffer(UINT8* buffer, UINT8 value);
UINT16 addUINT16ToBuffer(UINT8* buffer, UINT16 value);
UINT16 addINT8ToBuffer(UINT8* buffer, INT8 value);
UINT16 addINT32ToBuffer(UINT8* buffer, INT32 value);
UINT16 addUINT32ToBuffer(UINT8* buffer, UINT32 value);
UINT16 addStringToBuffer(UINT8* buffer, const std::string& text);
std::string getNextStringToken(std::string* rxData);

namespace detail
{
UINT16 writeToBuffer(BYTE* buffer, double value);
inline UINT16 writeToBuffer(BYTE* buffer, UINT8 value) { return addUINT8ToBuffer(buffer, value); }
inline UINT16 writeToBuffer(BYTE* buffer, INT8 value) { return addINT8ToBuffer(buffer, value); }
inline UINT16 writeToBuffer(BYTE* buffer, UINT16 value) { return addUINT16ToBuffer(buffer, value); }
inline UINT16 writeToBuffer(BYTE* buffer, INT16 value) { return addUINT16ToBuffer(buffer, value); }
inline UINT16 writeToBuffer(BYTE* buffer, UINT32 value) { return addUINT32ToBuffer(buffer, value); }
inline UINT16 writeToBuffer(BYTE* buffer, INT32 value) { return addINT32ToBuffer(buffer, value); }
inline UINT16 writeToBuffer(BYTE* buffer, const std::string& value) { return addStringToBuffer(buffer, value); }
}

double decodeReal(std::string* rxData);
INT16 decodeINT16(std::string* rxData);
INT32 decodeINT32(std::string* rxData);
UINT32 decodeUINT32(std::string* rxData);
UINT16 decodeUINT16(std::string* rxData);
UINT8 decodeUINT8(std::string* rxData);
UINT32 decodeXByte(std::string* rxData, UINT16 len);
std::string decodeString(std::string* rxData, UINT16 len = 0);
std::string convertRxBufferToString(UINT8* buffer, UINT16 bufferLen);

/// set of more efficient functions that do not copy strings (should be prefered in use together with the colaa::tokenizer)
double decodeReal(const std::string& rxData);
INT16 decodeINT16(const std::string& rxData);
INT32 decodeINT32(const std::string& rxData);
UINT8 decodeUINT8(const std::string& rxData);
UINT16 decodeUINT16(const std::string& rxData);
UINT32 decodeUINT32(const std::string& rxData);

//
UINT16 decodeUINT16(BYTE* buffer);

namespace detail
{
/// General template which is unimplemented; implemented specializations follow below
template<typename T>
inline T read (const std::string& str)
{
//	BOOST_STATIC_ASSERT(sizeof(T) == 0); // must not be instantiated
	return T(); // to avoid additional compiler errors
}
template<> inline double read<double>(const std::string& rxData) { return decodeReal(rxData); }
template<> inline INT16 read<INT16>(const std::string& rxData) { return decodeINT16(rxData); }
template<> inline INT32 read<INT32>(const std::string& rxData) { return decodeINT32(rxData); }
template<> inline INT8 read<INT8>(const std::string& rxData) { return decodeUINT8(rxData); }
template<> inline UINT8 read<UINT8>(const std::string& rxData) { return decodeUINT8(rxData); }
template<> inline UINT32 read<UINT32>(const std::string& rxData) { return decodeUINT32(rxData); }
template<> inline UINT16 read<UINT16>(const std::string& rxData) { return decodeUINT16(rxData); }
template<> inline std::string read<std::string>(const std::string& rxData) { return rxData; }
}

//
// Lese ein XByte-Array bekannter Laenge (1..4 Bytes) und verpacke es als UINT32-Wert.
// Das 1. empfangene Byte steht in den unteren 8 Bit des Ergebniswerts, usw.
//
// HINWEIS: der Iterator wird weitergeschoben len-1 mal. Um das naechste Element zu lesen
//          muss vorher ++tok aufgerufen werden.
//
// @param begin = Startpunkt, von wo aus einzelne Stringtokens in Zahlen verwandelt werden
// @param end = Ende des containers, ueber den iteriert wird
// @param len = Anzahl der Bytes (= Array-Elemente)
//
//UINT32 decodeXByte(tokenizer::const_iterator& tok, const tokenizer::const_iterator& end, UINT16 len);

} // END namespace colaa
#endif
