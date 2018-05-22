//
// colaa.cpp
//
// (c) 2011 SICK AG, Hamburg, Germany

#include "colaa.hpp"
#include <cstring>
#include <cassert>
#include <stdexcept>
#include "../tools/toolbox.hpp"	// for "toString"
#include "../tools/errorhandler.hpp"
#include <stdlib.h>
#include <limits>
// #include "Trace.hpp"

namespace colaa
{

//
// Konvertiert ein ASCII-Zeichen (= Hex-Wert-Nibble oder Ziffer) in seinen Zahlenwert.
//
UINT16 getValueOfChar(UINT8 c)
{
	UINT16 value = 0;

	if ((c >= '0') && (c <= '9'))
	{
		value = c - '0';
	}
	else if ((c >= 'A') && (c <= 'F'))
	{
		value = 10 + c - 'A';
	}
	else
	{
		// Keine HEX-Ziffer
		throw std::out_of_range("Unknown character where 0..9 or A..F was expected: '" + std::string(c, 1) + "'");
	}

	return value;
}

//
// Konvertiere ein Hex-Nibble (0..F) nach ASCII.
//
// Return value: UINT8 (= Char)
//
UINT8 nibbleToAscii(UINT8 value)
{
	UINT8 c;

	if (value > 0x0f)
	{
		throw std::out_of_range("Tried to convert value > 0x0f into hex-nibble: " + toString((INT16)value));
	}

	if (value < 10)
	{
		c = ('0' + value);
	}
	else
	{
		c = ('A' + (value - 10));
	}

	return c;
}


/**
 * Erzeuge den zu sendenden Kommandostring, inklusive sog. "23-Frame".
 * Der String wird im CoLa-A Format erzeugt und im Puffer abgelegt.
 * sendBuffer = Ziel-Puffer fuer die erzeugten Daten
 * cmdBuffer = Quelle-Puffer mit den Kommandodaten, aber ohne 23-Frame
 * len = Zeiger auf Laenge des cmdBuffer; bei Rueckgabe Laenge des sendBuffers.
 */
void addFrameToBuffer(UINT8* sendBuffer, UINT8* cmdBuffer, UINT16* len)
{
	UINT16 pos = 0;

	// Beginne mit dem 23-Frame-Header
	sendBuffer[pos++] = 0x02;
	sendBuffer[pos++] = 's';

	// Nun das Kommando
	memcpy(&(sendBuffer[pos]), cmdBuffer, *len);
	pos += *len;

//	for (UINT16 i = 0; i<(*len); i++)
//	{
//		sendBuffer[pos++] = cmdBuffer[i];
//	}

	// Schliesse den 23-Frame ab
	sendBuffer[pos++] = 0x03;

	// Fertig!
	*len = pos;
}
/**
 * Schreibe den UINT8-Wert als ASCII-HEX-String in den Puffer.
 * Es werden 1 bis 2 Bytes geschrieben.
 *
 * Return value: #Bytes
 */
UINT16 addUINT8ToBuffer(UINT8* buffer, UINT8 value)
{
	UINT16 len;

	len = addUINT32ToBuffer(buffer, (UINT32)value);

	return len;
}
/**
 * Schreibe den UINT16-Wert als ASCII-HEX-String in den Puffer.
 * Es werden 1 bis 4 Bytes (die 4 Nibbles) geschrieben.
 *
 * Return value: #Bytes
 */
UINT16 addUINT16ToBuffer(UINT8* buffer, UINT16 value)
{
	UINT16 len;

	len = addUINT32ToBuffer(buffer, (UINT32)value);

	return len;
}

/**
 * Int8-Wert in den Puffer schreiben (als Text).
 *
 * Return value: Laenge des hinzugefuegten Strings
 */
UINT16 addINT8ToBuffer(UINT8* buffer, INT8 value)
{
	UINT16 stellenwert;
	UINT8 c;
	UINT16 pos = 0;
	bool firstZero = true;

	// Vorzeichen
	if (value < 0)
	{
		buffer[pos++] = '-';
		value *= -1;
	}
	else
	{
		buffer[pos++] = '+';
	}

	// Dezimal-Konversion
	stellenwert = 100;

	while (stellenwert > 0)
	{
		c = value / stellenwert;
		if ((c != 0) || (firstZero == false) || (stellenwert == 1))
		{
			// Ziffer schreiben
			buffer[pos++] = ('0' + c);
		}
		if (c != 0)
		{
			// Wert != 0, d.h. ab jetzt auch jede 0 schreiben
			firstZero = false;
		}
		value -= c * stellenwert;
		stellenwert /= 10;
	}

	return pos;
}
/**
 * Int32-Wert in den Puffer schreiben (als Text).
 * Wird erstmal als HEX-Wert geschrieben...
 *
 * Return value: Laenge des hinzugefuegten Strings
 */
UINT16 addINT32ToBuffer(UINT8* buffer, INT32 value)
{
	UINT32 uValue = (UINT32)value;

	UINT16 pos = addUINT32ToBuffer(buffer, uValue);

	return pos;
}

/**
 * Schreibe den UINT32-Wert als ASCII-HEX-String in den Puffer.
 * Es werden 1 bis 8 Bytes geschrieben, je nach Zahlenwert.
 *
 * Return value: #Bytes
 */
UINT16 addUINT32ToBuffer(UINT8* buffer, UINT32 value)
{
	// Alle Nibbles durchgehen
	bool firstZero = true;
	UINT16 pos = 0;
	UINT8 nibble;

	for (INT16 i = 7; i >= 0; i -= 1)
	{
		nibble = (value >> (i * 4)) & 0x0F;
		if ((nibble != 0) || (firstZero == false) || (i == 0))
		{
			buffer[pos++] = nibbleToAscii(nibble);
			firstZero = false;
		}
	}

	return pos;
}
/**
 * Fuegt den Text in den Puffer ein (Kopie!).
 *
 * Return: Anzahl eingefuegte Zeichen.
 */
UINT16 addStringToBuffer(UINT8* buffer, const std::string& text)
{
	UINT16 len = text.length();

	// Kopiere den String
	strcpy ((char*)buffer, text.c_str());

	return len;
}

UINT16 detail::writeToBuffer(BYTE* buffer, double value)
{
	std::string tmp(toString(value, 6));
	printWarning("detail::writeToBuffer: Warning - Writing of floating-point values has not been cross-checked in Cola-A format!");
	return addStringToBuffer(buffer, tmp);
}

/**
 * Isoliere das naechste String-Token. Es geht bis zu naechsten
 * Leerzeichen; fuehrende Leerzeichen werden entfernt.
 * Der Ursprungs-String wird um das Token gekuerzt.
 */
std::string getNextStringToken(std::string* rxData)
{
	typedef std::string::size_type size_type;

	// Token finden
	size_type tokenStart = rxData->find_first_not_of(' ');	// Fuehrende Spaces loeschen
	size_type tokenEnd = rxData->find(' ', tokenStart);		// Folgendes Space finden
	size_type tokenLength = tokenEnd - tokenStart;

	// Token ausschneiden
	std::string token = rxData->substr(tokenStart, tokenLength);

	// Eingangsstring kuerzen
	*rxData = rxData->substr(tokenEnd + 1);

	// Rueckgabe des Token
	return token;
}


/**
 * Isoliere das naechste String-Token. Es geht bis zu naechsten
 * Leerzeichen; fuehrende Leerzeichen werden entfernt.
 * Der Ursprungs-Buffer bleibt erhalten!
 */
std::string getNextStringToken(UINT8* rxData)
{
//	typedef std::string::size_type size_type;
	
	UINT16 pos = 0;
	std::string token;
	
	// Fuehrende Spaces entfernen
	while (rxData[pos] != 0x00)
	{
		if (rxData[pos] != ' ')
		{
			break;
		}
		pos++;
	}
	
	// Etwas Gueltiges gefunden?
	if (rxData[pos] != 0x00)
	{
		// start zeigt auf etwas gueltiges.
		// So lange weiter fuellen, bis das Ende erreicht ist.
		while ((rxData[pos] != 0x00) && (rxData[pos] != ' '))
		{
			token += rxData[pos];
			pos++;
		}
	}

	// Rueckgabe des Token
	return token;
}


bool GetNibble(unsigned char data, unsigned char& rNibble)
{
	rNibble = 0;
	bool validHexData = true;
	if ((data >= '0') && (data <= '9'))
	{
		rNibble = (unsigned char)(data - '0');
	}
	else if ((data >= 'A') && (data <= 'F'))
	{
		rNibble = (unsigned char)(data - 'A' + 10);
	}
	else if ((data >= 'a') && (data <= 'f'))
	{
		rNibble = (unsigned char)(data - 'a' + 10);
	}
	else
	{
		validHexData = false;
	}
	return validHexData;
}

/**
 * Lese eine Real-Zahl aus dem Empfangspuffer.
 * Im Fehlerfall wird NaN zurueckgegeben.
 */
double decodeReal(std::string* rxData)
{
	double value = std::numeric_limits<double>::quiet_NaN();
	std::string text = colaa::getNextStringToken(rxData);
	if (text.empty() == false)
	{
		// Check representation
		if ((text[0] == '+') || (text[0] == '-'))
		{
			// ASCII
			value = atof(text.c_str());
		}
		else
		{
			// HEX
			// This simple conversion works only for fixed size!
			union
			{
				float f;
				unsigned char c[4];
			} converter;
			memset(&converter, 0, sizeof(converter));

			if (text.length() == 8)
			{
				int hexIndex = 0;
				int shift = 0;
				bool success = true;
				for (int i = 7; i >= 0; --i)
				{
					unsigned char nibble;
					success &= GetNibble(text[i], nibble);
					converter.c[hexIndex] |= (nibble << shift);
					hexIndex += (shift >> 2);
					shift ^= 4;
				}
				if (success == true)
				{
					value = converter.f;
				}
			}
		}
	}

	return value;
}

/**
 * Lese eine Real-Zahl aus dem Empfangspuffer.
 * Im Fehlerfall wird NaN zurueckgegeben.
 */
double decodeReal(const std::string& rxData)
{
	double value = std::numeric_limits<double>::quiet_NaN();
	const std::string& text = rxData;
	if (text.empty() == false)
	{
		// Check representation
		if ((text[0] == '+') || (text[0] == '-'))
		{
			// ASCII
			value = atof(text.c_str());
		}
		else
		{
			// HEX
			// TODO: Simple conversion working only for fixed size
			union
			{
				float f;
				unsigned char c[4];
			} converter;
			memset(&converter, 0, sizeof(converter));

			if (text.length() == 8)
			{
				int hexIndex = 0;
				int shift = 0;
				bool success = true;
				for (int i = 7; i >= 0; --i)
				{
					unsigned char nibble;
					success &= GetNibble(text[i], nibble);
					converter.c[hexIndex] |= (nibble << shift);
					hexIndex += (shift >> 2);
					shift ^= 4;
				}
				if (success == true)
				{
					value = converter.f;
				}
			}
		}
	}

	return value;
}


/**
 * Lese einen UINT32-Wert aus dem Empfangspuffer.
 */
UINT32 decodeUINT32(std::string* rxData)
{
	UINT32 value = 0;
	UINT32 tempVal;
	UINT32 factor = 1;
	UINT32 baseFactor = 10;
	UINT16 digits;

	// Zahlen-String extrahieren
	std::string number = colaa::getNextStringToken(rxData);

	if (number.at(0) == '+')
	{
		// Dezimalzahl
		baseFactor = 10;
		number = number.substr(1);
	}
	else
	{
		// Hexadezimalzahl
		baseFactor = 16;
	}
	digits = number.length();

	// Extrahiere die Zahl, letzte Stelle zuerst
	for (INT16 d = digits - 1; d >= 0; d -= 1)
	{
		tempVal = colaa::getValueOfChar(number.at(d));
		value += tempVal * factor;
		factor *= baseFactor;
	}

	return value;
}

/**
 * Lese einen INT16-Wert aus dem Empfangspuffer.
 */
INT16 decodeINT16(std::string* rxData)
{
	INT32 value = decodeINT32(rxData);
	assert ((value >= -32768) && (value <= 32767));
	return (INT16)value;
}

/**
 * Lese einen INT-Wert aus dem Empfangspuffer.
 * Ergebnis ist ein INT32-Wert.
 */
INT32 decodeINT32(std::string* rxData)
{
	INT32 value = 0;
	INT32 tempVal;
	INT32 factor = 1;
	INT32 baseFactor = 10;	// 10 = dez, 16 = hex
	INT32 sign = 1;		// 1 oder -1
	UINT16 digits;

	// Zahlen-String extrahieren
	std::string number = colaa::getNextStringToken(rxData);

	// Unterscheidung Pos/Neg/Hex
	if (number.at(0) == '+')
	{
		// pos. Dezimalzahl
		number = number.substr(1);	// Vorzeichen abschneiden
	}
	else if (number.at(0) == '-')
	{
		// neg. Dezimalzahl
		sign = -1;					// Neg. Vorzeichen
		number = number.substr(1);	// Vorzeichen abschneiden
	}
	else
	{
		// Hexadezimalzahl
		baseFactor = 16;			// Hex.
	}

	// Anzahl Ziffern
	digits = number.length();

	// Extrahiere die Zahl, letzte Stelle zuerst
	for (INT16 d = digits - 1; d >= 0; d -= 1)
	{
		tempVal = colaa::getValueOfChar(number.at(d));
		value += tempVal * factor;
		factor *= baseFactor;
	}

	// Vorzeichen einbauen
	value *= sign;

	return value;
}


/**
 * Lese einen INT16-Wert aus dem Empfangspuffer.
 */
INT16 decodeINT16(const std::string& rxData)
{
	INT32 value = decodeINT32(rxData);
	assert ((value >= -32768) && (value <= 32767));
	return (INT16)value;
}

/**
 * Lese einen INT-Wert aus dem Empfangspuffer.
 * Ergebnis ist ein INT32-Wert.
 */
INT32 decodeINT32(const std::string& rxData)
{
	INT32 value = 0;
	INT32 tempVal;
	INT32 factor = 1;
	INT32 baseFactor = 10;	// 10 = dez, 16 = hex
	INT32 sign = 1;		// 1 oder -1
	UINT16 digits;
	UINT16 offset = 0;

	// Zahlen-String extrahieren
	const std::string number = rxData;

	// Unterscheidung Pos/Neg/Hex
	if (number.at(0) == '+')
	{
		// pos. Dezimalzahl
//		number = number.substr(1);	// Vorzeichen abschneiden
		offset = 1;
	}
	else if (number.at(0) == '-')
	{
		// neg. Dezimalzahl
		sign = -1;					// Neg. Vorzeichen
//		number = number.substr(1);	// Vorzeichen abschneiden
		offset = 1;
	}
	else
	{
		// Hexadezimalzahl
		baseFactor = 16;			// Hex.
	}

	// Anzahl Ziffern
	digits = number.length();

	// Extrahiere die Zahl, letzte Stelle zuerst
	for (INT16 d = digits - 1; d >= offset; d -= 1)
	{
		tempVal = colaa::getValueOfChar(number.at(d));
		value += tempVal * factor;
		factor *= baseFactor;
	}

	// Vorzeichen einbauen
	value *= sign;

	return value;
}


/**
 * Lese einen String aus dem Empfangspuffer. Der Empfangspuffer wird anschliessend
 * um die Stringlaenge + 1 Zeichen (Trenn-Leerzeichen) gekuerzt.
 *
 * Ist der Parameter len = 0 (oder fehlt ganz), dann wird der naechste verfuegbare String
 * ausgelesen. Ist len > 0, dann werden genau len Zeichen gelesen.
 */
std::string decodeString(std::string* rxData, UINT16 len)
{
	std::string text;

	// String extrahieren
	if (len == 0)
	{
		// Keine spezielle Laenge gewuenscht.
		text = colaa::getNextStringToken(rxData);
	}
	else
	{
		// String bekannter Laenge ausschneiden
		text = rxData->substr(0, len);
		// Eingangsstring kuerzen
		*rxData = rxData->substr(len + 1);
	}

	return text;
}

//
// Lese einen UINT16-Wert aus dem Empfangspuffer.
//
UINT16 decodeUINT16(BYTE* buffer)
{
	std::string data = getNextStringToken(buffer);
	return decodeUINT16(data);
}

//
// Lese einen UINT16-Wert aus dem Empfangspuffer.
//
UINT16 decodeUINT16(std::string* rxData)
{
	UINT32 value;

	value = decodeUINT32(rxData);

	assert (value < 0x10000);
	return (UINT16)value;
}

/**
 * Lese einen UINT8-Wert aus dem Empfangspuffer.
 */
UINT8 decodeUINT8(std::string* rxData)
{
	UINT32 value;

	value = decodeUINT32(rxData);

	assert (value < 256);
	return (UINT8)value;
}

/**
 * Lese einen UINT16-Wert aus dem Empfangspuffer.
 */
UINT16 decodeUINT16(const std::string& rxData)
{
	UINT32 value;

	value = decodeUINT32(rxData);

	assert (value < 0x10000);
	return (UINT16)value;
}

/**
 * Lese einen UINT8-Wert aus dem Empfangspuffer.
 */
UINT8 decodeUINT8(const std::string& rxData)
{
	UINT32 value;

	value = decodeUINT32(rxData);

	assert (value < 256);
	return (UINT8)value;
}

/**
 * Lese einen INT-Wert aus dem Empfangspuffer.
 * Es wird vorausgesetzt, dass der übergebene String nur den Token für die Zahl enthält.
 * Ergebnis ist ein INT32-Wert.
 */
UINT32 decodeUINT32(const std::string& rxData)
{
	UINT32 value = 0;
	UINT32 factor = 1;
	UINT32 tempVal;
	UINT32 baseFactor = 10;	// 10 = dez, 16 = hex
	UINT16 digits;
	UINT16 offset = 0;

	// Zahlen-String extrahieren
	const std::string& number = rxData;

	// Unterscheidung Pos/Neg/Hex
	if (number.at(0) == '+')
	{
		// pos. Dezimalzahl
//		number = number.substr(1);	// Vorzeichen abschneiden
		offset = 1;
	}
	else
	{
		// Hexadezimalzahl
		baseFactor = 16;			// Hex.
	}

	// Anzahl Ziffern
	digits = number.length();

	// Extrahiere die Zahl, letzte Stelle zuerst
	for (INT16 d = digits - 1; d >= offset; d -= 1)
	{
		tempVal = colaa::getValueOfChar(number.at(d));
		value += tempVal * factor;
		factor *= baseFactor;
	}

	return value;
}

/**
 * Lese ein XByte-Array bekannter Laenge (1..4 Bytes) und verpacke es als UINT32-Wert.
 * Das 1. empfangene Byte steht in den unteren 8 Bit des Ergebniswerts, usw.
 *
 * len = Anzahl der Bytes (= Array-Elemente)
 */
UINT32 decodeXByte(std::string* rxData, UINT16 len)
{
	UINT32 value;
	UINT32 result = 0;

	assert (len < 5);	// Wir koennen nur bis zu 4 Bytes in einen UINT32 packen

	for (UINT16 i = 0; i < len; i++)
	{
		value = decodeUINT32(rxData);
		assert (value < 256);
		result += (value << (i * 8));
	}

	return result;
}

/**
 * Lese ein XByte-Array bekannter Laenge (1..4 Bytes) und verpacke es als UINT32-Wert.
 * Das 1. empfangene Byte steht in den unteren 8 Bit des Ergebniswerts, usw.
 *
 * HINWEIS: der Iterator wird weitergeschoben len-1 mal. Um das nächste Element zu lesen
 *          muss vorher ++tok aufgerufen werden.
 *
 * @param begin = Startpunkt, von wo aus einzelne Stringtokens in Zahlen verwandelt werden
 * @param end = Ende des containers, ueber den iteriert wird
 * @param len = Anzahl der Bytes (= Array-Elemente)
 */
/*
UINT32 decodeXByte(tokenizer::const_iterator& tok, const tokenizer::const_iterator& end, UINT16 len)
{
	UINT32 value;
	UINT32 result = 0;

	assert (len < 5);	// Wir koennen nur bis zu 4 Bytes in einen UINT32 packen

	tokenizer::const_iterator last = tok;

	for ( UINT16 i = 0; i < len && tok != end; ++tok, ++i)
	{
		value = decodeUINT32(*tok);
		assert (value < 256);
		result += (value << (i * 8));

		last = tok;
	}

	// set token one position back
	tok = last;


	return result;
}
*/

//
// Char-To-String-Umwandlung.
//
std::string convertRxBufferToString(UINT8* buffer, UINT16 bufferLen)
{
	buffer[bufferLen-1] = 0x00;	// End-Code 0x03 ueberschreiben mit 0x00
	std::string result = std::string((char*)(&(buffer[2])));
	return result;
}


} // END namespace colaa
