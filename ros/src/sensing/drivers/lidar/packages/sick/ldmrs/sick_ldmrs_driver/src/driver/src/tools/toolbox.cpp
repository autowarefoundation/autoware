/**
 * Toolbox. Some useful functions.
 *
 */
#include <stdio.h>      /* for printf() and fprintf() */
#include <sys/socket.h> /* for socket(), bind(), and connect() */
#include <arpa/inet.h>  /* for sockaddr_in and inet_ntoa() */
#include <stdlib.h>     /* for atoi() and exit() */
#include <string.h>     /* for memset() */
#include "toolbox.hpp"
#include <iostream>
#include <iomanip>		// for std::setprecision
#include <sstream>		// for std::stringstream
#include "errorhandler.hpp"

//
// Write a binary trace output, e.g. of buffer contents.
// Can be used for debugging.
//
void traceBuffer(std::string headerText, BYTE* buffer, UINT32 len)
{
	// Table header
	printInfoMessage(headerText, true);

	// Length
	std::string line;
	line = "Length= " + toString(len) + " bytes.";
	printInfoMessage(line, true);
	
	// Contents
	UINT32 pos = 0;
	while (pos < len)
	{
		line = toHexString(pos) + ": ";
		for (UINT16 i=0; i< 16; i++)
		{
			line += toHexString(buffer[pos]) +  " ";
			pos++;
			if (pos >= len)
			{
				break;
			}
		}
		printInfoMessage(line, true);
	}
}

//
// String conversion: Values to a hex string.
// 0..16 --> 0..F
//
std::string toHexStringNibble(UINT8 val)
{
	std::string s = "0123456789ABCDEF";
	std::string c;
	if (val < 16)
	{
		c = s.substr(val, 1);
	}
	else
	{
		c = "x";
	}
	return c;
}

//
// UINT32-value to Hex-String
// Result: "xxxxxxxx"
//
std::string toHexString(UINT32 val)
{
	std::string s = toHexString((UINT16)(val >> 16));
	s += toHexString((UINT16)(val & 0xFFFF));
	return s;
}

// Ergebnis: "xxxx"
std::string toHexString(UINT16 val)
{
	std::string s = toHexStringNibble((UINT8)(val >> 12));
	s += toHexStringNibble((UINT8)((val >> 8) & 0xF));
	s += toHexStringNibble((UINT8)((val >> 4) & 0xF));
	s += toHexStringNibble((UINT8)(val & 0xF));
	return s;
}

// Ergebnis: "xx"
std::string toHexString(UINT8 val)
{
	std::string s1 = toHexStringNibble((UINT8)(val >> 4));
	std::string s2 = toHexStringNibble((UINT8)(val & 0x0F));
	std::string s = s1 + s2;
	return s;
}


//
// Konvertiert einen String in Kleinbuchstaben
//
std::string toLower(const std::string& text)
{
	std::string low;
	UINT32 i;
	unsigned char c;
	for (i=0; i < text.length(); i++)
	{
		c = text.at(i);
		if ((c >= 'A') || (c <= 'Z'))
		{
			// Grossbuchstabe umwandeln
			c += ('a' - 'A');
		}
		low += c;
	}
	
	return low;
}


//
// Konvertiere eine Angabe in [m] in Fuesse und Inches, als Text.
// Ausgabeformat ist <<ff' ii">>
//
std::string convertMeterToFeetAndInch(double m)
{
	std::ostringstream os;
	std::string text;

	// Vorzeichen verarbeiten
	if (m < 0.0)
	{
		os << "-";
		m = -m;
	}

	INT32 feet = (INT32)(m / 0.3048);
	INT32 inch = (INT32)((m - ((double)feet * 0.3048)) / 0.0254);
	if (feet > 0)
	{
		os << feet << "'";
	}
	if ((inch > 0) || (feet == 0))
	{
		os << inch << "\"";
	}
	text = os.str();

	// Ausgabe
	return text;
}



//
// String --> UINT16
//
UINT16 fromString(const std::string& text)
{
	int value;
	int conversions = sscanf(text.c_str(), "%d", &value);
	if (conversions == 1)
	{
		return (UINT16)value;
	}
	
	return 0;
}

/**
 * Konvertierung eines Hex-Zeichens -> Zahlenwert.
 */
int hexCharToValue(char c)
{
	int value = 0;
	
	if ((c >= '0') && (c <= '9'))
	{
		value = c - '0';
	}
	else if ((c >= 'A') && (c <= 'F'))
	{
		value = c - 'A' + 10;
	}
	else if ((c >= 'a') && (c <= 'f'))
	{
		value = c - 'a' + 10;
	}
	
	return value;
}


/**
 * Konvertierung Zahlenwert -> Hex-Zeichen
 */
char convertNibbleToHexChar(int value, bool useLowerCaseLetters)
{
	char c;
	
	if (value < 10)
	{
		c = '0' + value;
	}
	else
	{
		if (useLowerCaseLetters == false)
		{
			// Grossbuchstaben
			c = 'A' + value - 10;
		}
		else
		{
			// Kleinbuchstaben
			c = 'a' + value - 10;
		}
	}
	
	return c;
}

/**
 * Konvertiert einen UINT8-Wert in einen 2-Byte-Hex-Wert, z.B.:
 * b = 0x07, String = "07"
 * Buffer muss mindestens 2 Byte gross sein.
 */
void convertUINT8toHexString(UINT8 byte, char* buffer)
{
	UINT8 value = (byte >> 4);
	buffer[0] = convertNibbleToHexChar(value);
	value = byte & 0x0F;
	buffer[1] = convertNibbleToHexChar(value);
}

/**
 * Konvertiert drei UINT8-Werte (R, G, B) in einen HEX-String, z.B.:
 * R = 0x12, G=0x34, B=0x56 --> String = "123456"
 * Buffer muss mindestens 6 Byte gross sein.
 */
void convertRGBtoHexString(UINT8 r, UINT8 g, UINT8 b, char* buffer)
{
	convertUINT8toHexString(r, buffer);
	convertUINT8toHexString(g, &buffer[2]);
	convertUINT8toHexString(b, &buffer[4]);
}



/**
 * Bringt den Winkel in den Bereich ]PI..-PI]
 */
double makeAngleValid(double angle)
{
	const double twoPi = (2.0 * PI);
	
	while (angle >= PI)
	{
		angle -= twoPi;
	}
	while (angle < -PI)
	{
		angle += twoPi;
	}
	
	return angle;
}

/**
 *
 */
std::string toString(INT32 value)
{
	char c[16];
	sprintf(c, "%i", value);
	return (std::string(c));
}

/**
 *
 */
std::string toString(UINT32 value)
{
	char c[16];
	sprintf(c, "%i", value);
	return (std::string(c));
}

#if INTPTR_MAX != INT32_MAX
std::string toString(size_t value)
{
	char c[16];
	sprintf(c, "%zu", value);
	return (std::string(c));
}
#endif

/*
 * Konvertiere Zahl in formatierten String.
 * digits_before_decimal_point = 0..20
 * digits_after_decimal_point = 0..20
 *
 * Der Ergebnisstring ist formatiert gem. den Vorgaben, d.h. er hat
 * exakt die Laenge digits_before_decimal_point + digits_after_decimal_point + 1,
 * ausser, die Vorkommazahl passte nicht in die Laengenvorgabe
 * "digits_before_decimal_point", dann ist er entsprechend laenger.
 */
std::string doubleToString(double val,
						   std::string::size_type digits_before_decimal_point,
						   std::string::size_type digits_after_decimal_point)
{
	// Konvertierung in String
	std::string text = doubleToString(val, digits_after_decimal_point);

	// Laengen festlegen: Zuerst vor dem Dezimalpunkt
	const std::string::size_type dotPosition = text.find_first_of('.', 0);
	if (dotPosition != std::string::npos)
	{
		// Punkt gefunden
		if (dotPosition < digits_before_decimal_point)
		{
			// Zu kurz, also vorne auffuellen
			std::string::size_type numExtraSpaces = digits_before_decimal_point - dotPosition;
			text = std::string(numExtraSpaces, ' ') + text;
		}
	}

	// Gesamtlaenge pruefen und ggf. verlaengern. NOTE: This will
	// never happen because the first doubleToString() will always fill
	// up with zero at the end, doesn't it?
	if (text.length() < (digits_before_decimal_point + digits_after_decimal_point + 1))
	{
		// Spaces hinten anfuegen
		std::string::size_type numExtraSpaces =
			(digits_before_decimal_point + digits_after_decimal_point + 1) -  text.length();
		text += std::string(numExtraSpaces, ' ');
	}

	return text;
}

/**
 * Konvertiere Double-Zahl in String.
 */
std::string doubleToString(double val,
						   int digits_after_decimal_point)
{
	// Konvertierung in String
	std::stringstream sstr;
	sstr << std::fixed << std::setprecision(digits_after_decimal_point) << val;

	return sstr.str();
}

std::string toString(double val, int digits_after_decimal_point)
{
	return doubleToString(val, digits_after_decimal_point);
}


//
// Konvertiere einen String in seine Adresse und seinen Port
//
// Beispiel: "192.168.0.1:1234" -> 0x0100A8C0
//
void stringToIpTarget(std::string ipAdrStr, UINT32& ipAddress, UINT16& port)
{
	std::string addrStr;
	std::string portStr;
	
	if (ipAdrStr.length() < 3)
	{
		// Ungueltig
		return;
	}
	
	UINT32 adrVal = INADDR_NONE;
	UINT16 portVal = 0;
	
	// Port extrahieren
	size_t pos = ipAdrStr.find_first_of(':');
	if ((pos > 0) && (pos < (ipAdrStr.length() - 1)))
	{
		addrStr = ipAdrStr.substr(0, pos);
		portStr = ipAdrStr.substr(pos+1);
	}
	else
	{
		addrStr = ipAdrStr;
	}

	// Adresse
	adrVal = (UINT32)inet_addr(addrStr.c_str());	// 	inet_addr("127.0.0.1");
	ipAddress = adrVal;
	
	// Port
	if (portStr.length() > 0)
	{
		portVal = fromString(portStr);
		port = portVal;
	}
}


//
// Konvertiere die IP-Adresse (IPv4) in einen String der Form a.b.c.d:port
//
std::string ipTargetToString(UINT32 ipAddress, UINT16 port)
{
	std::string addr;
	addr = ipAdrToString(ipAddress);
	
	// Port
	addr += ":";
	addr += toString((UINT16)port);
	
	return addr;
}


//
// Konvertiere die IP-Adresse (IPv4) in einen String der Form a.b.c.d.
// OHNE PORT!
//
std::string ipAdrToString(UINT32 ipAddress)
{
	std::string addr;
	addr =  toString((UINT16)((ipAddress >> 0 ) & 0xFF)) + "." +
			toString((UINT16)((ipAddress >> 8 ) & 0xFF)) + "." +
			toString((UINT16)((ipAddress >> 16) & 0xFF)) + "." +
			toString((UINT16)((ipAddress >> 24) & 0xFF));

	return addr;
}


//
// Read an UINT32 from a buffer. The buffer has the value in Big Endian format.
//
UINT32 memread_UINT32(BYTE*& buffer)
{
	UINT32 value = (((UINT32)buffer[0]) << 24) +
					(((UINT32)buffer[1]) << 16) +
					(((UINT32)buffer[2]) << 8 ) +
					(((UINT32)buffer[3])      );
	buffer += 4;
	return value;
}


//
// Read an UINT16 from a buffer. The buffer has the value in Big Endian format.
//
UINT16 memread_UINT16(BYTE*& buffer)
{
	UINT16 value = (((UINT16)buffer[0]) << 8) +
					((UINT16)buffer[1]);
	buffer += 2;
	return value;
}

//
// Read an UINT8 from a buffer.
//
UINT8 memread_UINT8(BYTE*& buffer)
{
	UINT8 value = buffer[0];
	buffer++;
	return value;
}

//
// Read an INT16 from a buffer. The buffer has the value in Big Endian format.
//
INT16 memread_INT16(BYTE*& buffer)
{
	UINT16 value = (((UINT16)buffer[0]) << 8) +
					((UINT16)buffer[1]);
	buffer += 2;
	return (INT16)value;
}

//
// Read an INT32 from a buffer. The buffer has the value in Big Endian format.
//
INT32 memread_INT32(BYTE*& buffer)
{
	UINT32 value = (((UINT32)buffer[0]) << 24) +
					(((UINT32)buffer[1]) << 16) +
					(((UINT32)buffer[2]) << 8 ) +
					(((UINT32)buffer[3])      );
	buffer += 4;
	return (INT32)value;
}

//
// Read a string (with fixed length) from a buffer.
//
std::string memread_string(BYTE*& buffer, UINT16 length)
{
	std::string text;
	
	for (UINT16 i = 0; i<length; i++)
	{
		text += buffer[i];
	}
	buffer += length;
	
	return text;
}


/// Internal: Structure for reading/writing a float as an UINT32
union FloatInt
{
	float value_float;
	UINT32 value_int;
};

//
// Read a float value from a buffer. The buffer has the value in Big Endian format, so it
// cannot be read directly on x86 machines.
//
float memread_float(BYTE*& buffer)
{
	FloatInt floatint;
	floatint.value_int = memread_UINT32(buffer);
	return floatint.value_float;
}

//
// Write a float value to a buffer, and advance the buffer pointer.
// After writing, the buffer has the value in Big Endian format.
//
void memwrite_float(BYTE*& buffer, float value)
{
	FloatInt floatint;
	floatint.value_float = value;
	memwrite_UINT32(buffer, floatint.value_int);
}

//
// Write an INT32 to a buffer, and advance the buffer pointer.
// The buffer has the value in Big Endian format.
//
void memwrite_INT32(BYTE*& buffer, INT32 value)
{
	memwrite_UINT32(buffer, (UINT32)value);
}

//
// Write an UINT32 to a buffer, and advance the buffer pointer.
// The buffer has the value in Big Endian format.
//
void memwrite_UINT32(BYTE*& buffer, UINT32 value)
{
	buffer[0] = ((value >> 24) & 0xFF);
	buffer[1] = ((value >> 16) & 0xFF);
	buffer[2] = ((value >> 8 ) & 0xFF);
	buffer[3] = ((value      ) & 0xFF);
	buffer += 4;
}

//
// Write an INT16 to a buffer, and advance the buffer pointer.
// The buffer has the value in Big Endian format.
//
void memwrite_INT16(BYTE*& buffer, INT16 value)
{
	memwrite_UINT16(buffer, (UINT16)value);
}

//
// Write an UINT16 to a buffer, and advance the buffer pointer.
// The buffer has the value in Big Endian format.
//
void memwrite_UINT16(BYTE*& buffer, UINT16 value)
{
	buffer[0] = ((value >> 8 ) & 0xFF);
	buffer[1] = ((value      ) & 0xFF);
	buffer += 2;
}

//
// Write an UINT8 to a buffer, and advance the buffer pointer.
//
void memwrite_UINT8(BYTE*& buffer, UINT8 value)
{
	buffer[0] = value;
	buffer++;
}

//
// Write an INT8 to a buffer, and advance the buffer pointer.
//
void memwrite_INT8(BYTE*& buffer, INT8 value)
{
	buffer[0] = value;
	buffer++;
}

//
// Write a string to a buffer, and advance the buffer pointer.
//
void memwrite_string(BYTE*& buffer, std::string text)
{
	strncpy((char*)buffer, text.c_str(), text.length());
	buffer += text.length();
}
