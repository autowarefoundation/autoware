/**
 * Toolbox. Some useful functions.
 *
 */
#ifndef TOOLBOX_HPP
#define TOOLBOX_HPP

#include <stdio.h>      /* for printf() and fprintf() */
#include <sys/socket.h> /* for socket(), bind(), and connect() */
#include <arpa/inet.h>  /* for sockaddr_in and inet_ntoa() */
#include <stdlib.h>     /* for atoi() and exit() */
#include <string.h>     /* for memset() */
#include "../BasicDatatypes.hpp"	// for PI

// Konvertierung eines Hex-Zeichens -> Zahlenwert.
int hexCharToValue(char c);

// Konvertierung Zahlenwert (0-15) -> Hex-Zeichen.
char convertNibbleToHexChar(int value, bool useLowerCaseLetters = false);

// Konvertiert Byte (0-255) in Hex-String 00-FF
void convertUINT8toHexString(UINT8 byte, char* buffer);

// Konvertiert 3 Byte-Werte (RGB) in einen char-String
void convertRGBtoHexString(UINT8 r, UINT8 g, UINT8 b, char* buffer);

// Bringt den Winkel in den Bereich ]PI..-PI]
double makeAngleValid(double angle);

// Konvertiere eine IP-Adresse (IPv4)+Port in einen String
std::string ipTargetToString(UINT32 ipAddress, UINT16 port);
// Konvertiere eine reine IP-Adresse (IPv4), ohne Port, in einen String
std::string ipAdrToString(UINT32 ipAddress);

// Konvertiere einen String (IP-Adresse (IPv4)+Port) in seine Bestandteile.
// Enthaelt der String keinen Port, bleibt dieser unveraendert.
void stringToIpTarget(std::string ipAdrStr, UINT32& ipAddress, UINT16& port);

// Konvertiere eine Integer-Zahl in einen Hex-String
std::string toHexString(UINT32 val);
std::string toHexString(UINT16 val);
std::string toHexString(UINT8 val);
std::string toHexStringNbble(UINT8 val);

// Konvertiere eine Integer-Zahl in einen String
std::string toString(UINT32 val);
std::string toString(INT32 val);
#if INTPTR_MAX != INT32_MAX
std::string toString(size_t val);
#endif

// Konvertiere eine Gleitkomma-Zahl in einen String
std::string toString(double val, int digits_after_decimal_point);
std::string doubleToString(double val, int digits_after_decimal_point);
std::string doubleToString(double val, int digits_before_decimal_point, int digits_after_decimal_point);

// Converts a length in [m] to a string with feet and inches. Intended for text output.
std::string convertMeterToFeetAndInch(double m);

// Konvertiere einen String in eine Zahl
UINT16 fromString(const std::string& text);

// Konvertiere einen String in Kleinbuchstaben
std::string toLower(const std::string& text);

//
// Write a binary trace output, e.g. of buffer contents.
// Can be used for debugging.
//
void traceBuffer(std::string headerText, BYTE* buffer, UINT32 len);

//
// Access functions for binary data in buffers (big endian format)
//
UINT32 memread_UINT32(BYTE*& buffer);
UINT16 memread_UINT16(BYTE*& buffer);
UINT8 memread_UINT8(BYTE*& buffer);
INT32 memread_INT32(BYTE*& buffer);
INT16 memread_INT16(BYTE*& buffer);
float memread_float(BYTE*& buffer);
std::string memread_string(BYTE*& buffer, UINT16 length);

void memwrite_float(BYTE*& buffer, float value);
void memwrite_INT32(BYTE*& buffer, INT32 value);
void memwrite_UINT32(BYTE*& buffer, UINT32 value);
void memwrite_INT16(BYTE*& buffer, INT16 value);
void memwrite_UINT16(BYTE*& buffer, UINT16 value);
void memwrite_INT8(BYTE*& buffer, INT8 value);
void memwrite_UINT8(BYTE*& buffer, UINT8 value);
void memwrite_string(BYTE*& buffer, std::string text);

#endif // TOOLBOX_HPP
