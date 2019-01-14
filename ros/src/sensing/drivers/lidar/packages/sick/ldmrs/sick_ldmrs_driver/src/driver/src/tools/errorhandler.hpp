/**
 * Fehler-Handler.
 *
 */
#ifndef ERRORHANDLER_HPP
#define ERRORHANDLER_HPP


#include "../BasicDatatypes.hpp"
#include <stdexcept>	// for throw


#define printInfoMessage(a, b)  (b ? infoMessage(a, b):doNothing())

// Fehler-"behandlung": Schreibe die Fehlermeldung und beende das Programm.
void dieWithError(std::string errorMessage);

void infoMessage(std::string message, bool print = true);

void printWarning(std::string message);

void printError(std::string message);

void doNothing();

#endif // ERRORHANDLER_HPP
