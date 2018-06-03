/**
 * ErrorHandler.
 *
 */
#include <stdio.h>      // for printf() and fprintf()
#include <stdlib.h>     // for atoi() and exit()
// #include <string.h>     // for memset()
//#include <backward/iostream.h>	// fuer cout()

#include "errorhandler.hpp"
#include "Time.hpp"

// Print mutex to print thread-safe
pthread_mutex_t m_printMutex = PTHREAD_MUTEX_INITIALIZER;

/**
 * Der Name ist Programm...
 */
void doNothing()
{
}


/**
 * Fehler-"behandlung": Schreibe die Fehlermeldung und beende das Programm.
 */
void dieWithError(std::string errorMessage)
{
	// Mutex setzen
	pthread_mutex_lock(&m_printMutex);
	
	// Nachricht schreiben
    printError(errorMessage.c_str());
	
	// Mutex wieder freigeben
	pthread_mutex_unlock(&m_printMutex);

	// Programm abbrechen
    exit(1);
}

/**
 * Info-Text auf die Ausgabe schreiben.
 */
void infoMessage(std::string message, bool print)
{
	if (print == true)
	{
		Time t = Time::now();
	
		// Mutex setzen
		pthread_mutex_lock(&m_printMutex);
		
		// Nachricht schreiben
		printf ("%s ", t.toString().c_str());
		printf ("Info: %s\n", message.c_str());
		fflush(0);

		// Mutex wieder freigeben
		pthread_mutex_unlock(&m_printMutex);
	}
}



//
// Warnungs-Text auf die Ausgabe schreiben.
//
void printWarning(std::string message)
{
	Time t = Time::now();
	
	// Mutex setzen
	pthread_mutex_lock(&m_printMutex);
		
	printf ("%s ", t.toString().c_str());
	printf ("Warning: %s\n", message.c_str());
	fflush(0);
		
	// Mutex wieder freigeben
	pthread_mutex_unlock(&m_printMutex);
}

//
// Fehler-Text auf die Ausgabe schreiben.
//
void printError(std::string message)
{
	Time t = Time::now();
	
	// Mutex setzen
	pthread_mutex_lock(&m_printMutex);
		
	printf ("%s ", t.toString().c_str());
	printf ("ERROR: %s\n", message.c_str());
	fflush(0);
	
	// Mutex wieder freigeben
	pthread_mutex_unlock(&m_printMutex);
}
