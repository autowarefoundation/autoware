//
// File.cpp
//
// Read from a binary file.
//

#include "../tools/errorhandler.hpp"
#include "../tools/toolbox.hpp"
#include <stdio.h>      // for sprintf()
#include <string.h>     // for memset()
#include "file.hpp"


File::File()
{
	m_beVerbose = false;
	
 	m_readThread.m_threadShouldRun = false;
	
	m_inputFileName = "";
 	m_readFunction = NULL;
	m_readFunctionObjPtr = NULL;
 	m_disconnectFunction = NULL;
	m_disconnectFunctionObjPtr = NULL;
}

//
// Destruktor.
//
File::~File(void)
{
	close();
}


//
// Open the input file for reading.
//
bool File::open(std::string inputFileName, bool beVerbose)
{
	m_beVerbose = beVerbose;
	
	close();

	// Store the file name for later use
	m_inputFileName = inputFileName;
	
	// Open the file
	m_inputFileStream.open(m_inputFileName.c_str(), std::ios_base::binary | std::ios_base::in);
	
	// Check if the file was opened
	if (m_inputFileStream.fail() == true)
	{
		printError("File::open(): Failed to open the file " + m_inputFileName + ", aborting!");
		return false;	// Exit here
	}
	
	startReadThread();
	
	return true;
}

//
// Close the input file, if it was open.
//
void File::close()
{
	if  (m_inputFileStream.is_open() == true)
	{
		m_inputFileStream.close();
	}
	m_inputFileName = "";
}

//
// Define the Read-Callback-Funktion.
//
void File::setReadCallbackFunction(File::ReadFunction readFunction, void* obj)
{
	m_readFunction = readFunction;
	m_readFunctionObjPtr = obj;
	
	// Start the read thread
	startReadThread();
}

//
//
//
void File::startReadThread()
{
	if ((m_readThread.isRunning() == false) &&
		(m_readFunctionObjPtr != NULL) &&
		(m_inputFileName != ""))
	{
		m_readThread.run(this);
	}
}

//
// Lese-Thread (Main function).
//
void File::readThreadFunction(bool& endThread, UINT16& waitTimeMs)
{
	INT32 result;

	// Read
	result = readInputData();

	// Result
	if (result < 0)
	{
		// Verbindung wurde abgebrochen
		if (m_readThread.m_threadShouldRun == true)
		{
			// Wir sollten eigentlich noch laufen!
			printInfoMessage("File::readThreadMain: End of file reached! Read thread terminates now.", m_beVerbose);
		}
		waitTimeMs = 0;
	}
	else if (result == 0)
	{
		// No data. We may have reached the end of the file. In any case, there is nothing
		// more we can do!
		waitTimeMs = 1;
		endThread = true;
	}
	else
	{
		// Wir haben etwas empfangen, also nicht schlafen
		waitTimeMs = 10;
	}
}


//
// Read some data from the file.
// Called from the read thread!
//
// Return value:
// > 0: Size of the data
// = 0: No data available
// < 0: Failed to read data, e.g. end of file
//
INT32 File::readInputData()
{
	// Prepare the input buffer
//	const UINT16 max_length = 8192;
	const std::streamsize max_length = 8192;
	char inBuffer[max_length];
	INT32 recvMsgSize = 0;
	
	// Ist die Verbindung offen?
	if (m_inputFileStream.is_open() == false)
	{
		printError("File::readInputData: File is not open, aborting!");
		return -1;
	}
		
	// Read some data, if any
	m_inputFileStream.read(inBuffer, max_length);	// Read
	recvMsgSize = m_inputFileStream.gcount();		// Get number of read bytes
	
	if (recvMsgSize > 0)
	{
		// Success
		printInfoMessage("File::readInputData: Read " + toString(recvMsgSize) + " bytes from the connection.", m_beVerbose);
		
		// Falls eine Callback-Funktion definiert ist, rufe sie auf mit den
		// empfangenen Daten. If not, discard the data!
		if (m_readFunction != NULL)
		{
			// Die Daten an die Callback-Funktion uebergeben
			UINT32 length_uint32 = (UINT32)recvMsgSize;
			m_readFunction(m_readFunctionObjPtr, (UINT8*)inBuffer, length_uint32);
		}
		else
		{
			printWarning("File::readInputData: Discarding data because there is no callback function!");
		}
	}
	else if (recvMsgSize == 0)
	{
		// Verbindungsabbruch
		printInfoMessage("File::readInputData: Read 0 bytes - end of file or file connection is lost!", true);
		
		// Informieren?
		if (m_disconnectFunction != NULL)
		{
			m_disconnectFunction(m_disconnectFunctionObjPtr);
		}
		
		// Mutex setzen
		ScopedLock lock(&m_inputFileMutex);

		m_inputFileStream.close();
	}
	
	return recvMsgSize;
}

//
// Setzt die Funktion, die bei einem Disconnect-Ereignis aufgerufen wird.
//
void File::setDisconnectCallbackFunction(DisconnectFunction discFunction, void* obj)
{
	m_disconnectFunction = discFunction;
	m_disconnectFunctionObjPtr = obj;
}

