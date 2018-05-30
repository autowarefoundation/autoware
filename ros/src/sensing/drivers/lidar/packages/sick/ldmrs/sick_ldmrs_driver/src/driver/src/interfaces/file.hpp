//
// File.hpp
//
// File reader.
//
// Sick AG
//
// HISTORY
//
// 1.0.0	2013-02-12, VWi
//			Initial version.
//



#ifndef FILE_HPP
#define FILE_HPP

#include "../BasicDatatypes.hpp"
#include "../tools/Mutex.hpp"
#include "../tools/SickThread.hpp"
#include <list>
// Ausgabestream und Kompression
#include <iostream>
#include <fstream>


//
// 
//
class File
{
public:
	File();
	~File();
	
	std::string getFilename();
	void close();
	bool open(std::string inputFileName, bool beVerbose = false);
	
	// Read callback (for being called when data is available)
	typedef void (*ReadFunction)(void* obj, UINT8* inputBuffer, UINT32& numBytes);	//  ReadFunction
	void setReadCallbackFunction(ReadFunction readFunction, void* obj);
	
	// Information if the connection is disconnected.
	typedef void (*DisconnectFunction)(void* obj);								//  Called on disconnect
	void setDisconnectCallbackFunction(DisconnectFunction discFunction, void* obj);


private:
	bool m_beVerbose;
	
	// Thread stuff
	void startReadThread();
	void readThreadFunction(bool& endThread, UINT16& waitTimeMs);
	SickThread<File, &File::readThreadFunction> m_readThread;
	INT32 readInputData();


	// File
	std::string m_inputFileName;
	std::ifstream m_inputFileStream;	// input file stream
	Mutex m_inputFileMutex;
	
	// Callbacks
	ReadFunction m_readFunction;		// Receive callback
	void* m_readFunctionObjPtr;			// Object of the Receive callback
	DisconnectFunction m_disconnectFunction;
	void* m_disconnectFunctionObjPtr;	// Object of the Disconect callback
};

#endif // FILE_HPP
