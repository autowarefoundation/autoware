//
// TCP.hpp
//
// Ethernet TCP data sender/receiver.
//
// Sick AG
//
// HISTORY
//
// 1.0.0	10.11.2011, VWi
//			Initial version.
//



#ifndef TCP_HPP
#define TCP_HPP

#include "../BasicDatatypes.hpp"
#include <sys/socket.h> /* for socket(), bind(), and connect() */
#include <arpa/inet.h>  /* for sockaddr_in and inet_ntoa() */
#include "../tools/Mutex.hpp"
#include "../tools/SickThread.hpp"
#include <list>


//
// Sender and receiver for data over a TCP connection. Client!
//
class Tcp
{
public:
	Tcp();
	~Tcp();

	// Opens the connection.
	bool open(std::string ipAddress, UINT16 port, bool enableVerboseDebugOutput = false);
	bool open(UINT32 ipAddress, UINT16 port, bool enableVerboseDebugOutput = false);
	void close();											// Closes the connection, if it was open.
	bool isOpen();	// "True" if a connection is currently open.
	bool write(UINT8* buffer, UINT32 numberOfBytes);		// Writes numberOfBytes bytes to the open connection.
	std::string readString(UINT8 delimiter);				// Reads a string, if available. Strings are separated with the delimiter char.

	// Buffer read function (for polling)
	UINT32 getNumReadableBytes();							// Returns the number of bytes currently available for reading.
	UINT32 read(UINT8* buffer, UINT32 bufferLen);			// Reads up to bufferLen bytes from the buffer.

	// Read callbacks (for being called when data is available)
	typedef void (*ReadFunction)(void* obj, UINT8* inputBuffer, UINT32& numBytes);	//  ReadFunction
	void setReadCallbackFunction(ReadFunction readFunction, void* obj);

	// Information if the connection is disconnected.
	typedef void (*DisconnectFunction)(void* obj);								//  Called on disconnect
	void setDisconnectCallbackFunction(DisconnectFunction discFunction, void* obj);

	
private:
	bool m_longStringWarningPrinted;
	std::string m_rxString;						// fuer readString()
	bool isClientConnected_unlocked();
	std::list<unsigned char> m_rxBuffer;		// Main input buffer
	void stopReadThread();
	void startServerThread();
	void stopServerThread();
	
    struct sockaddr_in m_serverAddr;				// Local address
	bool m_beVerbose;
	Mutex m_socketMutex;
	
	INT32 m_connectionSocket;	// Socket, wenn wir der Client sind (z.B. Verbindung zum Scanner)

	void readThreadFunction(bool& endThread, UINT16& waitTimeMs);
	SickThread<Tcp, &Tcp::readThreadFunction> m_readThread;
	INT32 readInputData();
	
	ReadFunction m_readFunction;		// Receive callback
	void* m_readFunctionObjPtr;			// Object of the Receive callback
	DisconnectFunction m_disconnectFunction;
	void* m_disconnectFunctionObjPtr;	// Object of the Disconect callback


/*	
	bool m_beVerbose;	///< Enable *very* verbose debug output
	asio::io_service m_io_service;
	asio::ip::tcp::socket* m_socket;

	boost::thread* m_readThreadPtr;
	bool m_readThreadIsRunning;
	bool m_readThreadShouldRun;
	boost::condition m_readThreadCondition;	///< Condition to wait for the spawned thread to be started
	boost::mutex m_readThreadMutex;			///< Mutex to wait for the spawned thread to be started
	void readThread();						///< Receive-thread for incoming scans.
	INT32 readInputData();

	/// Notification on disconnected socket: If non-NULL, the event
	/// monitor to which the m_disconnectedEventMask will be signalled
	/// if this connection was terminated.
	EventMonitor* m_disconnectedEventMonitor;

	/// Notification on disconnected socket: Event mask which will be
	/// signalled to the m_disconnectedEventMonitor if this connection
	/// was terminated.
	EventMonitor::Mask m_disconnectedEventMask;

	BoundedBuffer<UINT8> m_rxBuffer;		///< Main input buffer
	std::string m_rxString;					///< Eingangspuffer fuer Strings
	bool m_longStringWarningPrinted;
*/
};

#endif // TCP_HPP
