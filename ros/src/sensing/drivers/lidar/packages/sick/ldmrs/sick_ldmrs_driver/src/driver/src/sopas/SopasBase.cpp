//
// SopasBase.cpp
//
//  Created on: 18.07.2011
//      Author: sick
//

#include "SopasBase.hpp"


#include "../manager.hpp"
#include "../tools/errorhandler.hpp"
#include "../tools/toolbox.hpp"
#include "../tools/Mutex.hpp"

namespace devices
{
	
#define SOPASBASE_VERSION "1.0.0"

const std::string SopasBase::EVENTNAME_SUBSCRIBE_EVALCASES("LFErec");
const std::string SopasBase::EVENTNAME_SUBSCRIBE_SCANS("LMDscandata");
const std::string SopasBase::METHODNAME_LOGIN("SetAccessMode");
const std::string SopasBase::METHODNAME_LOGOUT("Run");
const std::string SopasBase::METHODNAME_SET_SCANCONFIG("mLMPsetscancfg");
const std::string SopasBase::METHODNAME_START_MEASURE("LMCstartmeas");
const std::string SopasBase::METHODNAME_STOP_MEASURE("LMCstopmeas");
const std::string SopasBase::VARIABLENAME_SCANCONFIG("LMPscancfg");
const std::string SopasBase::VARIABLENAME_DATAOUTPUTRANGE("LMPoutputRange");
const std::string SopasBase::VARIABLENAME_SCANDATACONFIG("LMDscandatacfg");
const std::string SopasBase::VARIABLENAME_DEVICEIDENT("DeviceIdent");

// sopas commands
const std::string SopasBase::COMMAND_Read_Variable_ByIndex("RI");
const std::string SopasBase::COMMAND_Write_Variable_ByIndex("WI");
const std::string SopasBase::COMMAND_Invoke_Method_ByIndex("MI");
const std::string SopasBase::COMMAND_Method_Result_ByIndex("AI");
const std::string SopasBase::COMMAND_Register_Event_ByIndex("EI");
const std::string SopasBase::COMMAND_Send_Event_ByIndex("SI"); // receive data event

const std::string SopasBase::COMMAND_Read_Variable_Answer("RA");
const std::string SopasBase::COMMAND_Write_Variable_Answer("WA");
const std::string SopasBase::COMMAND_Invoke_Method_Answer("MA");
const std::string SopasBase::COMMAND_Method_Result_Answer("AA");
const std::string SopasBase::COMMAND_Register_Event_Answer("EA");
const std::string SopasBase::COMMAND_Event_Acknowledge("SA");

const std::string SopasBase::COMMAND_Read_Variable_ByName("RN");
const std::string SopasBase::COMMAND_Write_Variable_ByName("WN");
const std::string SopasBase::COMMAND_Invoke_Method_ByName("MN");
const std::string SopasBase::COMMAND_Method_Result_ByName("AN");
const std::string SopasBase::COMMAND_Register_Event_ByName("EN");
const std::string SopasBase::COMMAND_Send_Event_ByName("SN"); // receive data event

const UINT16 SopasBase::INDEX_DEVICE_IDENT = 0;



SopasBase::SopasBase() :
	m_state(CONSTRUCTED)
{
	m_beVerbose = false;
	
	m_protocol = CoLa_A;	// Default protocol is CoLa-A
}



SopasBase::~SopasBase()
{
	printInfoMessage("Sopas device destructor: Running.", m_beVerbose);

	// Derived device must be stopped in it's constructor. No virtual functions may be called here!
//	assert(isRunning() == false);

	printInfoMessage("Sopas device destructor: Stopped, now disconnecting.", m_beVerbose);

	// Disconnect and shut down receive thread.
	if (isConnected() == true)
	{
		// Change from CONNECTED to CONSTRUCTED
		disconnect();
	}

//	printInfoMessage("Sopas device destructor: Disconnected, now deleting socket.", m_beVerbose);

	printInfoMessage("Sopas device destructor: Done, device is deleted.", m_beVerbose);
}



//
// Initialisation from Scanner class:
// Parameter setup only. Afterwards, call connect() to connect to the scanner.
//
bool SopasBase::init(SopasProtocol protocol,
						std::string ipAddress,
						UINT16 portNumber,
						bool weWantScanData,
						bool weWantFieldData,
						bool readOnlyMode,
						Tcp::DisconnectFunction disconnectFunction,
						void* obj)
{
	m_protocol = protocol;
	m_ipAddress = ipAddress;
	m_portNumber = portNumber;
	m_weWantScanData = weWantScanData;
	m_weWantFieldData = weWantFieldData;
	setReadOnlyMode(readOnlyMode);

	m_tcp.setDisconnectCallbackFunction(disconnectFunction, obj);

	return true;
}


//
// Verbinde mit dem unter init() eingestellten Geraet, und pruefe die Verbindung
// durch einen DeviceIdent-Aufruf.
//
// true = Erfolgreich.
//
bool SopasBase::connect()
{
	printInfoMessage("SopasBase::connect: Called.", m_beVerbose);
	
	assert (m_state == CONSTRUCTED); // must not be opened or running already

	// Initialise buffer variables
	m_numberOfBytesInReceiveBuffer = 0; // Buffer is empty
	m_numberOfBytesInResponseBuffer = 0; // Buffer is empty
	//	m_isLoggedIn = false;
	m_scannerName = "";
	m_scannerVersion = "";

	// Establish connection here
	// Set the data input callback for our TCP connection
	m_tcp.setReadCallbackFunction(&SopasBase::readCallbackFunctionS, this);	// , this, _1, _2));

	bool success = openTcpConnection();
	if (success == true)
	{
		// Check if scanner type matches
		m_state = CONNECTED;
		printInfoMessage("SopasBase::connect: Reading scanner infos", m_beVerbose);
		success = action_getScannerTypeAndVersion();

		if (success == true)
		{
			printInfoMessage("SopasBase::connect: Initialisation was successful.", m_beVerbose);
		}
		else
		{
			printError("SopasBase::connect: Failed to read scanner type and version.");
		}
	}

	if (success == false)
	{
		printWarning("SopasBase::connect: Initialisation failed!");
	}
	else
	{
		// Here, we could unsubscribe all events to have a defined default state.
	}

	printInfoMessage("SopasBase::connect: Done.", m_beVerbose);
	return success;
}



//
// True, if state is CONNECTED, that is:
// - A TCP-connection exists
// - Read thread is running
//
bool SopasBase::isConnected()
{
	return (m_state == CONNECTED);
}

//
// Disconnect from the scanner, and close the interface.
//
bool SopasBase::disconnect()
{
	closeTcpConnection();

	// Change back to CONSTRUCTED
	m_state = CONSTRUCTED;
	return true;
}



//
// (Un)set read only mode. Setting this mode, the sensor will ignore commands which will change any
// parameters on the device. The sensor itself will execute such commands.
//
void SopasBase::setReadOnlyMode(bool mode)
{
	m_readOnlyMode = mode;
}



bool SopasBase::isReadOnly()
{
	return m_readOnlyMode;
}



/**
 * Open TCP-connection to endpoint (usually IP-address and port)
 *
 * true = Connected, false = no connection
 */
bool SopasBase::openTcpConnection()
{
	printInfoMessage("SopasBase::openTcpConnection: Connecting TCP/IP connection to " + m_ipAddress + ":" + toString(m_portNumber) + " ...", m_beVerbose);

	bool success = m_tcp.open(m_ipAddress, m_portNumber, m_beVerbose);
	if (success == false)
	{
		printError("SopasBase::openTcpConnection: ERROR: Failed to establish TCP connection, aborting!");
		return false;
	}

	return true;
}



//
// Close TCP-connection and shut down read thread
//
void SopasBase::closeTcpConnection()
{
	if (m_tcp.isOpen())
	{
		m_tcp.close();
	}
}

//
// Static entry point.
//
void SopasBase::readCallbackFunctionS(void* obj, UINT8* buffer, UINT32& numOfBytes)
{
	((SopasBase*)obj)->readCallbackFunction(buffer, numOfBytes);
}


/**
 * Read callback. Diese Funktion wird aufgerufen, sobald Daten auf der Schnittstelle
 * hereingekommen sind.
 */
void SopasBase::readCallbackFunction(UINT8* buffer, UINT32& numOfBytes)
{
	bool beVerboseHere = false;
	printInfoMessage("SopasBase::readCallbackFunction(): Called with " + toString(numOfBytes) + " available bytes.", beVerboseHere);

	ScopedLock lock(&m_receiveDataMutex); // Mutex for access to the input buffer
	UINT32 remainingSpace = sizeof(m_receiveBuffer) - m_numberOfBytesInReceiveBuffer;
	UINT32 bytesToBeTransferred = numOfBytes;
	if (remainingSpace < numOfBytes)
	{
		bytesToBeTransferred = remainingSpace;
		printWarning("SopasBase::readCallbackFunction(): Input buffer space is to small, transferring only " +
										::toString(bytesToBeTransferred) + " of " + ::toString(numOfBytes) + " bytes.");
	}
	else
	{
		printInfoMessage("SopasBase::readCallbackFunction(): Transferring " + ::toString(bytesToBeTransferred) +
										 " bytes from TCP to input buffer.", beVerboseHere);
	}

	if (bytesToBeTransferred > 0)
	{
		// Data can be transferred into our input buffer
		memcpy(&(m_receiveBuffer[m_numberOfBytesInReceiveBuffer]), buffer, bytesToBeTransferred);
		m_numberOfBytesInReceiveBuffer += bytesToBeTransferred;

		UINT32 size = 0;

		while (1)
		{
			// Now work on the input buffer until all received datasets are processed
			SopasEventMessage frame = findFrameInReceiveBuffer();

			size = frame.size();
			if (size == 0)
			{
				// Framesize = 0: There is no valid frame in the buffer. The buffer is either empty or the frame
				// is incomplete, so leave the loop
				printInfoMessage("SopasBase::readCallbackFunction(): No complete frame in input buffer, we are done.", beVerboseHere);

				// Leave the loop
				break;
			}
			else
			{
				// A frame was found in the buffer, so process it now.
				printInfoMessage("SopasBase::readCallbackFunction(): Processing a frame of length " + ::toString(frame.size()) + " bytes.", beVerboseHere);
				processFrame(frame);
			}
		}
	}
	else
	{
		// There was input data from the TCP interface, but our input buffer was unable to hold a single byte.
		// Either we have not read data from our buffer for a long time, or something has gone wrong. To re-sync,
		// we clear the input buffer here.
		m_numberOfBytesInReceiveBuffer = 0;
	}

	printInfoMessage("SopasBase::readCallbackFunction(): Leaving. Current input buffer fill level is " + 
									 ::toString(m_numberOfBytesInReceiveBuffer) + " bytes.", beVerboseHere);
}



//
// Look for 23-frame (STX/ETX) in receive buffer.
// Move frame to start of buffer
//
// Return: 0 : No (complete) frame found
//        >0 : Frame length
//
SopasEventMessage SopasBase::findFrameInReceiveBuffer()
{
	UINT32 frameLen = 0;
	UINT32 i;

	// Depends on protocol...
	if (m_protocol == CoLa_A)
	{
		//
		// COLA-A
		//
		// Must start with STX (0x02)
		if (m_receiveBuffer[0] != 0x02)
		{
			// Look for starting STX (0x02)
			for (i = 1; i < m_numberOfBytesInReceiveBuffer; i++)
			{
				if (m_receiveBuffer[i] == 0x02)
				{
					break;
				}
			}

			// Found beginning of frame?
			if (i >= m_numberOfBytesInReceiveBuffer)
			{
				// No start found, everything can be discarded
				m_numberOfBytesInReceiveBuffer = 0; // Invalidate buffer
				return SopasEventMessage(); // No frame found
			}

			// Move frame start to index 0
			UINT32 newLen = m_numberOfBytesInReceiveBuffer - i;
			memmove(&(m_receiveBuffer[0]), &(m_receiveBuffer[i]), newLen);
			m_numberOfBytesInReceiveBuffer = newLen;
		}

		// Look for ending ETX (0x03)
		for (i = 1; i < m_numberOfBytesInReceiveBuffer; i++)
		{
			if (m_receiveBuffer[i] == 0x03)
			{
				break;
			}
		}

		// Found end?
		if (i >= m_numberOfBytesInReceiveBuffer)
		{
			// No end marker found, so it's not a complete frame (yet)
			return SopasEventMessage(); // No frame found
		}

		// Calculate frame length in byte
		frameLen = i + 1;

		return SopasEventMessage(m_receiveBuffer, CoLa_A, frameLen);
	}
	else if (m_protocol == CoLa_B)
	{
		UINT32 magicWord;
		UINT32 payloadlength;

		if (m_numberOfBytesInReceiveBuffer < 4)
		{
			return SopasEventMessage();
		}
		UINT16 pos = 0;
		magicWord = colab::getIntegerFromBuffer<UINT32>(m_receiveBuffer, pos);
		if (magicWord != 0x02020202)
		{
			// Look for starting STX (0x02020202)
			for (i = 1; i <= m_numberOfBytesInReceiveBuffer - 4; i++)
			{
				pos = i; // this is needed, as the position value is updated by getIntegerFromBuffer
				magicWord = colab::getIntegerFromBuffer<UINT32>(m_receiveBuffer, pos);
				if (magicWord == 0x02020202)
				{
					// found magic word
					break;
				}
			}

			// Found beginning of frame?
			if (i > m_numberOfBytesInReceiveBuffer - 4)
			{
				// No start found, everything can be discarded
				m_numberOfBytesInReceiveBuffer = 0; // Invalidate buffer
				return SopasEventMessage(); // No frame found
			}
			else
			{
				// Move frame start to index
				UINT32 bytesToMove = m_numberOfBytesInReceiveBuffer - i;
				memmove(&(m_receiveBuffer[0]), &(m_receiveBuffer[i]), bytesToMove); // payload+magic+length+s+checksum
				m_numberOfBytesInReceiveBuffer = bytesToMove;
			}
		}

		// Pruefe Laenge des Pufferinhalts
		if (m_numberOfBytesInReceiveBuffer < 9)
		{
			// Es sind nicht genug Daten fuer einen Frame
			printInfoMessage("SopasBase::findFrameInReceiveBuffer: Frame cannot be decoded yet, only " +
								 ::toString(m_numberOfBytesInReceiveBuffer) + " bytes in the buffer.", m_beVerbose);
			return SopasEventMessage();
		}

		// Read length of payload
		pos = 4;
		payloadlength = colab::getIntegerFromBuffer<UINT32>(m_receiveBuffer, pos);
		printInfoMessage("SopasBase::findFrameInReceiveBuffer: Decoded payload length is " + ::toString(payloadlength) + " bytes.", m_beVerbose);
		
		// Ist die Datenlaenge plausibel und wuede in den Puffer passen?
		if (payloadlength > (sizeof(m_receiveBuffer) - 9))
		{
			// magic word + length + checksum = 9
			printWarning("SopasBase::findFrameInReceiveBuffer: Frame too big for receive buffer. Frame discarded with length:"
											+ ::toString(payloadlength) + ".");
			m_numberOfBytesInReceiveBuffer = 0;
			return SopasEventMessage();
		}
		if ((payloadlength + 9) > m_numberOfBytesInReceiveBuffer)
		{
			// magic word + length + s + checksum = 10
			printInfoMessage("SopasBase::findFrameInReceiveBuffer: Frame not complete yet. Waiting for the rest of it (" +
								 ::toString(payloadlength + 9 - m_numberOfBytesInReceiveBuffer) + " bytes missing).", m_beVerbose);
			return SopasEventMessage(); // frame not complete
		}

		// Calculate the total frame length in bytes: Len = Frame (9 bytes) + Payload
		frameLen = payloadlength + 9;

		//
		// test checksum of payload
		//
		UINT8 temp = 0;
		UINT8 temp_xor = 0;
		UINT8 checkSum;

		// Read original checksum
		pos = frameLen - 1;
		checkSum = colab::getIntegerFromBuffer<UINT8>(m_receiveBuffer, pos);

		// Erzeuge die Pruefsumme zum Vergleich
		for (UINT16 i = 8; i < (frameLen - 1); i++)
		{
			pos = i;
			temp = colab::getIntegerFromBuffer<UINT8>(m_receiveBuffer, pos);
			temp_xor = temp_xor ^ temp;
		}

		// Vergleiche die Pruefsummen
		if (temp_xor != checkSum)
		{
			printWarning("SopasBase::findFrameInReceiveBuffer: Wrong checksum, Frame discarded.");
			m_numberOfBytesInReceiveBuffer = 0;
			return SopasEventMessage();
		}

		return SopasEventMessage(m_receiveBuffer, CoLa_B, frameLen);
	}

	// Return empty frame
	return SopasEventMessage();
}



/**
 * Send contents of buffer to scanner using according framing.
 *
 * Send buffer is limited to 1024 byte!
 */
void SopasBase::sendCommandBuffer(UINT8* buffer, UINT16 len)
{
	UINT8 sendBuffer[1024];

	assert (len < 1000);
	assert (m_tcp.isOpen() == true);

	// Frame the string
	if (m_protocol == CoLa_A)
	{
		colaa::addFrameToBuffer(sendBuffer, buffer, &len);
	}
	else if (m_protocol == CoLa_B)
	{
		colab::addFrameToBuffer(sendBuffer, buffer, &len);
	}
	
	// Debug: Print buffer
//	traceBuffer("Cmd buffer contents:", sendBuffer, len);
	
	// Send command (blocking)
	m_tcp.write(sendBuffer, len);
}



/**
 * Take answer from read thread and decode it.
 * Waits for a certain answer by name. Event data (scans) are filtered and processed
 * by read thread.
 *
 * By Name: name = "<Name>"
 * timeout: Number of cycles to check for an answer (approx. 1ms per cycle)
 */
bool SopasBase::receiveAnswer(SopasCommand cmd, std::string name, UINT32 timeout, SopasAnswer*& answer)
{
	switch (m_protocol)
	{
	case CoLa_A:
		printInfoMessage("SopasBase::receiveAnswer: calling receiveAnswer_CoLa_A.", m_beVerbose);
		return receiveAnswer_CoLa_A(cmd, name, timeout, answer);
	case CoLa_B:
		printInfoMessage("SopasBase::receiveAnswer: calling receiveAnswer_CoLa_B.", m_beVerbose);
		return receiveAnswer_CoLa_B(cmd, name, timeout, answer);
	default:
		printWarning("SopasBase::receiveAnswer: Wrong protocol (is either not CoLa-A or CoLa-B).");
	}

	return false;
}



/**
 * Take answer from read thread and decode it.
 * Waits for a certain answer by index. Event data (scans) are filtered and processed
 * by read thread.
 *
 * By Name: index = "<Index>"
 * timeout: Number of cycles to check for an answer (approx. 1ms per cycle)
 */
bool SopasBase::receiveAnswer(SopasCommand cmd, UINT16 index, UINT32 timeout, SopasAnswer*& answer)
{
	switch (m_protocol)
	{
	case CoLa_A:
		printInfoMessage("SopasBase::receiveAnswer: Calling receiveAnswer_CoLa_A.", m_beVerbose);
		return receiveAnswer_CoLa_A(cmd, index, timeout, answer);
	case CoLa_B:
		printInfoMessage("SopasBase::receiveAnswer: Calling receiveAnswer_CoLa_B.", m_beVerbose);
		return receiveAnswer_CoLa_B(cmd, index, timeout, answer);
	default:
		printWarning("SopasBase::receiveAnswer: Wrong protocol (is either not CoLa-A or CoLa-B).");
	}

	return false;
}


//
//
//
bool SopasBase::receiveAnswer_CoLa_A(SopasCommand cmd, UINT16 index, UINT32 timeout, SopasAnswer*& answer)
{
	printInfoMessage("SopasBase::receiveAnswer_CoLa_A: entering function.", m_beVerbose);

	SopasCommand receivedCommand;
	std::string rxData;
	if (timeout == 0)
	{
		timeout = 1; // Check at least once
	}

	// Check until number of cycles reaches timeout
	for (UINT32 i = 0; i < timeout; i++)
	{
		// Secure against read thread
		{
			ScopedLock lock(&m_receiveDataMutex);
			if (m_numberOfBytesInResponseBuffer > 0)
			{
				printInfoMessage("SopasBase::receiveAnswer_CoLa_A: Response received (len= " +
 										::toString(m_numberOfBytesInResponseBuffer) +
 										" bytes).", m_beVerbose);
				rxData = colaa::convertRxBufferToString(m_responseBuffer, m_numberOfBytesInResponseBuffer);
				m_numberOfBytesInResponseBuffer = 0; // Clear buffer
			}
		}

		if (rxData.length() > 0)
		{
			// Decode data
			receivedCommand = colaA_decodeCommand(&rxData);

			if (cmd == receivedCommand)
			{

				// Set variable answer
				if (receivedCommand == WA)
				{
					// Answer contains variable index only and informs about success.
					return true;
				}

				UINT16 receivedIndex = colaa::decodeUINT16(&rxData); // by name

				if (receivedIndex == index)
				{
					// Answer to Read Variable or invoke method?
					if (cmd == RA || cmd == AN)
					{
						printInfoMessage("SopasBase::receiveAnswer_CoLa_A: *** receive answer with data in return *** ", m_beVerbose);

						answer = new SopasAnswer((BYTE*)rxData.c_str(), rxData.length());
						m_numberOfBytesInResponseBuffer = 0; // Clear buffer
						return true;
					}
					else if (receivedCommand == EA)
					{
						// Antwort auf Event-Abonnement setzen/loeschen.
						printInfoMessage("SopasBase::receiveAnswer_CoLa_A: Event (by index) successfully (un)registered: " + 
										 ::toString(receivedIndex) + ".", m_beVerbose);

						m_numberOfBytesInResponseBuffer = 0;
						return true;
					}
				}
				else
				{
					printInfoMessage("SopasBase::receiveAnswer_CoLa_A: This is not the answer we are waiting for. ", m_beVerbose);
					m_numberOfBytesInResponseBuffer = 0; // Clear buffer
				}
			}
			// Error answer
			else if (receivedCommand == FA)
			{
				printInfoMessage("SopasBase::Error answer received: FA " + rxData + ".", m_beVerbose);
				answer = new SopasAnswer((BYTE*)rxData.c_str(), rxData.length());
				return false;
			}
			else
			{
				printInfoMessage("SopasBase::receiveAnswer_CoLa_A: This is not the answer we are waiting for. ", m_beVerbose);
				m_numberOfBytesInResponseBuffer = 0; // Clear buffer
				rxData.clear();
			}
		}
		else
		{
			// No data in response buffer. Sleep some time and check again
			usleep(1000);
		}
	}

	// Not successful, timeout.
	return false;
}



bool SopasBase::receiveAnswer_CoLa_A(SopasCommand cmd, std::string name, UINT32 timeout, SopasAnswer*& answer)
{
	printInfoMessage("SopasBase::receiveAnswer_CoLa_A: entering function.", m_beVerbose);

	SopasCommand receivedCommand;
	std::string rxData;
	if (timeout == 0)
	{
		timeout = 1; // Check at least once
	}

	// Check until number of cycles reaches timeout
	for (UINT32 i = 0; i < timeout; i++)
	{
		// Secure against read thread
		{
			ScopedLock lock(&m_receiveDataMutex);
			if (m_numberOfBytesInResponseBuffer > 0)
			{
				printInfoMessage("SopasBase::receiveAnswer_CoLa_A: Response received (len= " +
								 ::toString(m_numberOfBytesInResponseBuffer) + " bytes).", m_beVerbose);
				rxData = colaa::convertRxBufferToString(m_responseBuffer, m_numberOfBytesInResponseBuffer);
				m_numberOfBytesInResponseBuffer = 0; // Clear buffer
			}
		}

		if (rxData.length() > 0)
		{
			// Decode data
			receivedCommand = colaA_decodeCommand(&rxData);

			if (cmd == receivedCommand)
			{

				// Set variable answer
				if (receivedCommand == WA)
				{
					// Answer contains variable index only and informs about success.
					return true;
				}

				std::string receivedName = colaa::decodeString(&rxData); // by name

				if (receivedName == name)
				{
					// Answer to Read Variable or invoke method?
					if (cmd == RA || cmd == AN)
					{
						printInfoMessage("SopasBase::receiveAnswer_CoLa_A: *** receive answer with data in return *** ", m_beVerbose);

						answer = new SopasAnswer((BYTE*)rxData.c_str(), rxData.length());
						m_numberOfBytesInResponseBuffer = 0; // Clear buffer
						return true;
					}
					else if (receivedCommand == EA)
					{
						// Antwort auf Event-Abonnement setzen/loeschen.

						printInfoMessage("SopasBase::  Event successfully (un)registered: " + receivedName + ".", m_beVerbose);
						m_numberOfBytesInResponseBuffer = 0;
						return true;
					}
				}
				else
				{
					printInfoMessage("SopasBase::receiveAnswer_CoLa_A: This is not the answer we are waiting for.", m_beVerbose);
					m_numberOfBytesInResponseBuffer = 0; // Clear buffer
				}
			}
			// Error answer
			else if (receivedCommand == FA)
			{
				printInfoMessage("SopasBase::  Error answer received: FA " + rxData + ".", true);
				answer = new SopasAnswer((BYTE*)rxData.c_str(), rxData.length());
				return false;
			}
			else
			{
				printInfoMessage("SopasBase::receiveAnswer_CoLa_A: This is not the answer we are waiting for.", m_beVerbose);
				m_numberOfBytesInResponseBuffer = 0; // Clear buffer
				rxData.clear();
			}
		}
		else
		{
			// No data in response buffer. Sleep some time and check again
			usleep(1000);
		}
	}

	// Not successful, timeout.
	return false;
}



bool SopasBase::receiveAnswer_CoLa_B(SopasCommand cmd, std::string name, UINT32 timeout, SopasAnswer*& answer)
{
	printInfoMessage("SopasBase::receiveAnswer_CoLa_B: Entering function.", m_beVerbose);

	SopasCommand receivedCommand;
	UINT16 nextData;
	std::string receivedName;

	if (timeout == 0)
	{
		timeout = 1; // Check at least once
	}

	// Check until number of cycles reaches timeout
	for (UINT32 i = 0; i < timeout; i++)
	{
		if (m_numberOfBytesInResponseBuffer > 0)
		{
//			if (m_beVerbose)
//			{
//				printInfoMessage("SopasBase::receiveAnswer_CoLa_B: There is something in receive buffer." << std::endl;
//			}

			// protect response buffer
			ScopedLock lock(&m_receiveDataMutex);

			// there are only complete frames in response buffer
			SopasEventMessage frame (m_responseBuffer, CoLa_B, m_numberOfBytesInResponseBuffer);

			// Parse Sopas Kommando
			//			receivedCommand = stringToSopasCommand(colab::getCommandStringFromBuffer(m_responseBuffer));
			receivedCommand = stringToSopasCommand(frame.getCommandString());

			printInfoMessage("SopasBase::receiveAnswer_CoLa_B: receivedCommand= " + frame.getCommandString() + ".", m_beVerbose);

			if (receivedCommand == FA)
			{
				// Fehlermeldung vom Sensor
				nextData = 0;
				UINT16 errorCode = colab::getIntegerFromBuffer<UINT16>(&(m_responseBuffer[11]), nextData);
				printWarning("SopasBase::receiveAnswer_CoLa_B: Error from sensor! Code=" + toString(errorCode) +
								", meaning: " + convertSopasErrorCodeToText(errorCode) + ".");
				m_numberOfBytesInResponseBuffer = 0;
				return false;
			}

			if (cmd != receivedCommand)
			{
				// Nicht die gesuchte Antwort
				// da der sensor über sopas immer nur eine anfrage zur zeit bedienen kann, koennen wir die
				// Nachricht wegwerfen
				printInfoMessage("SopasBase::receiveAnswer_CoLa_B: This is not the answer (" + sopasCommandToString(receivedCommand) + ") are waiting for ("
						+ sopasCommandToString(cmd) + ").", m_beVerbose);
				m_numberOfBytesInResponseBuffer = 0;
				return false;
			}

			//			UINT16 nameLength = colab::getIntegerFromBuffer<UINT16>(m_responseBuffer, nextData);
			//			receivedName = colab::getStringFromBuffer(m_responseBuffer, nextData);

			receivedName = colab::getIdentifierFromBuffer(m_responseBuffer, nextData, sizeof(m_responseBuffer));

			printInfoMessage("SopasBase::receiveAnswer_CoLa_B: receicedName= " + receivedName + ".", m_beVerbose);

			if (name != receivedName)
			{
				// Nicht die gesuchte Antwort
				printInfoMessage("SopasBase::receiveAnswer_CoLa_B: This is not the answer we are waiting for. (" + frame.getCommandString() +
									 ", " + name + ").", m_beVerbose);

				m_numberOfBytesInResponseBuffer = 0;
				return false;
			}

			if (receivedCommand == WA)
			{
				// Variable schreiben erfolgreich
				printInfoMessage("SopasBase::receiveAnswer_CoLa_B: Variable successfully set.", m_beVerbose);
				m_numberOfBytesInResponseBuffer = 0;
				return true;
			}

			else if (receivedCommand == EA)
			{
				printInfoMessage("SopasBase::receiveAnswer_CoLa_B: Answer to (un)subscribe event " + receivedName + ".", m_beVerbose);
				m_numberOfBytesInResponseBuffer = 0;
				return true;
			}
			// Antwort auf Methode ByName
			else if (receivedCommand == AN )
			{
				printInfoMessage("SopasBase::receiveAnswer_CoLa_B: Answer to method call " + receivedName + ".", m_beVerbose);
				answer = new SopasAnswer(&m_responseBuffer[nextData], m_numberOfBytesInResponseBuffer - nextData);
				m_numberOfBytesInResponseBuffer = 0;
				return true;
			}
			else if (receivedCommand == RA)
			{
				// Antwort auf das Lesen einer Variablen
				answer = new SopasAnswer(&m_responseBuffer[nextData], m_numberOfBytesInResponseBuffer - nextData);
				m_numberOfBytesInResponseBuffer = 0;
				return true;
			}

			else if (receivedCommand == CMD_UNKNOWN)
			{
				printInfoMessage("SopasBase::receiveAnswer_CoLa_B: Unkown Sopas command.", true);
				m_numberOfBytesInResponseBuffer = 0;
				return false;
			}
			else
			{
				// unbekannter oder unbehandelter Befehl
				printInfoMessage("SopasBase::receiveAnswer_CoLa_B: Untreated Sopas command.", m_beVerbose);
				m_numberOfBytesInResponseBuffer = 0;
				return false;
			}

		}
		else
		{
			// No data in response buffer. Sleep some time and check again
			usleep(1000);
		}
	}

	// Not successful, timeout.
	printInfoMessage("SopasBase::receiveAnswer_CoLa_B: Leaving with timeout.", m_beVerbose);

	return false;
}

//
// Convert a Sopas error code to readable text.
//
std::string SopasBase::convertSopasErrorCodeToText(UINT16 errorCode)
{
	switch (errorCode)
	{
		case 0:
			return "Sopas_Ok";
		case 1:
			return "Sopas_Error_METHODIN_ACCESSDENIED";
		case 2:
			return "Sopas_Error_METHODIN_UNKNOWNINDEX";
		case 3:
			return "Sopas_Error_VARIABLE_UNKNOWNINDEX";
		case 4:
			return "Sopas_Error_LOCALCONDITIONFAILED";
		case 5:
			return "Sopas_Error_INVALID_DATA";
		case 6:
			return "Sopas_Error_UNKNOWN_ERROR";
		case 7:
			return "Sopas_Error_BUFFER_OVERFLOW";
		case 8:
			return "Sopas_Error_BUFFER_UNDERFLOW";
		case 9:
			return "Sopas_Error_ERROR_UNKNOWN_TYPE";
		case 10:
			return "Sopas_Error_VARIABLE_WRITE_ACCESSDENIED";
		case 11:
			return "Sopas_Error_UNKNOWN_CMD_FOR_NAMESERVER";
		case 12:
			return "Sopas_Error_UNKNOWN_COLA_COMMAND";
		case 13:
			return "Sopas_Error_METHODIN_SERVER_BUSY";
		case 14:
			return "Sopas_Error_FLEX_OUT_OF_BOUNDS";
		case 15:
			return "Sopas_Error_EVENTREG_UNKNOWNINDEX";
		case 16:
			return "Sopas_Error_COLA_A_VALUE_OVERFLOW";
		case 17:
			return "Sopas_Error_COLA_A_INVALID_CHARACTER";
		case 18:
			return "Sopas_Error_OSAI_NO_MESSAGE";
		case 19:
			return "Sopas_Error_OSAI_NO_ANSWER_MESSAGE";
		case 20:
			return "Sopas_Error_INTERNAL";
		case 21:
			return "Sopas_Error_HubAddressCorrupted";
		case 22:
			return "Sopas_Error_HubAddressDecoding";
		case 23:
			return "Sopas_Error_HubAddressAddressExceeded";
		case 24:
			return "Sopas_Error_HubAddressBlankExpected";
		case 0x19:
			return "Sopas_Error_AsyncMethodsAreSuppressed";
		case 0x20:
			return "Sopas_Error_ComplexArraysNotSupported";
		default:
			return "(unknown_Sopas_error_code)";
	}
	
	return "(unknown_Sopas_error_code)";
}

//
// Decode an incoming CoLa-B answer that was addressed by index.
//
bool SopasBase::receiveAnswer_CoLa_B(SopasCommand cmd, UINT16 index, UINT32 timeout, SopasAnswer*& answer)
{
	bool beVerboseHere = m_beVerbose;
//	beVerboseHere = true;
	
	printInfoMessage("SopasBase::receiveAnswer_CoLa_B_idx: Entering function.", beVerboseHere);

	SopasCommand receivedCommand;
	UINT16 nextData;

	if (timeout == 0)
	{
		timeout = 1; // Check at least once
	}

	// Check until number of cycles reaches timeout
	for (UINT32 i = 0; i < timeout; i++)
	{
		if (m_numberOfBytesInResponseBuffer > 0)
		{
			printInfoMessage("SopasBase::receiveAnswer_CoLa_B_idx: There is something in receive buffer.", beVerboseHere);

			// Protect response buffer
			ScopedLock lock(&m_receiveDataMutex);

			// There are only complete frames in response buffer, so we can just use it here.
			// Debug: Print buffer
//			traceBuffer("Response in response buffer:", m_responseBuffer, m_numberOfBytesInResponseBuffer);
			SopasEventMessage frame (m_responseBuffer, CoLa_B, m_numberOfBytesInResponseBuffer);

			// Parse Sopas Kommando
			receivedCommand = stringToSopasCommand(frame.getCommandString());

			printInfoMessage("SopasBase::receiveAnswer_CoLa_B_idx: ReceivedCommand= " + frame.getCommandString() + ".", beVerboseHere);

			nextData = 11; // without 0x02020202 + length(4 byte) + "sXX"
			UINT16 receivedIndex = colab::getIntegerFromBuffer<UINT16>(m_responseBuffer, nextData);
//			printInfoMessage("SopasBase::receiveAnswer_CoLa_B_idx: read command " + frame.getCommandString() + ".", beVerboseHere);
			if (receivedCommand == FA)
			{
				// Error message from the sensor. The next 2 bytes (in the receiveIndex) are not
				// the index but the error code.
				printWarning("SopasBase::receiveAnswer_CoLa_B_idx: Error from sensor! Code=" + toString(receivedIndex) +
								", meaning: " + convertSopasErrorCodeToText(receivedIndex) + ".");
				m_numberOfBytesInResponseBuffer = 0;
				return false;
			}

			else if (index != receivedIndex)
			{
				// Nicht die gesuchte Antwort
				// da der sensor über sopas immer nur eine anfrage zur zeit bedienen kann, koennen wir die
				// Nachricht wegwerfen
				printInfoMessage("SopasBase::receiveAnswer_CoLa_B_idx: This is not the index we are waiting for (expected="
						+ toString(index) + ", received=" + toString(receivedIndex) + ")." , beVerboseHere);
				m_numberOfBytesInResponseBuffer = 0;
				return false;
			}

			else if (cmd != receivedCommand)
			{
				// Not the desired answer.
				// As the sensor can only handle one command at a time on the SOPAS interface, we can discard the message.
				printInfoMessage("SopasBase::receiveAnswer_CoLa_B_idx: This is not the answer (" + sopasCommandToString(receivedCommand) +
						") we are waiting for (" + sopasCommandToString(cmd) + "). ", beVerboseHere);
				m_numberOfBytesInResponseBuffer = 0;
				return false;
			}
			
			else if (receivedCommand == WA)
			{
				// Variable schreiben erfolgreich
				printInfoMessage("SopasBase::receiveAnswer_CoLa_B_idx: Variable successfully set.", beVerboseHere);
				m_numberOfBytesInResponseBuffer = 0;
				return true;
			}

			else if (receivedCommand == EA)
			{
				// Antwort auf Event-Abonnement setzen/loeschen.
				printInfoMessage("SopasBase::receiveAnswer_CoLa_B_idx: Event successfully (un)registered: " + ::toString(receivedIndex) + ".", beVerboseHere);
				m_numberOfBytesInResponseBuffer = 0;
				return true;
			}

			else if (receivedCommand == AI )
			{
				// Antwort auf entfernten Methodenaufruf (by name oder by index).
				printInfoMessage("SopasBase::receiveAnswer_CoLa_B_idx: Answer to method call with index " + ::toString(receivedIndex) + ".", beVerboseHere);
				answer = new SopasAnswer(&m_responseBuffer[nextData], m_numberOfBytesInResponseBuffer - nextData);
				m_numberOfBytesInResponseBuffer = 0;
				return true;
			}

			else if (receivedCommand == RA)
			{
				// Antwort auf das Lesen einer Variablen
				printInfoMessage("SopasBase::receiveAnswer_CoLa_B_idx: Answer to read variable with index " + ::toString(receivedIndex) + ".", beVerboseHere);

				answer = new SopasAnswer(&m_responseBuffer[nextData], m_numberOfBytesInResponseBuffer - nextData);
				m_numberOfBytesInResponseBuffer = 0;
				return true;
			}

			else if (receivedCommand == CMD_UNKNOWN)
			{
				printWarning("SopasBase::receiveAnswer_CoLa_B_idx: Unknown Sopas command.");
				m_numberOfBytesInResponseBuffer = 0;
				return false;
			}
			else
			{
				// unbekannter oder unbehandelter Befehl
				printInfoMessage("SopasBase::receiveAnswer_CoLa_B_idx: Untreated Sopas command.", beVerboseHere);
				m_numberOfBytesInResponseBuffer = 0;
				return false;
			}
		}
		else
		{
			// No data in response buffer. Sleep some time and check again
			usleep(1000);
		}
	}

	// Not successful, timeout.
	printInfoMessage("SopasBase::receiveAnswer_CoLa_B_idx: Leaving with timeout.", beVerboseHere);

	// Not successful, timeout.
	return false;
}



/**
 * Reads one frame from receive buffer and decodes it.
 * Switches directly to the decoder of the protocol.
 *
 */
void SopasBase::processFrame(SopasEventMessage& frame)
{

	if (m_protocol == CoLa_A)
	{
		printInfoMessage("SopasBase::processFrame: Calling processFrame_CoLa_A() with " + ::toString(frame.size()) + " bytes.", m_beVerbose);
		processFrame_CoLa_A(frame);
	}
	else if (m_protocol == CoLa_B)
	{
		printInfoMessage("SopasBase::processFrame: Calling processFrame_CoLa_B() with " + ::toString(frame.size()) + " bytes.", m_beVerbose);
		processFrame_CoLa_B(frame);
	}
}



//
// Reads one frame from receive buffer and decodes it.
//
// CoLa-A protocol only.
//
void SopasBase::processFrame_CoLa_A(SopasEventMessage& frame)
{
	std::string command;
	command = m_receiveBuffer[2];
	command += m_receiveBuffer[3];
	assert (command.length() == 2);

	//
	// Process asynchronous event data directly.
	// Other commands are copied to separate buffer
	//
	if (frame.getMessageType() == MSG_SEND_EVENT)
	{
		if (frame.size() > 8)
		{
			// "SN" ist ein abonniertes Event, z.B. Scans oder Schutzfelder
			std::string eventName;
			UINT32 i = 5;// Pointer to the fist Letter of the event Name (without the first 5 bytes: "STX's''S''N'' '")
			while (m_receiveBuffer[i] != ' ' && i < frame.size() )
			{
				// go to the ' ' after the event Name
				i++;
			}
			eventName = std::string((char*) m_receiveBuffer, 5, i - 5); // get the event Name

			if (m_decoderFunctionMapByName[eventName] != NULL)
			{
				// Not empty
				m_decoderFunctionMapByName[eventName](frame);// call the callback of the Event
			}
			else
			{
				// Das Event ist unbekannt
				printWarning("SopasBase::Received an unknown event! Event name is: " + eventName + ".");
			}
//
//			if (eventName == scanString) // Scan data (hopefully, event name not parsed yet...)
//			{
//				if (!m_decoderFunctionMapByName[eventName].empty())
//				{
//					m_scanDecoderFunction(frame);
//				}
//				else
//				{
//					printInfoMessage("SopasBase::no scan decoder registered - discarding data." << std::endl;
//				}
//			}
//			else if (eventName == fieldString) // eval case result data
//			{
//
//				if (!m_evalCaseDecoderFunction.empty())
//				{
//					m_evalCaseDecoderFunction(frame);
//				}
//				else
//				{
//					printInfoMessage("SopasBase::no eval case decoder registered - discarding data." << std::endl;
//				}
//			}
//			else
//			{ // Das Event ist unbekannt
//				printWarning("SopasBase::Received an unknown event! Short name is: " << eventName << "." << std::endl;
//			}
		}
		else
		{
			// Das Kommando ist insgesamt zu kurz zum dekodieren
			printWarning("SopasBase::Received a very short (and unknown) event! Frame length is " + ::toString(frame.size()) +
											" bytes.");
		}
	}
	else
	{
		// Copy frame to response buffer. Old contents are destroyed
		// boost::mutex::scoped_lock lock(m_receiveDataMutex);
		copyFrameToResposeBuffer(frame.size());
	}

	// Remove frame from receive buffer
	removeFrameFromReceiveBuffer(frame.size());
}



//
// Reads and decodes a frame from the receiveBuffer. Frame format is CoLa-B.
//
// Note that this function typically runs in TCP callback context, not the device thread.
//
void SopasBase::processFrame_CoLa_B(SopasEventMessage& frame)
{
	// Lese das "Kommando" aus
	std::string command = colab::getCommandStringFromBuffer(m_receiveBuffer);

	printInfoMessage("SopasBase::processFrame_CoLa_B: Command is " + command + ".", m_beVerbose);

	//
	// Process asynchronous event data directly.
	// Other commands are copied to separate buffer
	//
	bool frameWasProcessed = false;
	if (frame.getMessageType() == MSG_SEND_EVENT) // LDMRS: SI, LMS100: SN
	{
		if (frame.size() > 12)
		{
			// "SN" or "SI" is a registered event, e.g. Scans or Eval case results.
			//			printInfoMessage("SopasBase::SN empfangen, Laenge = " << frameLength << "Bytes ." << std::endl;
			std::string eventName;
			UINT32 eventIndex = 0;
			UINT16 pos = 0;

			switch (frame.getEncodingType())
			{
			case ByIndex: // by index
				pos = 11;
				eventIndex = colab::getIntegerFromBuffer<UINT16>(m_receiveBuffer, pos);
				eventName = m_indexToNameMap[eventIndex];

				switch (eventIndex)
				{
					case 0x11: 	// SopasEventByIndex_LDMRS::index_event_ScanData:
						eventName = EVENTNAME_SUBSCRIBE_SCANS;
						scanDataDecoder(frame);
						frameWasProcessed = true;
						break;
					case 0x29: 	// SopasEventByIndex_LDMRS::index_event_aEvalCaseResult:
						eventName = EVENTNAME_SUBSCRIBE_EVALCASES;
						evalCaseResultDecoder(frame);
						frameWasProcessed = true;
						break;
					default:
						;
				}
				break;
				
			default: // by name
				printWarning("SopasBase::processFrame_CoLa_B: Decoding of events by name is not implemented yet.");
				eventName = frame.getVariableName();
				break;
			}

			if (frameWasProcessed == false)
			{
				// The incoming event was not handeled
				printWarning("SopasBase::processFrame_CoLa_B: Don't know how to process the incoming event with the short name <" +
								eventName + "> and the index " + ::toString(eventIndex) + ", ignoring it!");
			}
		}
		else
		{
			// The frame was too short to be decoded
			printWarning("SopasBase::processFrame_CoLa_B: Received a very short (and unknown) event! Frame length is " + ::toString(frame.size()) + " bytes.");
		}
	}
	else
	{
		// Copy frame to response buffer. Old contents are destroyed.
		copyFrameToResposeBuffer(frame.size());
	}

	removeFrameFromReceiveBuffer(frame.size());
}



//
// Copies a complete frame - in any protocol - from the main input buffer to
// the response buffer.
// The frame is *not* removed from the main input buffer.
//
void SopasBase::copyFrameToResposeBuffer(UINT32 frameLength)
{
	printInfoMessage("SopasBase::copyFrameToResposeBuffer: Copying a frame of " + ::toString(frameLength) +
								 " bytes to response buffer.", m_beVerbose);

	if (frameLength <= sizeof(m_responseBuffer))
	{
		// Wir duerfen kopieren
		memcpy(m_responseBuffer, m_receiveBuffer, frameLength);
		m_numberOfBytesInResponseBuffer = frameLength;
	}
	else
	{
		// Der respose-Buffer ist zu klein
		printError("SopasBase::copyFrameToResposeBuffer: Failed to copy frame (Length=" + ::toString(frameLength) +
									   " bytes) to response buffer because the response buffer is too small (buffer size=" +
									   ::toString(sizeof(m_responseBuffer)) + " bytes).");
		m_numberOfBytesInResponseBuffer = 0;
	}
}



//
// Removes a complete frame - in any protocol - from the main input buffer.
//
void SopasBase::removeFrameFromReceiveBuffer(UINT32 frameLength)
{
	// Remove frame from receive buffer
	if (frameLength < m_numberOfBytesInReceiveBuffer)
	{
		// More data in buffer, move them to the buffer start
		UINT32 newLen = m_numberOfBytesInReceiveBuffer - frameLength;
		printInfoMessage("SopasBase::removeFrameFromReceiveBuffer: Removing " + ::toString(frameLength) +
							 " bytes from the input buffer. New length is " + ::toString(newLen) + " bytes.", m_beVerbose);
		memmove(m_receiveBuffer, &(m_receiveBuffer[frameLength]), newLen);
		m_numberOfBytesInReceiveBuffer = newLen;
	}
	else
	{
		// No other data in buffer, just mark as empty
		printInfoMessage("SopasBase::removeFrameFromReceiveBuffer: Done, no more data in input buffer.", m_beVerbose);
		m_numberOfBytesInReceiveBuffer = 0;
	}
}



/**
 * Read command bytes from buffer (2 bytes followed by space)
 * and convert string to enum value accordingly. The command string is removed from
 * input.
 */
SopasBase::SopasCommand SopasBase::colaA_decodeCommand(std::string* rxData)
{
	return stringToSopasCommand(colaa::getNextStringToken(rxData));
}



SopasBase::SopasCommand SopasBase::stringToSopasCommand(const std::string& cmdString)
{
	// Request by Name
	if (cmdString == "RN")
	{
		return RN;
	} // Read Variable
	//	else if (cmdString == "WN") { return WN; }	// Write Variable
	//	else if (cmdString == "MN") { return MN; }	// Invoke Method
	else if (cmdString == "AN")
	{
		return AN;
	} // Method Result (Answer)
	//	else if (cmdString == "EN") { return EN; }	// Register Event
	else if (cmdString == "SN")
	{
		return SN;
	} // Send Event

	// Request by Index
	if (cmdString == "RI")
	{
		return RI;
	} // Read Variable
	else if (cmdString == "WI")
	{
		return WI;
	} // Write Variable
	else if (cmdString == "MI")
	{
		return MI;
	} // Invoke Method
	else if (cmdString == "AI")
	{
		return AI;
	} // Method Result (Answer)
	else if (cmdString == "EI")
	{
		return EI;
	} // Register Event
	else if (cmdString == "SI")
	{
		return SI;
	} // Send Event

	// Response
	else if (cmdString == "RA")
	{
		return RA;
	} // Read Variable
	else if (cmdString == "WA")
	{
		return WA;
	} // Write Variable
	else if (cmdString == "MA")
	{
		return MA;
	} // Invoke Method
	else if (cmdString == "AA")
	{
		return AA;
	} // Method Result (Answer)
	else if (cmdString == "EA")
	{
		return EA;
	} // Register Event
	else if (cmdString == "SA")
	{
		return SA;
	} // Event Acknowledge (Only used for reliable events
	else if (cmdString == "FA")
	{
		return FA;
	} // Error

	else
	{
		printError("SopasBase::stringToSopasCommand: Trying to resolve an unknown command: " + cmdString + ".");
	}
	return CMD_UNKNOWN;
}

//
// For debugging: Convert the command into a readable string.
//
std::string SopasBase::sopasCommandToString(SopasCommand cmd)
{
	
	if (cmd == RN)
	{
		return ("RN");	// Request by Name
	}
	//	else if (cmdString == "WN") { return WN; }	// Write Variable
	//	else if (cmdString == "MN") { return MN; }	// Invoke Method
	else if (cmd == AN)
	{
		return ("AN");	// Method Result, by name
	}
	//	else if (cmdString == "EN") { return EN; }	// Register Event
	else if (cmd == SN)
	{
		return ("SN");	 // Send Event
	}
	if (cmd == RI)
	{
		return ("RI");		// Request by Index
	} // Read Variable
	else if (cmd == WI)
	{
		return ("WI");		// Write Variable
	} 
	else if (cmd == MI)
	{
		return ("MI");		// Invoke Method
	}
	else if (cmd == AI)
	{
		return ("AI");		// Method Result (Answer)
	} 
	else if (cmd == EI)
	{
		return ("EI");		// Register Event
	}
	else if (cmd == SI)
	{
		return ("SI");
	} // Send Event

	// Response
	else if (cmd == RA)
	{
		return ("RA");
	} // Read Variable
	else if (cmd == WA)
	{
		return ("WA");		// Write Variable
	}
	else if (cmd == MA)
	{
		return ("MA");
	} // Invoke Method
	else if (cmd == AA)
	{
		return ("AA");
	} // Method Result (Answer)
	else if (cmd == EA)
	{
		return ("EA");
	} // Register Event
	else if (cmd == SA)
	{
		return ("SA");
	} // Event Acknowledge (Only used for reliable events)
	else if (cmd == FA)
	{
		return ("FA");		// Error
	} 
	else
	{
		printError("SopasBase::sopasCommandToString: Trying to resolve an unknown command!");
	}
	
	return "(unknown)";
}



/**
 * Get scanner type and version string
 * Also used as connection check.
 *
 * true: Information was read, false if an error occured
 */
bool SopasBase::action_getScannerTypeAndVersion()
{
	// every scanner must support ByName !!!

	bool result = false;

	// Clear old data.
	m_scannerName.empty();
	m_scannerVersion.empty();

	SopasAnswer* answer = NULL;
	result = readVariable(INDEX_DEVICE_IDENT, answer);

	if (result && answer != NULL && answer->isValid())
	{

		// decode answer
		std::string colaaAnswer;
		switch (m_protocol)
		{
		case CoLa_A:
			colaaAnswer = std::string((char*)answer->getBuffer(), answer->size());
			colaA_decodeScannerTypeAndVersion(&colaaAnswer);
			break;
		case CoLa_B:
			colaB_decodeScannerTypeAndVersion(answer->getBuffer(), 0);
			break;
		}

		if (!m_scannerName.empty() && !m_scannerVersion.empty())
		{
			result = true;
		}
	}

	if (answer != NULL)
	{
		delete answer;
	}

	return result;
}



/**
 * Decode answer for action_getScannerTypeAndVersion
 */
void SopasBase::colaA_decodeScannerTypeAndVersion(std::string* rxData)
{
	//
	// 1. part: Type
	//
	// String length
	UINT16 len = colaa::decodeUINT16(rxData);

	// Read string
	m_scannerName.clear();
	m_scannerName = rxData->substr(0, len);

	// Move input data
	*rxData = rxData->substr(len + 1);

	//
	// 2. part: Version
	//
	// String length
	len = colaa::decodeUINT16(rxData);

	// Read string
	m_scannerVersion.clear();
	m_scannerVersion = rxData->substr(0, len);
}


//
//
//
void SopasBase::colaB_decodeScannerTypeAndVersion(UINT8* buffer, UINT16 pos)
{
	printInfoMessage("SopasBase::colaB_decodeScannerTypeAndVersion: Entering function.", m_beVerbose);

	UINT16 fieldLength;

	// read device type
	fieldLength = colab::getIntegerFromBuffer<UINT16>(buffer, pos);
	m_scannerName = colab::getStringFromBuffer(buffer, pos, fieldLength);

	// read device version
	fieldLength = colab::getIntegerFromBuffer<UINT16>(buffer, pos);
	m_scannerVersion = colab::getStringFromBuffer(buffer, pos, fieldLength);

	printInfoMessage("SopasBase::colaB_decodeScannerTypeAndVersion: scanner '" + m_scannerName + "', version '"
									 + m_scannerVersion + "'.", m_beVerbose);
}


//
//
//
bool SopasBase::invokeMethod(const std::string& methodeName, BYTE* parameters, UINT16 parametersLength, SopasAnswer*& answer)
{
	// Build command
	BYTE cmdBuffer[128];
	UINT16 cmdBufferLen = 0;

	switch (m_protocol)
	{
	case CoLa_A:
		cmdBufferLen += colaa::addStringToBuffer(cmdBuffer, COMMAND_Invoke_Method_ByName);
		cmdBuffer[cmdBufferLen++] = ' ';
		//Name
		cmdBufferLen += colaa::addStringToBuffer(&(cmdBuffer[cmdBufferLen]), methodeName);

		if (parametersLength > 0)
		{
			cmdBuffer[cmdBufferLen++] = ' ';
		}
		break;
	case CoLa_B:
		colab::addStringToBuffer(cmdBuffer, cmdBufferLen, COMMAND_Invoke_Method_ByName);
		// add length of string as UINT16
		colab::addIntegerToBuffer<UINT16>(cmdBuffer, cmdBufferLen, methodeName.size());
		colab::addStringToBuffer(cmdBuffer, cmdBufferLen, methodeName); // remote method name
		break;
	}

	if (parametersLength > 0)
	{
		// add parameters (which must be already in the right encoding (colaa or colab))
		memcpy(&cmdBuffer[cmdBufferLen], parameters, parametersLength);
		cmdBufferLen += parametersLength;
	}

	// Send command
	sendCommandBuffer(&(cmdBuffer[0]), cmdBufferLen);

	// Wait for answer (the answer of a method is "AN" - not "EA")
	bool result = receiveAnswer(AN, methodeName, 2000, answer);

	// Evaluate answer
	if (result == true)
	{
		printInfoMessage("SopasBase::invokeMethod: Calling of " + methodeName + " was successful.", m_beVerbose);
	}
	else
	{
		printWarning("SopasBase::invokeMethod: Calling of " + methodeName + " was NOT successful.");
	}

	return result;
}


//
//
//
bool SopasBase::invokeMethod(UINT16 index, BYTE* parameters, UINT16 parametersLength, SopasAnswer*& answer)
{
	// Build command
	BYTE cmdBuffer[128];
	UINT16 cmdBufferLen = 0;

	switch (m_protocol)
	{
	case CoLa_A:
		printError("SopasBase::invokeMethod: Invoke method cola-a by index not supported.");
		return false;
		break;
	case CoLa_B:
		colab::addStringToBuffer(cmdBuffer, cmdBufferLen, COMMAND_Invoke_Method_ByIndex);
		// add length of string as UINT16
		colab::addIntegerToBuffer<UINT16>(cmdBuffer, cmdBufferLen, index);
		break;
	}

	if (parametersLength > 0)
	{
		// add parameters (which must be already in the right encoding (colaa or colab))
		memcpy(&cmdBuffer[cmdBufferLen], parameters, parametersLength);
		cmdBufferLen += parametersLength;
	}

	// Send command
	sendCommandBuffer(&(cmdBuffer[0]), cmdBufferLen);

	// Wait for answer
	bool result = receiveAnswer(AI, index, 2000, answer);

	// Evaluate answer
	if (result == true)
	{
		printInfoMessage("SopasBase::invokeMethod: Calling of method with index=" + ::toString(index) + " was successful.", m_beVerbose);
	}
	else
	{
		printWarning("SopasBase::invokeMethod: Calling of method with index=" + ::toString(index) + " was NOT successful.");
	}

	return result;
}


//
//
//
bool SopasBase::readVariable(const std::string& variableName, SopasAnswer*& answer)
{
	// Build command
	BYTE cmdBuffer[128];
	UINT16 cmdBufferLen = 0;

	if (m_protocol == CoLa_A)
	{
		cmdBufferLen += colaa::addStringToBuffer(&(cmdBuffer[0]), COMMAND_Read_Variable_ByName);
		cmdBuffer[cmdBufferLen++] = ' ';
		cmdBufferLen += colaa::addStringToBuffer(&(cmdBuffer[cmdBufferLen]), variableName);
	}
	else
	{
		// up to now only tested with LD-MRS
		colab::addStringToBuffer(cmdBuffer, cmdBufferLen, COMMAND_Read_Variable_ByName);
		cmdBuffer[cmdBufferLen++] = ' ';
		colab::addStringToBuffer(cmdBuffer, cmdBufferLen, variableName);
	}

	// Send
	sendCommandBuffer(&(cmdBuffer[0]), cmdBufferLen);

	// Wait for answer
	bool result = receiveAnswer(RA, variableName, 2000, answer);

	if (result)
	{
		printInfoMessage("SopasBase::readVariable: Answer to " + variableName + " received.", m_beVerbose);
	}
	else
	{
		printWarning("SopasBase::readVariable: Answer to " + variableName + " not successful.");
	}

	return result;
}


//
//
//
bool SopasBase::readVariable(UINT16 index, SopasAnswer*& answer)
{
	// Build command
	BYTE cmdBuffer[128];
	UINT16 cmdBufferLen = 0;

	if (m_protocol == CoLa_A)
	{
		cmdBufferLen += colaa::addStringToBuffer(&(cmdBuffer[0]), COMMAND_Read_Variable_ByIndex);
		cmdBuffer[cmdBufferLen++] = ' ';
		cmdBufferLen += colaa::addUINT32ToBuffer(&(cmdBuffer[cmdBufferLen]), (UINT32)index);
	}
	else
	{
		colab::addStringToBuffer(cmdBuffer, cmdBufferLen, COMMAND_Read_Variable_ByIndex);
		colab::addIntegerToBuffer<UINT16>(cmdBuffer, cmdBufferLen, index);
	}

	// Send
	sendCommandBuffer(&(cmdBuffer[0]), cmdBufferLen);

	// Wait for answer
	bool result = receiveAnswer(RA, index, 2000, answer);

	if (result)
	{
		printInfoMessage("SopasBase::readVariable: Answer to " + ::toString(index) + " received.", m_beVerbose);
	}
	else
	{
		printWarning("SopasBase::readVariable: Answer to " + ::toString(index) + " not successful.");
	}

	return result;
}



bool SopasBase::writeVariable(const std::string& variableName, BYTE* parameters, UINT16 parametersLength)
{
	if (m_readOnlyMode)
	{
		printInfoMessage("SopasBase::writeVariable: ReadOnly Modus - ignore writing to variable '" +
							variableName + "'", m_beVerbose);
		return true;
	}

	// Build command
	BYTE cmdBuffer[128];
	UINT16 cmdBufferLen = 0;

	if (m_protocol == CoLa_A)
	{
		cmdBufferLen += colaa::addStringToBuffer(&(cmdBuffer[0]), COMMAND_Write_Variable_ByName);
		cmdBuffer[cmdBufferLen++] = ' ';
		cmdBufferLen += colaa::addStringToBuffer(&(cmdBuffer[cmdBufferLen]), variableName);
		if (parametersLength > 0)
		{
			cmdBuffer[cmdBufferLen++] = ' ';
		}
	}
	else
	{
		printError("SopasBase::writeVariable: Write variable cola-b by Name: NOT IMPLEMENTED");
		return false;
// 		colab::addStringToBuffer(cmdBuffer, cmdBufferLen, COMMAND_Write_Variable_ByName);
// 		colab::addIntegerToBuffer<UINT16>(cmdBuffer, cmdBufferLen, variableName.size()); // add length of string as UINT16
// 		colab::addStringToBuffer(cmdBuffer, cmdBufferLen, variableName);
// 		or ?
// 		colab::addStringToBuffer(cmdBuffer, cmdBufferLen, COMMAND_Write_Variable_ByName);
// 		colab::addStringToBuffer(cmdBuffer, cmdBufferLen, variableName);
	}


	if (parametersLength > 0)
	{
		// add parameters (which must be already in the right encoding (colaa or colab))
		memcpy(&cmdBuffer[cmdBufferLen], parameters, parametersLength);
		cmdBufferLen += parametersLength;
	}


	// Send
	sendCommandBuffer(&(cmdBuffer[0]), cmdBufferLen);
	SopasAnswer* answer = NULL;
	// Wait for answer
	bool result = receiveAnswer(WA, variableName, 2000, answer);
	// free memory for answer
	if (answer != NULL)
	{
		delete answer;
	}

	if (result)
	{
		printInfoMessage("SopasBase::writeVariable: Answer to " + variableName + " received.", m_beVerbose);
	}
	else
	{
		printInfoMessage("SopasBase::writeVariable: Answer to " + variableName + " not successful.", m_beVerbose);
	}

	return result;
}


//
// Write a variable, addressed by index.
//
bool SopasBase::writeVariable(UINT16 variableIndex, BYTE* parameters, UINT16 parametersLength)
{
	bool beVerboseHere = m_beVerbose;
//	beVerboseHere = true;
	
	if (m_readOnlyMode == true)
	{
		printInfoMessage("SopasBase::writeVariable: ReadOnly Modus - ignore writing to variable index '" + ::toString(variableIndex) +
									 "'", m_beVerbose);
		return true;
	}

	// Create the command buffer
	UINT32 cmdBufferLen = parametersLength + 4;
	BYTE* cmdBuffer = new BYTE[cmdBufferLen];
	
	// Add the command
	colab::addStringToBuffer(&(cmdBuffer[0]), COMMAND_Write_Variable_ByIndex);
	
	// Add the index
	BYTE* buffer = &(cmdBuffer[2]);
	memwrite_UINT16(buffer, variableIndex);
//	UINT16 pos = 0;
//	colab::addIntegerToBuffer<UINT16>(&(cmdBuffer[2]), pos, variableIndex);
	
	// Copy the data
	memcpy(&(cmdBuffer[4]), parameters, parametersLength);
	
	// Send. The frame is added automatically.
	printInfoMessage("SopasBase::writeVariable: Sending command buffer now (payload len=" + toString(parametersLength+4) + " bytes).", beVerboseHere);
	sendCommandBuffer(cmdBuffer, cmdBufferLen);
	
	printInfoMessage("SopasBase::writeVariable: Command sent, waiting for reply...", beVerboseHere);
	SopasAnswer* answer = NULL;
	// Wait for answer
	bool result = receiveAnswer(WA, variableIndex, 2000, answer);
	// free memory for answer
	if (answer != NULL)
	{
		delete answer;
	}
	if (result)
	{
		printInfoMessage("SopasBase::writeVariable: Answer to " + toString(variableIndex) + " received.", beVerboseHere);
	}
	else
	{
		printWarning("SopasBase::writeVariable: Answer to " + toString(variableIndex) + " not successful!");
	}

	printInfoMessage("SopasBase::writeVariable: All done, leaving.", beVerboseHere);
	return result;
}



bool SopasBase::registerEvent(const std::string& eventName)
{
	BYTE cmdBuffer[128];
	UINT16 cmdBufferLen = 0;

	if (m_protocol == CoLa_A)
	{
		cmdBufferLen += colaa::addStringToBuffer(&(cmdBuffer[0]), COMMAND_Register_Event_ByName);
		cmdBuffer[cmdBufferLen++] = ' ';
		cmdBufferLen += colaa::addStringToBuffer(&(cmdBuffer[cmdBufferLen]), eventName);
		cmdBuffer[cmdBufferLen++] = ' ';
		cmdBuffer[cmdBufferLen++] = '1';
	}
	else
	{
		//traceError(SOPASBASE_VERSION) << "register event cola-B by name not supported." << std::endl;
		colab::addStringToBuffer(cmdBuffer, cmdBufferLen, COMMAND_Register_Event_ByName);
		cmdBuffer[cmdBufferLen++] = ' ';
		colab::addStringToBuffer(cmdBuffer, cmdBufferLen, eventName);
		cmdBuffer[cmdBufferLen++] = ' ';
		colab::addIntegerToBuffer<UINT8>(cmdBuffer, cmdBufferLen, 1);

		// return false;
	}

	// Send
	sendCommandBuffer(&(cmdBuffer[0]), cmdBufferLen);
	SopasAnswer* answer = NULL;
	// Wait for answer
	bool result = receiveAnswer(EA, eventName, 2000, answer);


	// free memory for answer
	if (answer != NULL)
	{
		delete answer;
	}
	return result;
}


//
//
//
bool SopasBase::registerEvent(UINT16 index)
{
	// Build command
	BYTE cmdBuffer[128];
	UINT16 cmdBufferLen = 0;

	switch (m_protocol)
	{
	case CoLa_A:
		printError("SopasBase::register event cola-a by index not supported, aborting.");

		return false;

		break;
	case CoLa_B:
		colab::addStringToBuffer(cmdBuffer, cmdBufferLen, COMMAND_Register_Event_ByIndex);
		colab::addIntegerToBuffer<UINT16>(cmdBuffer, cmdBufferLen, index);
		colab::addIntegerToBuffer<UINT8>(cmdBuffer, cmdBufferLen, 1);  // 1 to subscribe
		break;
	}

	// Send command
	sendCommandBuffer(&(cmdBuffer[0]), cmdBufferLen);

	// Wait for answer
	SopasAnswer* answer = NULL;
	bool result = receiveAnswer(EA, index, 2000, answer);

	// there will be no answer (but to be sure to prevent memory leaks)
	if (answer != NULL)
	{
		delete answer;
	}

	// Evaluate answer
	if (result == true)
	{
		printInfoMessage("SopasBase::registerEvent: Calling of register with index=" + ::toString(index) + " was successful.", m_beVerbose);
	}
	else
	{
		printError("SopasBase::registerEvent: Calling of method with index=" + ::toString(index) + " was NOT successful.");
	}
	return result;
}



bool SopasBase::unregisterEvent(UINT16 index)
{
	// Build command
	BYTE cmdBuffer[128];
	UINT16 cmdBufferLen = 0;

	switch (m_protocol)
	{
	case CoLa_A:
		printError("SopasBase::unregisterEvent: Unregister event cola-a by index not supported.");

		return false;

		break;
	case CoLa_B:
		colab::addStringToBuffer(cmdBuffer, cmdBufferLen, COMMAND_Register_Event_ByIndex);
		colab::addIntegerToBuffer<UINT16>(cmdBuffer, cmdBufferLen, index);
		colab::addIntegerToBuffer<UINT8>(cmdBuffer, cmdBufferLen, 0);  // 1 to subscribe
		break;
	}

	// Send command
	sendCommandBuffer(&(cmdBuffer[0]), cmdBufferLen);

	// Wait for answer
	SopasAnswer* answer = NULL;
	bool result = receiveAnswer(EA, index, 2000, answer);

	// there will be no answer (but to be sure to prevent memory leaks)
	if (answer != NULL)
	{
		delete answer;
	}

	// Evaluate answer
	if (result == true)
	{
		printInfoMessage("SopasBase::calling of register with index=" + ::toString(index) + " was successful.", m_beVerbose);
	}
	else
	{
		printInfoMessage("SopasBase::calling of register with index=" + ::toString(index) + " was NOT successful.", m_beVerbose);
	}

	return result;
}



bool SopasBase::unregisterEvent(const std::string& eventName)
{
	BYTE cmdBuffer[128];
	UINT16 cmdBufferLen = 0;

	if (m_protocol == CoLa_A)
	{
		cmdBufferLen += colaa::addStringToBuffer(&(cmdBuffer[0]), COMMAND_Register_Event_ByName);
		cmdBuffer[cmdBufferLen++] = ' ';
		cmdBufferLen += colaa::addStringToBuffer(&(cmdBuffer[cmdBufferLen]), eventName);
		cmdBuffer[cmdBufferLen++] = ' ';
		cmdBuffer[cmdBufferLen++] = '0';
	}
	else
	{
//		traceError(SOPASBASE_VERSION) << "unregister event cola-B by name not supported." << std::endl;
		colab::addStringToBuffer(cmdBuffer, cmdBufferLen, COMMAND_Register_Event_ByName);
		cmdBuffer[cmdBufferLen++] = ' ';
		colab::addStringToBuffer(cmdBuffer, cmdBufferLen, eventName);
		cmdBuffer[cmdBufferLen++] = ' ';
		colab::addIntegerToBuffer<UINT8>(cmdBuffer, cmdBufferLen, 0);
//		return false;
	}

	// Send
	sendCommandBuffer(&(cmdBuffer[0]), cmdBufferLen);
	SopasAnswer* answer = NULL;
	// Wait for answer
	bool result = receiveAnswer(EA, eventName, 2000, answer);


	// free memory for answer
	if (answer != NULL)
	{
		delete answer;
	}
	return result;
}



/**
 * Map angle to range ]PI..-PI]
 */
double SopasBase::makeAngleValid(double angle)
{
	return ::normalizeRadians(angle);
}



//
// ************************* SOPAS FRAME ************************************************** //
//
SopasEventMessage::SopasEventMessage() :
	m_buffer(NULL), m_protocol(SopasBase::CoLa_A), m_frameLength(0), m_encoding(SopasBase::ByName)
{
}



SopasEventMessage::SopasEventMessage(BYTE* buffer, SopasBase::SopasProtocol protocol, UINT32 frameLength) :
	m_buffer(buffer), m_protocol(protocol), m_frameLength(frameLength), m_encoding(SopasBase::ByName)
{
	detectEncoding();
	detectMessageType();
}



UINT32 SopasEventMessage::getPayLoadLength() const
{
	UINT32 payLoadLength = 0;

	switch (m_protocol)
	{
	case SopasBase::CoLa_A:
		payLoadLength = m_frameLength - 2; // everything except the 0x02 0x03 frame
		break;
	case SopasBase::CoLa_B:
		payLoadLength = m_frameLength - 9; // everything except start 0x02020202(4byte), payloadLength(4byte) and checksum(1 byte)
	}

	return payLoadLength;
}



std::string SopasEventMessage::getCommandString() const
{
	std::string commandString;

	switch (m_protocol)
	{
	case SopasBase::CoLa_A:
		commandString = std::string((char*) &m_buffer[2], 2);
		break;
	case SopasBase::CoLa_B:
		commandString = std::string((char*) &m_buffer[9], 2);
	}

	return commandString;
}


//
// Returns a pointer to the first payload byte. 
// CoLa-A: Points beyond the leading "0x02" to the "s..." data.
// CoLa-B: Points beyond the magic word and length bytes, to the "s..." data.
//
BYTE* SopasEventMessage::getPayLoad()
{
	BYTE* bufferPos = NULL;

	switch (m_protocol)
	{
	case SopasBase::CoLa_A:
		bufferPos = &m_buffer[1];
		break;
	case SopasBase::CoLa_B:
		bufferPos = &m_buffer[8];
		break;
	}

	return bufferPos;
}



INT32 SopasEventMessage::getVariableIndex()
{
	INT32 index = -1;

	if (m_encoding != SopasBase::ByIndex)
	{
		// Encoding is not byIndex, so abort here
		printWarning("SopasEventMessage::getVariableIndex: Encoding is not ByIndex, aborting!");
		return index;
	}
	
	BYTE* bufferPos = &getPayLoad()[3];
	switch (m_protocol)
	{
		case SopasBase::CoLa_A:
			index = (INT32)(colaa::decodeUINT16(bufferPos));
			break;
		case SopasBase::CoLa_B:
			index = (INT32)(colab::decodeUINT16(bufferPos));
			break;
		default:
			printError("SopasEventMessage::getVariableIndex: Unknown protocol!");
	}

	return index;
}

//
// Read the variable name from a sensor message. This works only if the encoding "ByName" is used!
//
std::string SopasEventMessage::getVariableName()
{
	std::string name;
	UINT32 i;
	BYTE* bufferPos;

	if (m_encoding == SopasBase::ByName)
	{
		switch (m_protocol)
		{
		case SopasBase::CoLa_A:
			printError("SopasEventMessage::getVariableName: Protocol CoLa-A is not supported, aborting!");
			return "";
			break;
		case SopasBase::CoLa_B:
			bufferPos = &getPayLoad()[4];
			i = 4;

			// example for message "sSI <variablename> 0x000binarydata"

			// search for the next white space
			while ((*bufferPos != ' ') && (i < getPayLoadLength()))
			{
				name += *bufferPos;
				bufferPos++;
				i++;
			}
			break;
		}
	}

	return name;
}


//
// Detects the encoding method (ByName or ByIndex) from the sensor message.
//
void SopasEventMessage::detectEncoding()
{
	// if the third byte of the payload is an 'I', the encoding is ByIndex
	// sXI: sAI, sRI, sWI, sMI, sEI, sSI
	if (getPayLoad()[2] == 'I')
	{
		m_encoding = SopasBase::ByIndex;
	}
}



void SopasEventMessage::detectMessageType()
{
	std::string command = getCommandString();

	if (command == SopasBase::COMMAND_Event_Acknowledge)
	{
		m_messageType = SopasBase::MSG_EVENT_ACKNOWLEDGE;
	}
	else if (command == SopasBase::COMMAND_Invoke_Method_Answer)
	{
		m_messageType = SopasBase::MSG_INVOKE_METHOD_ANSWER;
	}
	else if (command == SopasBase::COMMAND_Method_Result_Answer)
	{
		m_messageType = SopasBase::MSG_METHOD_RESULT_ANSWER;
	}
	else if (command == SopasBase::COMMAND_Send_Event_ByIndex || command == SopasBase::COMMAND_Send_Event_ByName)
	{
		m_messageType = SopasBase::MSG_SEND_EVENT;
	}
	else if (command == SopasBase::COMMAND_Read_Variable_Answer)
	{
		m_messageType = SopasBase::MSG_READ_VARIABLE_ANSWER;
	}
	else if (command == SopasBase::COMMAND_Write_Variable_Answer)
	{
		m_messageType = SopasBase::MSG_WRITE_VARIABLE_ANSWER;
	}
	else
	{
		m_messageType = SopasBase::MSG_UNKNOWN;
	}
}

SopasAnswer::SopasAnswer(const BYTE* answer, UINT32 answerLength) : m_answerLength(answerLength)
{
	if (answerLength > 0)
	{
		m_answerBuffer = new BYTE[m_answerLength];
		memcpy(m_answerBuffer, answer, answerLength);
	}
	else
	{
		m_answerBuffer = NULL;
	}
}

SopasAnswer::~SopasAnswer()
{
	if (m_answerBuffer != NULL)
	{
		delete[] m_answerBuffer;
		m_answerLength = 0;
	}
}

}	// namespace devices
