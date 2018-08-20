//
// SopasBase.h
//
//  Created on: 18.07.2011
//     Author: sick
//

#ifndef SOPASBASE_H
#define SOPASBASE_H

#include "../BasicDatatypes.hpp"
#include "../datatypes/Scan.hpp"
#include "../interfaces/tcp.hpp"

#include "colaa.hpp"
#include "colab.hpp"
#include <map>	// for std::map
#include "../tools/Mutex.hpp"

namespace devices
{
	
class SopasEventMessage;
class SopasAnswer;

/**
 * Class SopasBase encapsuls the communication to a sensor via SopasProtocol. It offers the functions: </br>
 * 			- invokeMethode<br/>
 * 			- readVariable<br/>
 * 			- writeVariable<br/>
 * 			- (un)registerEvent<br/>
 *
 * 		Callback functions are used to inform you about incoming events (scans or eval cases).
 */
class SopasBase
{
public:
	static const std::string EVENTNAME_SUBSCRIBE_EVALCASES;
	static const std::string EVENTNAME_SUBSCRIBE_SCANS;
	static const std::string METHODNAME_LOGIN;
	static const std::string METHODNAME_LOGOUT;
	static const std::string METHODNAME_SET_SCANCONFIG;
	static const std::string METHODNAME_START_MEASURE;
	static const std::string METHODNAME_STOP_MEASURE;
	static const std::string VARIABLENAME_DEVICEIDENT;
	static const std::string VARIABLENAME_SCANCONFIG;
	static const std::string VARIABLENAME_DATAOUTPUTRANGE;
	static const std::string VARIABLENAME_SCANDATACONFIG;

	// sopas commands
	static const std::string COMMAND_Read_Variable_ByIndex;
	static const std::string COMMAND_Write_Variable_ByIndex;
	static const std::string COMMAND_Invoke_Method_ByIndex;
	static const std::string COMMAND_Method_Result_ByIndex;
	static const std::string COMMAND_Register_Event_ByIndex;
	static const std::string COMMAND_Send_Event_ByIndex; // receive data event

	static const std::string COMMAND_Read_Variable_Answer;
	static const std::string COMMAND_Write_Variable_Answer;
	static const std::string COMMAND_Invoke_Method_Answer;
	static const std::string COMMAND_Method_Result_Answer;
	static const std::string COMMAND_Register_Event_Answer;
	static const std::string COMMAND_Event_Acknowledge;

	static const std::string COMMAND_Read_Variable_ByName;
	static const std::string COMMAND_Write_Variable_ByName;
	static const std::string COMMAND_Invoke_Method_ByName;
	static const std::string COMMAND_Method_Result_ByName;
	static const std::string COMMAND_Register_Event_ByName;
	static const std::string COMMAND_Send_Event_ByName; // receive data event

	static const UINT16 INDEX_DEVICE_IDENT;

	enum SopasProtocol
	{
		CoLa_A, ///< Command Language ASCI
		CoLa_B  ///< Command Language binary
	};

	enum SopasEncoding
	{
		ByName, ///< read/write variable, invoke methods by name
		ByIndex ///< read/write variable, invoke methods by index (indexes will be generated !!!)
	};

	/// types of answers of the sensor
	enum SopasMessageType
	{
		MSG_UNKNOWN, ///< Unknown message
		//		MSG_READ_VARIABLE, ///< Read Variable
		//		MSG_WRITE_VARIABLE, ///< Write Variable
		//		MSG_INVOKE_METHOD, ///< Invoke Method
		//		MSG_METHOD_RESULT, ///< Method Result
		//		MSG_REGISTER_EVENT, ///< Register Event
		MSG_SEND_EVENT, ///< Send Event
		MSG_READ_VARIABLE_ANSWER, ///< Read Variable Answer
		MSG_WRITE_VARIABLE_ANSWER, ///< Write Variable Answer
		MSG_INVOKE_METHOD_ANSWER, ///< Invoke Method Answer
		MSG_METHOD_RESULT_ANSWER, ///< Method Result Answer
		MSG_REGISTER_EVENT_ANSWER, ///< Register Event Answer
		MSG_EVENT_ACKNOWLEDGE, ///< Event Acknowledge -Answer to register event
		MSG_ERROR
		///< Error
	};

	typedef void (*DecoderFunction)(SopasEventMessage& frame);	//  Decoder for events

	/// Default constructor.
	SopasBase();

	/// Destructor
	virtual ~SopasBase();

	/// Initialization
	/**
	 * @brief
	 * @param protocol
	 * @param ipAddress
	 * @param portNumber
	 * @param weWantScanData
	 * @param weWantFieldData
	 * @param readOnlyMode
	 * @param disconnectFunction Function to be called on disconnect events.
	 * obj = pointer to the object that holds the disconnectFunction
	 * @return
	 */
	virtual bool init(SopasProtocol protocol,
						std::string ipAddress,
						UINT16 portNumber,
						bool weWantScanData,
						bool weWantFieldData,
						bool readOnlyMode,
						Tcp::DisconnectFunction disconnectFunction,
						void* obj);

	/// Connects to a sensor via tcp and reads the device name.
	bool connect();

	/// Returns true if the tcp connection is established.
	bool isConnected();

	/** \brief Closes the connection to the LMS. This is the opposite of init().
	 *
	 * Switches this device from the CONNECTED state back in the
	 * CONSTRUCTED state.
	 *
	 * \return True if the device is now in the CONSTRUCTED state
	 */
	bool disconnect();

	/**
	 * @brief Reads the scanner type and version variable from the sensor and stores it in the
	 *        member variables. This is done always by name.
	 * @return true if no errors occurred.
	 */
	bool action_getScannerTypeAndVersion();


	void setReadOnlyMode(bool mode);

	bool isReadOnly();

	/**
	 * @brief Invoke a method on the sensor.
	 * @param methodeName name of the method to call
	 * @param parameters byte buffer with parameter (NOTE: you have to fill this buffer with the correct protocol - cola-a or cola-b)
	 * @param parametersLength length of the byte buffer
	 * @param answer pointer to an answer message (NOTE: memory for this object will be allocated - free this after usage !!!)
	 * @return true if no errors occurred.
	 */
	bool invokeMethod(const std::string& methodeName, BYTE* parameters, UINT16 parametersLength, SopasAnswer*& answer);

	/**
	 * @brief Invoke a method on the sensor.
	 * @param index index of the method to call
	 * @param parameters byte buffer with parameter (NOTE: you have to fill this buffer with the correct protocol - cola-a or cola-b)
	 * @param parametersLength length of the byte buffer
	 * @param answer pointer to an answer message (NOTE: memory for this object will be allocated - free this after usage !!!)
	 * @return true if no errors occurred.
	 */
	bool invokeMethod(UINT16 index, BYTE* parameters, UINT16 parametersLength, SopasAnswer*& answer);

	/**
	 * @brief Reads a variable from the sensor by name.
	 * @param variableName name of the variable
	 * @param answer pointer to an answer message (NOTE: memory for this object will be allocated - free this after usage !!!)
	 * @return true if no errors occurred.
	 */
	bool readVariable(const std::string& variableName, SopasAnswer*& answer);

	/**
	 * @brief Reads a variable from the sensor by index.
	 * @param index of the variable
	 * @param answer
	 * @return true if no errors occurred.
	 */
	bool readVariable(UINT16 index, SopasAnswer*& answer);

	/**
	 * @brief Write a variable to the sensor by name.
	 * @param variableName name of the variable.
	 * @param parameters byte buffer with parameter (NOTE: you have to fill this buffer with the correct protocol - cola-a or cola-b)
	 * @param parametersLength length of the byte buffer
	 * @return true if no errors occurred.
	 */
	bool writeVariable(const std::string& variableName, BYTE* parameters, UINT16 parametersLength);

	/**
	 * @brief Write a variable to the sensor by index.
	 * @param index of the variable
	 * @param parameters byte buffer with parameter (NOTE: you have to fill this buffer with the correct protocol - cola-a or cola-b)
	 * @param parametersLength length of the byte buffer
	 * @return true if no errors occurred.
	 */
	bool writeVariable(UINT16 index, BYTE* parameters, UINT16 parametersLength);

	/**
	 * @brief Registers an event by name.
	 * @param eventName name of the event
	 * @return true if no errors occurred.
	 */
	bool registerEvent(const std::string& eventName);

	/**
	 * @brief Registers an event by index.
	 * @param index of the event.
	 * @return true if no errors occurred.
	 */
	bool registerEvent(UINT16 index);

	/**
	 * @brief Unregisters an event by name.
	 * @param eventName name of the event
	 * @return true if no errors occurred.
	 */
	bool unregisterEvent(const std::string& eventName);

	/**
	 * @brief Unregisters an event by index.
	 * @param index of the event
	 * @return true if no errors occurred.
	 */
	bool unregisterEvent(UINT16 index);

	/**
	 *
	 * @param decoderFunction
	 * @param eventName
	 */
	void setEventCallbackFunction(DecoderFunction decoderFunction, const std::string& eventName)
	{
		m_decoderFunctionMapByName[eventName] = decoderFunction;
	}

	/**
	 *
	 * @param decoderFunction
	 * @param eventIndex
	 */
	void setEventCallbackFunction(DecoderFunction decoderFunction, UINT16 eventIndex)
	{
		m_decoderFunctionMapByIndex[eventIndex] = decoderFunction;
	}


	double makeAngleValid(double angle);

	const std::string& getScannerName() const { return m_scannerName; }
	const std::string& getScannerVersion() const { return m_scannerVersion; }

	// Convert a SOPAS error code to readable text
	static std::string convertSopasErrorCodeToText(UINT16 errorCode);

protected:

	enum SopasCommand
	{
		CMD_UNKNOWN = 0, ///< Unknown command
		RI = 1, ///< Read Variable
		WI = 2, ///< Write Variable
		MI = 3, ///< Invoke Method
		AI = 4, ///< Method Result
		EI = 5, ///< Register Event
		SI = 6, ///< Send Event
		RA = 7, ///< Read Variable Answer
		WA = 8, ///< Write Variable Answer
		MA = 9, ///< Invoke Method Answer
		AA = 10, ///< Method Result Answer
		EA = 11, ///< Register Event Answer
		SA = 12, ///< Event Acknowledge
		RN = 20, ///< Read Variable (by name)
		AN = 21, ///< Method Result (ny name)
		SN = 22, ///< Send Event (by name, receive)
		FA = 50
		///< Error
	};

	enum State
	{
		/// Object has been constructed. Use init() to go into CONNECTED state.
		CONSTRUCTED
		/// Object is now connected. Use run() to go into RUNNING
		/// state, or disconnect() to go back into CONSTRUCTED state.
		,
		CONNECTED
		/// Object is connected and emitting data. Use stop() to go back into CONNECTED,
		/// or disconnect() to go back into CONSTRUCTED state.
		//		, RUNNING
	};

	/**
	 * @brief Take answer from read thread and decode it.
	 *        Waits for a certain answer by name. Event data (scans) are filtered and processed
	 *        by read thread.
	 * @param cmd Waits for the answer to this command.
	 * @param name name of the method/variable.
	 * @param timeout in [ms]
	 * @param answer Pointer to answer. Will be filled if answer contains parameter.
	 * @return true if no error occurred.
	 */
	bool receiveAnswer(SopasCommand cmd, std::string name, UINT32 timeout, SopasAnswer*& answer);
	bool receiveAnswer_CoLa_A(SopasCommand cmd, std::string name, UINT32 timeout, SopasAnswer*& answer );
	bool receiveAnswer_CoLa_B(SopasCommand cmd, std::string name, UINT32 timeout, SopasAnswer*& answer );
	bool receiveAnswer(SopasCommand cmd, UINT16 index, UINT32 timeout, SopasAnswer*& answer );
	bool receiveAnswer_CoLa_A(SopasCommand cmd, UINT16 index, UINT32 timeout, SopasAnswer*& answer);
	bool receiveAnswer_CoLa_B(SopasCommand cmd, UINT16 index, UINT32 timeout, SopasAnswer*& answer);

	/**
	 * @brief Sends the content of the buffer via TCP to the sensor.
	 * @param buffer pointer to the buffer
	 * @param len length of buffer to be sent.
	 */
	void sendCommandBuffer(UINT8* buffer, UINT16 len);

	SopasCommand colaA_decodeCommand(std::string* rxData);

	/// Converts strings in sopas answer buffer to SopasCommand enum.
	SopasCommand stringToSopasCommand(const std::string& cmdString);
	std::string sopasCommandToString(SopasCommand cmd);

protected:
	// Decoder functions that need to be overwritten by derived classes
	virtual void evalCaseResultDecoder(SopasEventMessage& msg) = 0;
	virtual void scanDataDecoder(SopasEventMessage& msg) = 0;
	
	bool m_scanEventIsRegistered;
	bool m_fieldEventIsRegistered;
	bool m_weWantScanData; ///< Flag to enable/disable scan data reception
	bool m_weWantFieldData; ///< Flag to enable/disable protection field data reception

	/// Device info
	State m_state;
	std::string m_scannerName; ///< Read from scanner
	std::string m_scannerVersion; ///< Read from scanner

	bool m_beVerbose; //   true = Show extended status traces

	bool m_isLoggedIn;

private:
	// TCP
	bool openTcpConnection();
	void closeTcpConnection();

	/// Function that will be called on incomming data via tcp.
	static void readCallbackFunctionS(void* obj, UINT8* buffer, UINT32& numOfBytes);
	void readCallbackFunction(UINT8* buffer, UINT32& numOfBytes);

	/// Depending on the protocol the start and end of a frame will be found.
	SopasEventMessage findFrameInReceiveBuffer();

	/// Reads one frame from receive buffer and decodes it.
	void processFrame(SopasEventMessage& frame);
	void processFrame_CoLa_A(SopasEventMessage& frame);
	void processFrame_CoLa_B(SopasEventMessage& frame);
	void copyFrameToResposeBuffer(UINT32 frameLength);
	void removeFrameFromReceiveBuffer(UINT32 frameLength);

	// SOPAS / Cola
	SopasProtocol m_protocol; ///< Used protocol (ColaA oder ColaB)
	SopasEncoding m_encoding; ///< ByName or ByIndex
	void colaA_decodeScannerTypeAndVersion(std::string* rxData);
	void colaB_decodeScannerTypeAndVersion(UINT8* buffer, UINT16 pos);

private:
	typedef std::map<std::string, DecoderFunction> DecoderFunctionMapByName;
	typedef std::map<UINT16, DecoderFunction> DecoderFunctionMapByIndex;
	DecoderFunctionMapByName m_decoderFunctionMapByName;
	DecoderFunctionMapByIndex m_decoderFunctionMapByIndex;

//	DecoderFunction m_scanDecoderFunction;
//	DecoderFunction m_evalCaseDecoderFunction;

	typedef std::map<UINT16, std::string> IndexToNameMap;
	IndexToNameMap m_indexToNameMap;

	// Response buffer
	UINT32 m_numberOfBytesInResponseBuffer; ///< Number of bytes in buffer
	UINT8 m_responseBuffer[1024]; ///< Receive buffer for everything except scan data and eval case data.
	Mutex m_receiveDataMutex; ///< Access mutex for buffer

	// Receive buffer
	UINT32 m_numberOfBytesInReceiveBuffer; ///< Number of bytes in buffer
	UINT8 m_receiveBuffer[25000]; ///< Low-Level receive buffer for all data (25000 should be enough for NAV300 Events)

	// TCP
	Tcp m_tcp;
	std::string m_ipAddress;
	UINT16 m_portNumber;

	bool m_readOnlyMode;
};


/// Class that represents a message that was sent by a sensor. (Event message)
class SopasEventMessage
{
public:
	/// Default constructor
	SopasEventMessage();

	/// Destructor
	~SopasEventMessage() {}

	/**
	 * @brief Constructor. This class will only store a pointer to the byte buffer. It will not deallocate the
	 *        memory. Please make sure that the buffer is not deallocated while you are working with this class.
	 * @param buffer byte buffer with the message (Sopas frame)
	 * @param protocol type of protocol (Cola-A, Cola-B)
	 * @param frameLength length of the frame
	 */
	SopasEventMessage(BYTE* buffer, SopasBase::SopasProtocol protocol, UINT32 frameLength);

	SopasBase::SopasProtocol getProtocolType() const
	{
		return m_protocol;
	}

	SopasBase::SopasEncoding getEncodingType() const
	{
		return m_encoding;
	}

	SopasBase::SopasMessageType getMessageType() const
	{
		return m_messageType;
	}

	UINT32 size() const
	{
		return m_frameLength;
	}

	/// contains 's' + command string(2 byte) + content(payload length - 3)
	UINT32 getPayLoadLength() const;

	std::string getCommandString() const;

	/// contains 's' + command string(2 byte) + content(payload length - 3)
	BYTE* getPayLoad();

	/// Returns the index of a variable (answer to read variable by index). In case of error a negative value will be returned
	INT32 getVariableIndex();

	/// Returns the name of a variable (answer to read variable by name). In case of error an empty value will be returned
	std::string getVariableName();

	bool isValid() const { return (m_buffer != NULL); }

private:
	void detectEncoding();

	void detectMessageType();

private:
	BYTE* m_buffer;
	SopasBase::SopasProtocol m_protocol;
	UINT32 m_frameLength;
	SopasBase::SopasEncoding m_encoding;
	SopasBase::SopasMessageType m_messageType;
};

/// Class that encapsulates a buffer that was sent as return to a sync call. (variable / method)
class SopasAnswer
{
public:
	/// Constructor. Copies the content of the answer into the buffer of this object.
	SopasAnswer(const BYTE* answer, UINT32 answerLength);

	/// Destructor. Frees the memory for the copied buffer.
	~SopasAnswer();

	BYTE* getBuffer() { return m_answerBuffer; }

	UINT32 size() { return m_answerLength; }

	bool isValid() { return (m_answerBuffer != NULL); }

private:
	UINT32 m_answerLength;
	BYTE*  m_answerBuffer;
};

}	// namespace devices

#endif // SOPASBASE_H
