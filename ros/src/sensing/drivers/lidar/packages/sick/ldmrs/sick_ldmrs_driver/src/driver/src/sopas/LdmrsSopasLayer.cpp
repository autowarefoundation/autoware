//
// LdmrsSopasLayer.cpp
//
//
#include "LdmrsSopasLayer.hpp"
#include "../datatypes/EvalCaseResult.hpp"
#include "../tools/errorhandler.hpp"

namespace devices
{
	
using namespace datatypes;

//
// ****************************** LdmrsSopasLayer ************************************* //
//



LdmrsSopasLayer::LdmrsSopasLayer(Manager* manager,
									const UINT8 deviceId,
									std::string ipAddress,
									UINT16 portNumber,
									bool weWantFieldData,
									bool weWantScanData,
									bool readOnlyMode)
{
	m_beVerbose = false;
	
	m_manager = manager;
	m_deviceId = deviceId;
	m_ipAddress = ipAddress;
	m_portNumber = portNumber;
	m_weWantFieldData = weWantFieldData;
	m_weWantScanData = weWantScanData;
	m_readOnlyMode = readOnlyMode;
}



LdmrsSopasLayer::~LdmrsSopasLayer()
{
}



bool LdmrsSopasLayer::init(Tcp::DisconnectFunction disconnectFunction, void* obj)
{
	bool success = SopasBase::init(CoLa_B,
									m_ipAddress,
									m_portNumber,
									m_weWantScanData,
									m_weWantFieldData,
									m_readOnlyMode,
									disconnectFunction, obj);

	if (success == true)
	{
		// Success
		printInfoMessage("LdmrsSopasLayer::init: SopasBase was initialized successfully.", m_beVerbose);
	}
	else
	{
		printError("LdmrsSopasLayer::init: Failed, aborting!");
		return false;
	}
	
	//
	// Connect to the sensor
	//
	success = connect();
	if (success == true)
	{
		// Success
		printInfoMessage("LdmrsSopasLayer::init: Connected to scanner successfully.", m_beVerbose);
	}
	else
	{
		printError("LdmrsSopasLayer::init: Failed to connect to scanner, aborting!");
		return false;
	}
	
	// Read the field and eval case configuration, but only if we want field data
	if (m_weWantFieldData == true)
	{
		// Field configuration
		printInfoMessage("LdmrsSopasLayer::init: Reading current field configuration.", m_beVerbose);
		success = action_readFields();
		if (success == true)
		{
			printInfoMessage("LdmrsSopasLayer::init: Successfully read the field configuration (" +
								::toString(m_fields.getFields().size()) + " fields).", m_beVerbose);

			// Post the fields to the manager
			Fields* f = new Fields;
			*f = m_fields;
			f->setSourceId(m_deviceId);
			m_manager->setDeviceData(f);
		}
		else
		{
			// Fail
			printError("LdmrsSopasLayer::init: Failed to read the field configuration, aborting!");
			return false;
		}
		
		// Eval cases
		printInfoMessage("LdmrsSopasLayer::init: Reading current eval case configuration.", m_beVerbose);
		success = action_readEvalCases();
		if (success == true)
		{
			printInfoMessage("LdmrsSopasLayer::init: Successfully read the eval cases (" +
								::toString(m_evalCases.getEvalCases().size()) + " cases).", m_beVerbose);
			
			// Post the eval cases to the manager
			EvalCases* e = new EvalCases;
			*e = m_evalCases;
			e->setSourceId(m_deviceId);
			m_manager->setDeviceData(e);
		}
		else
		{
			// Fail
			printError("LdmrsSopasLayer::init: Failed to read the eval cases, aborting!");
			return false;
		}
	}
	else
	{
		printInfoMessage("LdmrsSopasLayer::init: Skipping field configuration as we want no field data.", m_beVerbose);
	}
	
	return success;
}

//
// Register for the wanted events.
//
bool LdmrsSopasLayer::run()
{
	bool beVerboseHere = m_beVerbose;
//	beVerboseHere = true;
	
	printInfoMessage("LdmrsSopasLayer::run: Called.", beVerboseHere);
	
	bool result = false;
	
	// Field data?
	if (m_weWantFieldData == true)
	{
		printInfoMessage("LdmrsSopasLayer::run: Now subscribing to EvalCaseResults.", beVerboseHere);
		result = action_subscribeEvalCaseResults();
		if (result == false)
		{
			// Fail
			printError("LdmrsSopasLayer::run: Failed to subscribe to EvalCaseResults, aborting.");
			return false;
		}
		printInfoMessage("LdmrsSopasLayer::run: Subscription to EvalCaseResults was successful.", beVerboseHere);
	}
	
	// Here, we could also subscribe to scan data. However, scans are handeled more efficientely over the LUX
	// interface, so we ignore this part here.
	if (m_weWantScanData == true)
	{
//		printWarning("LdmrsSopasLayer::run: Scan data reading via SOPAS / CoLa is not implemented, ignoring this setting.");
		printInfoMessage("LdmrsSopasLayer::run: Now subscribing to scan data.", beVerboseHere);
		result = action_subscribeScanData();
		if (result == false)
		{
			// Fail
			printError("LdmrsSopasLayer::run: Failed to subscribe to scan data, aborting.");
			return false;
		}
		printInfoMessage("LdmrsSopasLayer::run: Subscription to scan data was successful.", beVerboseHere);
		
	}
	
	return true;
}

//
// Save all field-related (=SOPAS) parameters permanently.
// This function can be called from runlevel USER, so no login is required.
//
bool LdmrsSopasLayer::action_flashFieldParameters()
{
	bool beVerboseHere = m_beVerbose;
//	beVerboseHere = true;
	
	printInfoMessage("LdmrsSopasLayer::action_flashFieldParameters: Called.", beVerboseHere);

	if (isConnected() == false)
	{
		printWarning("LdmrsSopasLayer::action_flashFieldParameters: LD-MRS not connected - aborting.");
		return false;
	}

	bool result = false;
	SopasAnswer* answer = NULL;
	UINT8 parameterBuffer[128];
	UINT16 parameterBufferLen = 0;

	result = invokeMethod(index_meth_FlashFieldParameters, parameterBuffer, parameterBufferLen, answer);

	if ((result == true) && (answer != NULL) && (answer->isValid() == true) && (answer->size() > 0))
	{
		// Evaluate answer. Successful?  0 = false, 1 = true
		BYTE* buffer = answer->getBuffer();
		UINT8 success = buffer[0];	// ::memread_UINT8 (buffer);
		if (success == 0)
		{
			// 0 = Not successful
			m_isLoggedIn = false;
			result = false;
			printError("LdmrsSopasLayer::action_flashFieldParameters: FlashFieldPara command was not successful!");
		}
		else
		{
			// Success
			m_isLoggedIn = true;
			result = true;
			printInfoMessage("LdmrsSopasLayer::action_flashFieldParameters: Field parameters flashed successfully.", beVerboseHere);
		}
	}

	// free memory for answer
	if (answer != NULL)
	{
		delete answer;
		answer = NULL;
	}
	
	printInfoMessage("LdmrsSopasLayer::action_flashFieldParameters: All done, leaving.", beVerboseHere);
	return result;
}

//
// Save all scan-related (=LUX/MRS) parameters permanently.
// This function can be called from runlevel USER, so no login is required.
//
bool LdmrsSopasLayer::action_flashMrsParameters()
{
	bool beVerboseHere = m_beVerbose;
//	beVerboseHere = true;
	
	printInfoMessage("LdmrsSopasLayer::action_flashMrsParameters: Called.", beVerboseHere);

	if (isConnected() == false)
	{
		printWarning("LdmrsSopasLayer::action_flashMrsParameters: LD-MRS not connected - aborting.");
		return false;
	}

	bool result = false;
	SopasAnswer* answer = NULL;
	UINT8 parameterBuffer[128];
	UINT16 parameterBufferLen = 0;

	result = invokeMethod(index_meth_MthdFlashLUXParameters, parameterBuffer, parameterBufferLen, answer);

	if ((result == true) && (answer != NULL) && (answer->isValid() == true) && (answer->size() > 0))
	{
		// Evaluate answer. Successful?  0 = false, 1 = true
		BYTE* buffer = answer->getBuffer();
		UINT8 success = buffer[0];	// ::memread_UINT8 (buffer);
		if (success == 0)
		{
			// 0 = Not successful
			m_isLoggedIn = false;
			result = false;
			printError("LdmrsSopasLayer::action_flashMrsParameters: FlashLUXPara command was not successful!");
		}
		else
		{
			// Success
			m_isLoggedIn = true;
			result = true;
			printInfoMessage("LdmrsSopasLayer::action_flashMrsParameters: MRS parameters flashed successfully.", beVerboseHere);
		}
	}

	// free memory for answer
	if (answer != NULL)
	{
		delete answer;
	}

	printInfoMessage("LdmrsSopasLayer::action_flashMrsParameters: All done, leaving.", beVerboseHere);
	
	return result;
}


//
// Log in as "Authorized client".
//
// This is required to set parameters.
//
bool LdmrsSopasLayer::action_login()
{
	bool beVerboseHere = m_beVerbose;
//	beVerboseHere = true;
	
	// cola-B ByIndex
	printInfoMessage("LdmrsSopasLayer::action_login: Trying to log in as authorized client.", beVerboseHere);

	bool result = false;

	if (isConnected() == true)
	{
		const static UINT32 passwordHash_LDMRS = 0xF4724744; // Hash for "client"
		SopasAnswer* answer = NULL;

		INT8 level = 3; // 3 = AUTHORIZEDCLIENT
		UINT8 parameterBuffer[128];
		UINT16 parameterBufferLen = 0;

		colab::addIntegerToBuffer<INT8>(parameterBuffer, parameterBufferLen, level); 				// user level
		colab::addIntegerToBuffer<UINT32>(parameterBuffer, parameterBufferLen, passwordHash_LDMRS); // password hash

		result = invokeMethod(index_meth_SetAccessMode, parameterBuffer, parameterBufferLen, answer);

		if ((result == true) && (answer != NULL) && (answer->isValid() == true) && (answer->size() > 0))
		{
			// Evaluate answer. Login successful?  0 = false, 1 = true
			BYTE* buffer = answer->getBuffer();
			UINT8 success = buffer[0];	// ::memread_UINT8 (buffer);
			if (success == 0)
			{
				// 0 = Not successful
				m_isLoggedIn = false;
				result = false;
				printError("LdmrsSopasLayer::action_login: Login was not successful!");
			}
			else
			{
				// Success
				m_isLoggedIn = true;
				result = true;
				printInfoMessage("LdmrsSopasLayer::action_login: Login was successful.", beVerboseHere);
			}
		}

		// free memory for answer
		if (answer != NULL)
		{
			delete answer;
		}
	}
	else
	{
		printWarning("LdmrsSopasLayer::action_login: LD-MRS not connected - cannot login.");
	}

	return result;
}


//
// Logout.
//
bool LdmrsSopasLayer::action_logout()
{
	// cola-B ByIndex
	bool beVerboseHere = m_beVerbose;
//	beVerboseHere = true;
	
	// cola-B ByIndex
	printInfoMessage("LdmrsSopasLayer::action_logout: Logging out now.", beVerboseHere);

	bool result = false;

	if (isConnected() == true)
	{
		const static UINT32 passwordHash_LDMRS = 0x00000000; // Not needed for LOGOUT
		SopasAnswer* answer = NULL;

		INT8 level = 1; // 1 = Logout, 3 = AUTHORIZEDCLIENT
		UINT8 parameterBuffer[128];
		UINT16 parameterBufferLen = 0;

		colab::addIntegerToBuffer<INT8>(parameterBuffer, parameterBufferLen, level); 				// user level
		colab::addIntegerToBuffer<UINT32>(parameterBuffer, parameterBufferLen, passwordHash_LDMRS); // password hash

		result = invokeMethod(index_meth_SetAccessMode, parameterBuffer, parameterBufferLen, answer);

		if ((result == true) && (answer != NULL) && (answer->isValid() == true) && (answer->size() > 0))
		{
			// Evaluate answer. Logout successful?  0 = false, 1 = true
			BYTE* buffer = answer->getBuffer();
			UINT8 success = buffer[0];	// ::memread_UINT8 (buffer);
			if (success == 0)
			{
				// 0 = Not successful
				result = false;
				printError("LdmrsSopasLayer::action_logout: Logout was not successful!");
			}
			else
			{
				// Success
				m_isLoggedIn = false;
				result = true;
				printInfoMessage("LdmrsSopasLayer::action_logout: Logout was successful.", beVerboseHere);
			}
		}

		// free memory for answer
		if (answer != NULL)
		{
			delete answer;
		}
	}
	else
	{
		printWarning("LdmrsSopasLayer::action_logout: LD-MRS not connected - cannot log out, aborting.");
	}

	return result;
}


//
// Subscribe to EvalCase results. After the subscription, the scanner will send EvalCaseResults whenever
// there is a change in the EvalCase status.
//
bool LdmrsSopasLayer::action_subscribeEvalCaseResults()
{
	// cola-B ByIndex
	bool result = false;

	if (isConnected() == false)
	{
		printError("LdmrsSopasLayer::action_subscribeEvalCaseResults: LD-MRS not connected, aborting!");
		return false;
	}
	
	result = registerEvent((UINT16)(index_event_aEvalCaseResult));

	if (result == true)
	{
		m_fieldEventIsRegistered = true;
	}
	else
	{
		printError("LdmrsSopasLayer::action_subscribeEvalCaseResults: Failed to subscribe, aborting!");
	}

	return result;
}

//
// Un-subscribe from the EvalCase results.
//
bool LdmrsSopasLayer::action_unSubscribeEvalCaseResults()
{
	// cola-B ByIndex
	bool result = unregisterEvent((UINT16)(index_event_aEvalCaseResult));

	if (result == true)
	{
		m_fieldEventIsRegistered = false;
	}
	else
	{
		printError("LdmrsSopasLayer::action_unSubscribeEvalCases: Failed to un-subscribe, aborting!");
	}
	
	return result;
}



//
// Subscribe to scan data. After the subscription, the scanner will send new scan data whenever
// a scan is ready.
//
bool LdmrsSopasLayer::action_subscribeScanData()
{
	bool beVerboseHere = m_beVerbose;
//	beVerboseHere = true;
	printInfoMessage("LdmrsSopasLayer::action_subscribeScanData: Called.", beVerboseHere);
	
	// cola-B ByIndex
	bool result = false;

	if (isConnected() == false)
	{
		printError("LdmrsSopasLayer::action_subscribeScanData: LD-MRS not connected, aborting!");
		return false;
	}
	
	result = registerEvent((UINT16)(index_event_ScanDataMonitor));

	if (result == true)
	{
		m_scanEventIsRegistered = true;
	}
	else
	{
		printError("LdmrsSopasLayer::action_subscribeScanData: Failed to subscribe, aborting!");
	}

	printInfoMessage("LdmrsSopasLayer::action_subscribeScanData: Done, leaving.", beVerboseHere);
	return result;
}


//
// Un-subscribe from the scan data.
//
bool LdmrsSopasLayer::action_unSubscribeScanData()
{
	// cola-B ByIndex
	bool result = unregisterEvent((UINT16)(index_event_ScanDataMonitor));

	if (result == true)
	{
		m_scanEventIsRegistered = false;
	}
	else
	{
		printError("LdmrsSopasLayer::action_unSubscribeScanData: Failed to un-subscribe, aborting!");
	}
	
	return result;
}




bool LdmrsSopasLayer::action_readScanConfig()
{
	SopasAnswer* answer = NULL;
	bool success = readVariable(index_var_ScanConfig, answer);

	if (success && answer != NULL && answer->size() > 0)
	{
		// decode scanconfig
		BYTE* bufferPos = answer->getBuffer();
		ScanFreqEnum scanFreqEnum = ScanFreqEnum(memread_UINT8(bufferPos));

		switch (scanFreqEnum)
		{
		case ScanFreq1250:
			m_scanFreq = 12.5;
			break;
		case ScanFreq2500:
			m_scanFreq = 25.;
			break;
		case ScanFreq5000:
			m_scanFreq = 50.;
			break;
		}

		UINT16 length = memread_UINT16(bufferPos);

		if (length > 0)
		{
			m_angleResolution = angleToRad(memread_INT16(bufferPos));
			m_scanStartAngle = angleToRad(memread_INT16(bufferPos));
			m_scanEndAngle = angleToRad(memread_INT16(bufferPos));
		}
	}

	if (answer != NULL)
	{
		delete answer;
	}

	return success;
}

//
// Read the current eval cases.
// Note that this is 'just' the configuration of the eval cases, not their current evaluation status,
// which is the EvalCaseResult.
//
bool LdmrsSopasLayer::action_readEvalCases()
{
	SopasAnswer* answer = NULL;
	bool success = readVariable(index_var_evalCaseParam, answer);

	if (success)
	{
		m_evalCases = colaB_evalCaseDecoder(answer);
	}

	if (answer != NULL)
	{
		delete answer;
	}

	return success;
}

//
// Decode the configured eval cases.
//
EvalCases LdmrsSopasLayer::colaB_evalCaseDecoder(SopasAnswer* answer)
{
	bool beVerboseHere = m_beVerbose;
//	beVerboseHere = true;
	
	printInfoMessage("LdmrsSopasLayer::colaB_evalCaseDecoder: Called.", beVerboseHere);
	
	EvalCases evalCases;

	if (answer != NULL && answer->size() > 0)
	{
		// decode answer
		BYTE* bufferPos = answer->getBuffer();

		UINT16 arrayLength = memread_UINT16(bufferPos);

		for (UINT16 i = 0; i < arrayLength; ++i)
		{
			EvalCase_ptr evalCasePtr(new EvalCase);
			EvalCase& evalCase = *evalCasePtr; // just for easier access

			evalCase.setVersionNumber(memread_UINT16(bufferPos));
			evalCase.setCaseNumber(memread_UINT8(bufferPos));
			evalCase.setStrategy(EvalCase::EvaluationStrategy(memread_UINT8(bufferPos)));
			evalCase.setResultNegation(memread_UINT8(bufferPos) != 0);
			evalCase.setResponseTime(memread_UINT32(bufferPos));
			evalCase.setResponseTimeExtended(memread_UINT32(bufferPos));
			evalCase.setOutputNumber(memread_UINT8(bufferPos));
			// EnumX
			// no hardware inputs available
//			evalCase.setHardwareInputs(EvalCaseParameter::inputState_from_UINT8(memread_UINT8(bufferPos), 2));
//			memread_UINT8(bufferPos); // reserved
			UINT8 inputs = memread_UINT8(bufferPos);
			evalCase.setLogicalInputState_from_UINT8(inputs);
			memread_UINT8(bufferPos); // reserved
			evalCase.setDistDependent(memread_UINT8(bufferPos) != 0);
			evalCase.setMaxRadialCorridor(double(memread_UINT16(bufferPos)) / 1000. ); // conversion from [mm] to [m]
			evalCase.setManipulationPrevention(EvalCase::ManipulationPrevention(memread_UINT8(bufferPos)));
			evalCase.setBlankingSize(double(memread_UINT16(bufferPos)) / 1000.); // conversion from [mm] to [m]
			evalCase.setMinFieldExp(double(memread_UINT16(bufferPos)) / 1000.); // conversion from [mm] to [m]
			evalCase.setFieldNumber(memread_UINT8(bufferPos));
			evalCase.setFilterType(EvalCase::FilterType(memread_UINT8(bufferPos)));

			UINT16 nameLength = memread_UINT16(bufferPos);

			std::string name = std::string((char *)bufferPos, nameLength);
			bufferPos += nameLength;
			evalCase.setCaseName(name);
			printInfoMessage("LdmrsSopasLayer::colaB_evalCaseDecoder: Decoding EvalCase with the name " +
								name + ". Inputs: " + ::toHexString(inputs) + ".", beVerboseHere);

			UINT16 commentLength = memread_UINT16(bufferPos);
			std::string comment = std::string((char*)bufferPos, commentLength);
			bufferPos += commentLength,
						 evalCase.setComment(comment);

			evalCases.add(evalCasePtr);
		}
	}
	
	printInfoMessage("LdmrsSopasLayer::colaB_evalCaseDecoder: Done. Decoded " +
						::toString(evalCases.getEvalCases().size()) + " cases.", beVerboseHere);
	
	return evalCases;
}

//
// Encode the given eval cases.
// Returns the number of bytes that have been written into the buffer.
//
UINT32 LdmrsSopasLayer::colaB_evalCaseEncoder(BYTE* buffer, const EvalCases& evalCases)
{
	bool beVerboseHere = m_beVerbose;
//	beVerboseHere = true;
	
	printInfoMessage("LdmrsSopasLayer::colaB_evalCaseEncoder: Called.", m_beVerbose);
	
	//
	// Encode the cases
	//
	BYTE* bufferPos = buffer;

	// Number of eval cases
	UINT16 arrayLength = evalCases.getEvalCases().size();
	memwrite_UINT16(bufferPos, arrayLength);

	// The cases
	for (UINT16 i = 0; i < arrayLength; i++)
	{
		EvalCase* evalCase = evalCases.getEvalCases().at(i);

		printInfoMessage("LdmrsSopasLayer::colaB_evalCaseEncoder: Encoding EvalCase with the name " +
							evalCase->getCaseName() + ".", beVerboseHere);
		
		// Version number
		UINT16 versionNumber = evalCase->getVersionNumber();
		memwrite_UINT16(bufferPos, versionNumber);
	
		// Case number
		UINT8 caseNumber = evalCase->getCaseNumber();
		memwrite_UINT8(bufferPos, caseNumber);
		
		// Eval strategy
		UINT8 evalStrategy = (UINT8)(evalCase->getStrategy());
		memwrite_UINT8(bufferPos, evalStrategy);
		
		// Result negation
		bool resultNegation = evalCase->getResultNegation();
		if (resultNegation == true)
		{
			memwrite_UINT8(bufferPos, 1);
		}
		else
		{
			memwrite_UINT8(bufferPos, 0);
		}

		// Response time
		UINT32 responseTime = evalCase->getResponseTime();
		memwrite_UINT32(bufferPos, responseTime);

		// Response time extended
		UINT32 responseTimeExtended = evalCase->getResponseTimeExtended();
		memwrite_UINT32(bufferPos, responseTimeExtended);

		// Output number
		UINT8 outputNumber = evalCase->getOutputNumber();
		memwrite_UINT8(bufferPos, outputNumber);
		
		// Assigned inputs
		UINT8 logicalInputs = evalCase->getLogicalInputState_as_UINT8();
		memwrite_UINT8(bufferPos, logicalInputs);
		
		memread_UINT8(bufferPos); // reserved
		
		// DistDependent
		bool distDependent = evalCase->getDistDependent();
		if (distDependent == true)
		{
			memwrite_UINT8(bufferPos, 1);
		}
		else
		{
			memwrite_UINT8(bufferPos, 0);
		}
		
		// RadialCorridor
		UINT16 maxRadialCorridor = (UINT16)((evalCase->getMaxRadialCorridor() * 1000.0) + 0.5);
		memwrite_UINT16(bufferPos, maxRadialCorridor);
		
		// Sensitivity
		UINT8 manipulationPrevention = (UINT8)(evalCase->getManipulationPrevention());	// Effectively boolean
		memwrite_UINT8(bufferPos, manipulationPrevention);
		
		// Blanking size
		UINT16 blankingSize = (UINT16)((evalCase->getBlankingSize() * 1000.0) + 0.5);
		memwrite_UINT16(bufferPos, blankingSize);
		
		// MinFieldExp
		UINT16 minFieldExp = (UINT16)((evalCase->getMinFieldExp() * 1000.0) + 0.5);
		memwrite_UINT16(bufferPos, minFieldExp);

		// Field number (of assigned field)
		UINT8 fieldNumber = evalCase->getFieldNumber();
		memwrite_UINT8(bufferPos, fieldNumber);

		// Filter type
		UINT8 filterType = (UINT8)(evalCase->getFilterType());
		memwrite_UINT8(bufferPos, filterType);

		// Name length + name
		std::string name = evalCase->getCaseName();
		memwrite_UINT16(bufferPos, name.length());
		memwrite_string(bufferPos, name);
		
		// Comment length + comment
		std::string comment = evalCase->getComment();
		memwrite_UINT16(bufferPos, comment.length());
		memwrite_string(bufferPos, comment);
	}
	
	// How many bytes have been used?
	UINT32 len = (UINT32)((UINT64)bufferPos - (UINT64)buffer);
	
	printInfoMessage("LdmrsSopasLayer::colaB_evalCaseEncoder: Done. Encoded " +
						::toString(evalCases.getEvalCases().size()) + " cases. Used " +
						::toString(len) + " bytes.", beVerboseHere);
	
	return len;
}


//
// Write a single field.
// fieldNum = Number of the field to be written (0..15).
//
// Must be logged in as AuthorizedClient in order to do this.
//
bool LdmrsSopasLayer::action_writeField(UINT16 fieldNum, const FieldParameter& para)
{
	bool beVerboseHere = m_beVerbose;
//	beVerboseHere = true;
	
	printInfoMessage("LdmrsSopasLayer::action_writeField: Called for field " + toString(fieldNum) + ".", beVerboseHere);
	bool result = false;
	
	if (fieldNum > 15)
	{
		printError("LdmrsSopasLayer::action_writeField: FieldNum must be 0..15, but is " + toString(fieldNum) + ", aborting!");
		return false;
	}
	
	// Build the internal data structure
	UINT32 bufferSize = 1024;
	UINT16 usedBufferLen = 0;
	BYTE* buffer = new BYTE[bufferSize];
	usedBufferLen = colaB_fieldEncoder(buffer, para);
	
	// Write the variable
	result = writeVariable(index_var_field000 + fieldNum, buffer, usedBufferLen);

	if (result == false)
	{
		// Failure
		printError("LdmrsSopasLayer::action_writeField: Failed to write field " + toString(fieldNum) + ", aborting!");
		return false;
	}
	
	// All done
	printInfoMessage("LdmrsSopasLayer::action_writeField: All done, leaving.", beVerboseHere);
	return result;
}


//
// Write all eval cases.
//
// Must be logged in as AuthorizedClient in order to do this.
//
bool LdmrsSopasLayer::action_writeEvalCases(const EvalCases& evalCases)
{
	bool beVerboseHere = m_beVerbose;
//	beVerboseHere = true;
	
	printInfoMessage("LdmrsSopasLayer::action_writeEvalCases: Called, with " + toString(evalCases.getEvalCases().size()) + " eval cases.", beVerboseHere);
	bool result = false;
	
	if (evalCases.getEvalCases().size() > 16)
	{
		printError("LdmrsSopasLayer::action_writeEvalCases: The MRS can only handle up to 16 eval cases, aborting!");
		return false;
	}
	
	// Build the internal data structure
	UINT32 bufferSize = 1024;
	UINT16 usedBufferLen = 0;
	BYTE* buffer = new BYTE[bufferSize];
	usedBufferLen = colaB_evalCaseEncoder(buffer, evalCases);
	
	// Write the variable
	result = writeVariable(index_var_evalCaseParam, buffer, usedBufferLen);

	if (result == false)
	{
		// Failure
		printError("LdmrsSopasLayer::action_writeEvalCases: Failed to write eval cases, aborting!");
		return false;
	}
	
	// All done
	printInfoMessage("LdmrsSopasLayer::action_writeEvalCases: All done, leaving.", beVerboseHere);
	return result;
}


//
// Read the current parameters of all fields.
// The MRS always has 16 fields.
//
bool LdmrsSopasLayer::action_readFields()
{
	SopasAnswer* answer = NULL;
	bool success;
	
	// Find out how many fields we've got
	// Unfortunately, this does not work with the MRS. The reply is
	// always "16".
/*	success = readVariable(index_var_numOfParamFields, answer);
	UINT16 numOfParamFields = 0;

	if ((success == true) &&
		(answer != NULL) &&
		(answer->size() > 0))
	{
		// always 0 !!! ???
		BYTE* bufferPos = answer->getBuffer();
		numOfParamFields = memread_UINT16(bufferPos);

		printInfoMessage("LdmrsSopasLayer::action_readFields: NumOfParamFields=" + toString(numOfParamFields) + ".", m_beVerbose);
	}

	// Get rid of the answer
	if (answer != NULL)
	{
		delete answer;
	}
*/
	// Now read all possible fields
	// As the parameter "index_var_numOfParamFields" seems unreliable, read all (!) possible fields.
	for (UINT16 i = 0; i < MAX_NUM_OF_FIELDS; ++i)
	{
		printInfoMessage("LdmrsSopasLayer::action_readFields: Reading field " + toString(i) + ".", true);
		success = readVariable(index_var_field000 + i, answer);

		if ((success == true) &&
			(answer != NULL) &&
			(answer->size() > 0))
		{
			FieldParameter* fieldPtr = colaB_fieldDecoder(answer);

			// handle dummy field answer: no field will be set
			if (fieldPtr != NULL)
			{
				m_fields.add(fieldPtr);
			}
		}

		if (answer != NULL)
		{
			delete answer;
			answer = NULL;
		}
	}

	return success;
}

/// computes an angle in [rad] from INT32 as 1/10000 deg in scanner coordinate system
double LdmrsSopasLayer::angleToRad(INT32 angle)
{
	return ((double)angle / 32.0) * deg2rad;
}

//
// Decode an incoming Field strucure. This is the parameter set for one of the 16 possible
// fields.
//
// Note: A field with field number = 0 is invalid.
//
FieldParameter* LdmrsSopasLayer::colaB_fieldDecoder(SopasAnswer* answer)
{
	bool beVerboseHere = m_beVerbose;
//	beVerboseHere = true;
	
	
	FieldParameter* fieldPtr(new FieldParameter);

	if ((answer != NULL) && (answer->size() > 0))
	{
		FieldParameter& field = *fieldPtr; // just for easier access
		BYTE* bufferPos = answer->getBuffer();

		field.setDistScaleFactor(memread_float(bufferPos));		// [mm per step]
		field.setDistScaleOffset(memread_float(bufferPos));		// [mm]
		field.setAngleScaleFactor(memread_UINT32(bufferPos));	// 1/32°
		field.setAngleScaleOffset(memread_INT32(bufferPos));	// 1/32°
		field.setFieldTypeIntern(memread_UINT8(bufferPos));		// Field type (Rectangular or Segmented)
		field.setFieldNumber(memread_UINT8(bufferPos));

		printInfoMessage("LdmrsSopasLayer::colaB_fieldDecoder: setDistScaleFactor: " + toString(field.getDistScaleFactor(), 2) + ".", beVerboseHere);
		printInfoMessage("LdmrsSopasLayer::colaB_fieldDecoder: setDistScaleOffset: " + toString(field.getDistScaleOffset(), 2) + ".", beVerboseHere);
		printInfoMessage("LdmrsSopasLayer::colaB_fieldDecoder: setAngleScaleFactor: " + toString(field.getAngleScaleFactor()) + ".", beVerboseHere);
		printInfoMessage("LdmrsSopasLayer::colaB_fieldDecoder: setAngleScaleOffset: " + toString(field.getAngleScaleOffset()) + ".", beVerboseHere);
		printInfoMessage("LdmrsSopasLayer::colaB_fieldDecoder: setFieldTypeIntern: " + toString((INT32)(field.getFieldTypeIntern())) + ".", beVerboseHere);
		printInfoMessage("LdmrsSopasLayer::colaB_fieldDecoder: Field number: " + toString(field.getFieldNumber()) + ".", beVerboseHere);

		// Segmented field
		UINT16 fieldSeg = memread_UINT16(bufferPos);
		if (fieldSeg == 1)
		{
			printInfoMessage("LdmrsSopasLayer::colaB_fieldDecoder: Found segmented field.", beVerboseHere);

			UINT8 arrayLength = memread_UINT16(bufferPos);
			FieldSegmented* seg(new FieldSegmented);
			
			printInfoMessage("LdmrsSopasLayer::colaB_fieldDecoder: ArrayLength=" + toString(arrayLength) + ".", beVerboseHere);

			// iterate through points to fill the polygon in the correct order
			for (UINT8 i = 0; i < arrayLength; ++i)
			{
				UINT16 angleIdx = memread_UINT16(bufferPos);
				UINT16 startDist = memread_UINT16(bufferPos);
				UINT16 endDist = memread_UINT16(bufferPos);
				
				printInfoMessage("LdmrsSopasLayer::colaB_fieldDecoder: Point " + toString(i) + ": AngIdx=" +
								toString(angleIdx) + ", StartDist=" + toString(startDist) + ", EndDist=" +
								toString(endDist) + ".", beVerboseHere);

				// start dist not valid if 65535
				double s = (startDist == 65535) ? ::NaN_double : ((double)startDist / 1000.) * field.getDistScaleFactor() + (field.getDistScaleOffset() / 1000.); // from [mm] to [m]
				printInfoMessage("LdmrsSopasLayer::colaB_fieldDecoder: StartDist is " + toString(s, 2) + ".", beVerboseHere);

				// transform to 1/32° tics
				double angle = field.getAngleScaleOffset() + (double)angleIdx * field.getAngleScaleFactor();
				// Quick hack: The LD-MRS treats the angle direction differently as expected and defined in DIN70000. Therefore, the
				// sign has to be reversed here.
				angle = -angle;
				printInfoMessage("LdmrsSopasLayer::colaB_fieldDecoder: Angle is " + toString(angle, 2) + ".", beVerboseHere);

				// end dist not valid if 65535
				double e = (endDist == 65535) ? ::NaN_double : ((double)endDist / 1000.) * field.getDistScaleFactor() + (field.getDistScaleOffset() / 1000.); // from [mm] to [m]
				printInfoMessage("LdmrsSopasLayer::colaB_fieldDecoder: EndDist is " + toString(e, 2) + ".", beVerboseHere);

				seg->addPoint(FieldSegmentedPoint(angleToRad(angle), s, e));
			}
			seg->computePolygon();

			field.setField((FieldDescription*)(seg));
		}

		// Rectangular field
		UINT16 fieldRect = memread_UINT16(bufferPos);
		if (fieldRect == 1)
		{
			printInfoMessage("LdmrsSopasLayer::colaB_fieldDecoder: Found rectangular field.", beVerboseHere);

			FieldRectangle* rect(new FieldRectangle);
			INT32 refPointAngle = memread_INT32(bufferPos);
			// Quick hack: The LD-MRS treats the angle direction differently as expected. Therefore, the
			// sign has to be reversed here.
			refPointAngle = -refPointAngle;
			UINT32 refPointDist = (memread_UINT16(bufferPos) * field.getDistScaleFactor()) + field.getDistScaleOffset();
			INT32 rotAngle = memread_INT32(bufferPos);
			// Quick hack: The LD-MRS treats the angle direction differently as expected. Therefore, the
			// sign has to be reversed here.
			rotAngle = -rotAngle;
			UINT32 length = memread_UINT32(bufferPos); // width and length changed due to other coordiante system in LD-MRS!!!
			UINT32 width = memread_UINT32(bufferPos);

			printInfoMessage("LdmrsSopasLayer::colaB_fieldDecoder: refPointDist is " + toString(refPointDist) + ".", beVerboseHere);
			printInfoMessage("LdmrsSopasLayer::colaB_fieldDecoder: refPointAngle is " + toString(-refPointAngle) + ".", beVerboseHere);
			printInfoMessage("LdmrsSopasLayer::colaB_fieldDecoder: rotAngle is " + toString(rotAngle, 2) + ".", beVerboseHere);
			printInfoMessage("LdmrsSopasLayer::colaB_fieldDecoder: length is " + toString(length, 2) + ".", beVerboseHere);
			printInfoMessage("LdmrsSopasLayer::colaB_fieldDecoder: width is " + toString(width, 2) + ".", beVerboseHere);

			// convert to [m] and [rad]
			rect->setRefPointAngle(angleToRad(refPointAngle)); // 1/32°
			rect->setRefPointDist((double)refPointDist / 1000.);
			rect->setRotAngle(angleToRad(rotAngle));
			rect->setWidth((double) width / 1000.);
			rect->setLength((double) length / 1000.);

			rect->computePolygon();

			field.setField((FieldDescription*)(rect));
		}

		// Radial field
		UINT16 fieldRad = memread_UINT16(bufferPos);
		if (fieldRad == 1)
		{
			printError("LdmrsSopasLayer::colaB_fieldDecoder: Found radial field, but the LD-MRS does not support radial fields!");
			return NULL;
		}

		// Dynamic field
		UINT16 fieldDyn = memread_UINT16(bufferPos);
		if (fieldDyn == 1)
		{
			printError("LdmrsSopasLayer::colaB_fieldDecoder: Found dynamic field, but the LD-MRS does not support dynamic fields!");
			return NULL;
		}

		// The rest of the data
		UINT16 versionNumber = memread_UINT16(bufferPos);
		field.setVersionNumber(versionNumber);
		printInfoMessage("LdmrsSopasLayer::colaB_fieldDecoder: Field version number= " + toString(versionNumber) + ".", beVerboseHere);
		UINT16 fieldNameLength = memread_UINT16(bufferPos);

		if (fieldNameLength > 0)
		{
			std::string fieldName = colab::getStringFromBuffer(bufferPos, fieldNameLength);
			field.setFieldName(fieldName);

			printInfoMessage("LdmrsSopasLayer::colaB_fieldDecoder: Field name= " + fieldName + ".", beVerboseHere);
		}

		UINT16 commentLength = memread_UINT16(bufferPos);

		if (commentLength > 0)
		{
			std::string comment = colab::getStringFromBuffer(bufferPos, commentLength);
			field.setComment(comment);
		}

		// Rest of information not needed and therefore not decoded
		// - bool enable layer filter
		// - layer bit field
	}

	return fieldPtr;
}


//
// Decodes incoming scans.
//
// Note that the created scans are not sorted by (increasing) hporizontal scanpoint angle. Instead, the scanpoints
// are decoded into the scan structure in the order in which they are transmitted in the message. If a sorted
// scan is required, this has to be done.
//
void LdmrsSopasLayer::scanDataDecoder(SopasEventMessage& frame)
{
	bool beVerboseHere = m_beVerbose;
	beVerboseHere = true;	// false;
	
	// cola-B by index
	printInfoMessage("LdmrsSopasLayer::scanDataDecoder: Called. Decoding SOPAS frame with length=" +
						::toString(frame.size()) + " bytes.", beVerboseHere);

	// Decode this data only if we want the results
	if (m_weWantScanData == false)
	{
		// No scan data wanted, so ignore this message.
		return;
	}

	std::string receivedName;
	UINT32 receiveIndex;
	BYTE* bufferPos;

	printInfoMessage("LdmrsSopasLayer::scanDataDecoder: Reading variable index.", beVerboseHere);
	receiveIndex = frame.getVariableIndex(); // byte number 12 13

	if (receiveIndex != index_event_ScanDataMonitor)
	{
		printError("LdmrsSopasLayer::scanDataDecoder: This is not a scan data message, aborting!");
		return;
	}
	else
	{
		printInfoMessage("LdmrsSopasLayer::scanDataDecoder: This is a scan data message.", beVerboseHere);
	}
	
	// Obviously scan data events are registered. This is just for global information.
	m_scanEventIsRegistered = true;

	bufferPos = &(frame.getPayLoad()[5]); // pay load without "sSI + index (2 byte)"
	UINT16 versionNumber = memread_UINT16(bufferPos);
	printInfoMessage("LdmrsSopasLayer::scanDataDecoder: Version number=" + ::toString(versionNumber) + ".", beVerboseHere);

	// Device block
	UINT16 logicalNumber = memread_UINT16(bufferPos);
	printInfoMessage("LdmrsSopasLayer::scanDataDecoder: Logical number of the device=" + ::toString(logicalNumber) + ".", beVerboseHere);
	UINT32 serialNumber = memread_UINT32(bufferPos);
	printInfoMessage("LdmrsSopasLayer::scanDataDecoder: Serial number of the device=" + ::toString(serialNumber) + ".", beVerboseHere);
	UINT16 statusBits = memread_UINT16(bufferPos);
	printInfoMessage("LdmrsSopasLayer::scanDataDecoder: Status bits=" + ::toString(statusBits) + ".", beVerboseHere);
	
	// Status block
	UINT16 telegramCount = memread_UINT16(bufferPos);
	printInfoMessage("LdmrsSopasLayer::scanDataDecoder: Telegram count=" + ::toString(telegramCount) + ".", beVerboseHere);
	UINT16 scanCount = memread_UINT16(bufferPos);
	printInfoMessage("LdmrsSopasLayer::scanDataDecoder: Scan count=" + ::toString(scanCount) + ".", beVerboseHere);
	UINT32 systemTimeScan = memread_UINT32(bufferPos);
	printInfoMessage("LdmrsSopasLayer::scanDataDecoder: System time scan=" + ::toString(systemTimeScan) + ".", beVerboseHere);
	UINT32 systemTimeTransmit = memread_UINT32(bufferPos);
	printInfoMessage("LdmrsSopasLayer::scanDataDecoder: System time transmit=" + ::toString(systemTimeTransmit) + ".", beVerboseHere);
	UINT16 inputState = memread_UINT16(bufferPos);
	printInfoMessage("LdmrsSopasLayer::scanDataDecoder: Input state=" + ::toString(inputState) + ".", beVerboseHere);
	UINT16 outputState = memread_UINT16(bufferPos);
	printInfoMessage("LdmrsSopasLayer::scanDataDecoder: Output state=" + ::toString(outputState) + ".", beVerboseHere);
	UINT16 reserved_1 = memread_UINT16(bufferPos);
	printInfoMessage("LdmrsSopasLayer::scanDataDecoder: Reserved_1=" + ::toString(reserved_1) + ".", beVerboseHere);
	UINT32 scanFrequency = memread_UINT32(bufferPos);	// [1/256 Hz]
	printInfoMessage("LdmrsSopasLayer::scanDataDecoder: Scan frequency=" + ::doubleToString(double(scanFrequency) / 256.0, 2) + " Hz.", beVerboseHere);
	UINT32 measurementFrequency = memread_UINT32(bufferPos);	// [100 Hz]
	printInfoMessage("LdmrsSopasLayer::scanDataDecoder: Measurement frequency=" + ::doubleToString(double(measurementFrequency) / 10.0, 1) + " kHz.", beVerboseHere);
	
	// Encoder block (Always 0 as the MRS has no encoder inputs)
	UINT16 numOfEncoders = memread_UINT16(bufferPos);
	printInfoMessage("LdmrsSopasLayer::scanDataDecoder: Number of encoder blocks=" + ::toString(numOfEncoders) + ".", beVerboseHere);
	for (UINT16 i = 0; i<numOfEncoders; i++)
	{
		UINT32 encoderPos = memread_UINT32(bufferPos);	// [inc]
		printInfoMessage("LdmrsSopasLayer::scanDataDecoder: Encoder position=" + ::toString(encoderPos) + " increments.", beVerboseHere);
		INT16 encoderSpeed = memread_INT16(bufferPos);	// [inc/mm]
		printInfoMessage("LdmrsSopasLayer::scanDataDecoder: Encoder speed=" + ::toString(encoderSpeed) + " inc/mm.", beVerboseHere);
	}
	
	// 16-bit data channels
	UINT16 numOf16BitDataChannels = memread_UINT16(bufferPos);
	printInfoMessage("LdmrsSopasLayer::scanDataDecoder: Number of 16-bit data channels=" + ::toString(numOf16BitDataChannels) + ".", beVerboseHere);
	
	// Setup of the vertical layer angles. Only valid for standard "4-layer" devices,
	// so-called "8-layer" devices have different angles!
	double vAngles[4];
	vAngles[0] = -1.6 * 0.75 * deg2rad;
	vAngles[1] = -1.6 * 0.25 * deg2rad;
	vAngles[2] = 1.6 * 0.25 * deg2rad;
	vAngles[3] = 1.6 * 0.75 * deg2rad;
	
	// Create the new scan
	Scan* scan = new Scan;
	// Decode the range data
	for (UINT16 i = 0; i<numOf16BitDataChannels; i++)
	{
		// Data channel header
		std::string contentType = memread_string(bufferPos, 6);
		printInfoMessage("LdmrsSopasLayer::scanDataDecoder: Content type=" + contentType + ".", beVerboseHere);
		float scaleFactor = memread_float(bufferPos);
		printInfoMessage("LdmrsSopasLayer::scanDataDecoder: Scale factor=" + ::doubleToString(double(scaleFactor), 10) + ".", beVerboseHere);
		float scaleOffset = memread_float(bufferPos);
		printInfoMessage("LdmrsSopasLayer::scanDataDecoder: Scale offset=" + ::doubleToString(double(scaleOffset), 10) + ".", beVerboseHere);
		INT32 startAngle = memread_INT32(bufferPos);			// [1/10000 deg]
		printInfoMessage("LdmrsSopasLayer::scanDataDecoder: Start angle=" + ::doubleToString(double(startAngle)/10000.0, 2) + ".", beVerboseHere);
		UINT32 angularResolution = memread_UINT16(bufferPos);	// [1/10000 deg]. Decoded value is UINT16, but needed as UINT32 later.
		printInfoMessage("LdmrsSopasLayer::scanDataDecoder: Angular resolution=" + ::doubleToString(double(angularResolution)/10000.0, 2) + ".", beVerboseHere);

		// Decode the channel header
		UINT16 layer = contentType.c_str()[4] - 'A';	// A, B, C or D --> 0..3
		UINT16 echo = contentType.c_str()[5] - '1';		// 1, 2 or 3 --> 0..2
		if (layer > 3)
		{
			printError("LdmrsSopasLayer::scanDataDecoder: We have decoded an invalid layer value of " + toString(layer) + ", allowed is 0..3.");
			return;
		}
		if (echo > 2)
		{
			printError("LdmrsSopasLayer::scanDataDecoder: We have decoded an invalid echo value of " + toString(echo) + ", allowed is 0..2.");
			return;
		}
		
		// The data
		UINT16 numOf16BitData = memread_UINT16(bufferPos);
		printInfoMessage("LdmrsSopasLayer::scanDataDecoder: Number of 16-bit data=" + ::toString(numOf16BitData) + ".", beVerboseHere);
		for (UINT16 d = 0; d<numOf16BitData; d++)
		{
			//	Read the point
			UINT16 data = memread_UINT16(bufferPos);
//				printInfoMessage("LdmrsSopasLayer::scanDataDecoder: Data=" + ::toString(data) + ".", beVerboseHere);
			
			// Add the point only if it is valid.
			if ((data > 0) && (data != 0xFFFF))
			{
				// Create the new point structure in the scan, and fill it with the values
				ScanPoint& newPoint = scan->addNewPoint();

				// Horizontal angle
				double hAngle = ((startAngle + d * angularResolution) / 10000.0) * deg2rad;	// hAngle is in [rad]

				// Radial distance
				double dist = (double(data) * scaleFactor) / 1000.0;	// dist is in [m]

				// Set the dist and angle values
				newPoint.setPolar (dist, hAngle, vAngles[layer]);

				// Copy data to new scan point
				newPoint.setEchoWidth (0.0);
				newPoint.setFlags (0);
				newPoint.setSourceId (m_deviceId);
				newPoint.setLayer (layer);
				newPoint.setEchoNum (echo); // 0 or 1 or ...
			}
		}
	}
	
	// 8-bit data
	UINT16 numOf8BitDataChannels = memread_UINT16(bufferPos);
	printInfoMessage("LdmrsSopasLayer::scanDataDecoder: Number of 8-bit data channels=" + ::toString(numOf8BitDataChannels) + ".", beVerboseHere);

	// Position block (scanner coordinates)
	UINT16 numOfPositionBlocks = memread_UINT16(bufferPos);
	printInfoMessage("LdmrsSopasLayer::scanDataDecoder: Number of position blocks=" + ::toString(numOfPositionBlocks) + ".", beVerboseHere);
	if (numOfPositionBlocks == 1)
	{
		float posX = memread_float(bufferPos);
		printInfoMessage("LdmrsSopasLayer::scanDataDecoder: Position X=" + ::doubleToString(double(posX), 2) + ".", beVerboseHere);
		float posY = memread_float(bufferPos);
		printInfoMessage("LdmrsSopasLayer::scanDataDecoder: Position Y=" + ::doubleToString(double(posY), 2) + ".", beVerboseHere);
		float posZ = memread_float(bufferPos);
		printInfoMessage("LdmrsSopasLayer::scanDataDecoder: Position Z=" + ::doubleToString(double(posZ), 2) + ".", beVerboseHere);
		float rotX = memread_float(bufferPos);	// roll angle
		printInfoMessage("LdmrsSopasLayer::scanDataDecoder: Rot angle X=" + ::doubleToString(double(rotX), 2) + ".", beVerboseHere);
		float rotY = memread_float(bufferPos);
		printInfoMessage("LdmrsSopasLayer::scanDataDecoder: Rot angle Y=" + ::doubleToString(double(rotY), 2) + ".", beVerboseHere);
		float rotZ = memread_float(bufferPos);
		printInfoMessage("LdmrsSopasLayer::scanDataDecoder: Rot angle Z=" + ::doubleToString(double(rotZ), 2) + ".", beVerboseHere);
		UINT8 rotMode = memread_UINT8(bufferPos);
		printInfoMessage("LdmrsSopasLayer::scanDataDecoder: Rot mode=" + ::toString(rotMode) + ".", beVerboseHere);
	}
	
	// Device name
	UINT16 numOfDeviceNames = memread_UINT16(bufferPos);
	printInfoMessage("LdmrsSopasLayer::scanDataDecoder: Number of device names=" + ::toString(numOfDeviceNames) + ".", beVerboseHere);
	if (numOfDeviceNames == 1)
	{
		UINT16 deviceNameStringLen = memread_UINT16(bufferPos);
		printInfoMessage("LdmrsSopasLayer::scanDataDecoder: Device name string length=" + ::toString(deviceNameStringLen) + ".", beVerboseHere);
		if (deviceNameStringLen > 16)
		{
			printError("LdmrsSopasLayer::scanDataDecoder: We have decoded an invalid device name string length of " +
							toString(deviceNameStringLen) + ", allowed is 0..16.");
			return;
		}
		std::string deviceName = memread_string(bufferPos, deviceNameStringLen);
	}
	
	// Comment block
	UINT16 numOfCommentBlocks = memread_UINT16(bufferPos);
	printInfoMessage("LdmrsSopasLayer::scanDataDecoder: Number of comment blocks=" + ::toString(numOfCommentBlocks) + ".", beVerboseHere);
	if (numOfCommentBlocks == 1)
	{
		UINT16 commentStringLen = memread_UINT16(bufferPos);
		printInfoMessage("LdmrsSopasLayer::scanDataDecoder: Comment string length=" + ::toString(commentStringLen) + ".", beVerboseHere);
		if (commentStringLen > 128)
		{
			printError("LdmrsSopasLayer::scanDataDecoder: We have decoded an invalid comment string length of " +
							toString(commentStringLen) + ", allowed is 0..128.");
			return;
		}
		std::string commentString = memread_string(bufferPos, commentStringLen);
	}
	
	//
	// Ignore the rest of the data as the MRS does not fill it with any information.
	//

	
	//
	// Set some information about the scanner. As we have very little information,
	// fill most values with "0.0".
	// For more information, use the LuxBase-Scandata instead.
	//
	// Create Scanner Info
	ScannerInfo si;
	si.setStartAngle(m_scanStartAngle);
	si.setEndAngle(m_scanEndAngle);
//	printInfoMessage("LdmrsSopasLayer::scanDataDecoder: Scan start angle=" + ::doubleToString(scanStartAngle * rad2deg, 1) + ".", beVerboseHere);
//	printInfoMessage("LdmrsSopasLayer::scanDataDecoder: Scan end angle=" + ::doubleToString(scanEndAngle * rad2deg, 1) + ".", beVerboseHere);

	si.setScanFrequency(double(scanFrequency) / 256.0);
	si.setBeamTilt(0.0);					// For 8-layer devices, the beam tilt is different (either 1.2 or 1.6 deg)!
	si.setScanFlags(0);
	si.setScanNumber(scanCount);
	si.setDeviceID(m_deviceId);
	si.setScannerType(Sourcetype_LDMRS); // for compatibility, if no value is set in the scanner's config.
	Time start, end;
	start.set(double(systemTimeScan) / 1000000.0);													// No real information available
	end.set((double(systemTimeScan) / 1000000.0) + (1.0 / (double(scanFrequency) / 256.0)));		// No real information available
	si.setTimestamps(start, end);

	// Mounting position
	double yawAngle, pitchAngle, rollAngle, offsetX, offsetY, offsetZ;
	yawAngle = 0.0;	// No information available
	pitchAngle = 0.0;	// No information available
	rollAngle = 0.0;	// No information available
	offsetX = 0.0;	// No information available
	offsetY = 0.0;	// No information available
	offsetZ = 0.0;	// No information available
	Position3D mp(yawAngle, pitchAngle, rollAngle, offsetX, offsetY, offsetZ);
	si.setMountingPosition(mp);
	scan->setScannerInfos(Scan::ScannerInfoVector(1, si));

	// Post this scan to the world above...
	scan->setSourceId(m_deviceId);
	m_manager->setDeviceData(scan);

	// All done
	printInfoMessage("LdmrsSopasLayer::scanDataDecoder: Done, leaving.", beVerboseHere);
}


//
// Decodes incoming EvalCaseResult-Messages.
//
void LdmrsSopasLayer::evalCaseResultDecoder(SopasEventMessage& frame)
{
	// cola-B by index
	printInfoMessage("LdmrsSopasLayer::evalCaseResultDecoder: Called. Decoding SOPAS frame with length=" +
						::toString(frame.size()) + " bytes.", m_beVerbose);

	if (m_weWantFieldData == true)
	{
		std::string receivedName;
		UINT32 receiveIndex;
		BYTE* bufferPos;

		EvalCaseResults* evalCases;
		evalCases = new EvalCaseResults();
		evalCases->setSourceId(m_deviceId);

		printInfoMessage("LdmrsSopasLayer::evalCaseResultDecoder: Reading variable index.", m_beVerbose);
		receiveIndex = frame.getVariableIndex(); // byte number 12 13

		if (receiveIndex != index_event_aEvalCaseResult)
		{
			printError("LdmrsSopasLayer::evalCaseResultDecoder: This is not an eval case message, aborting!");
			return;
		}
		else
		{
			printInfoMessage("LdmrsSopasLayer::evalCaseResultDecoder: This is an eval case message.", m_beVerbose);
		}

		// Obviously field events are registered. This is just for global information.
		m_fieldEventIsRegistered = true;

		// Read the number of eval cases
		bufferPos = &(frame.getPayLoad()[5]); // pay load without "sSI + index (2 byte)"
		UINT16 cases = memread_UINT16 (bufferPos); // 14 15, array length in sopas is always decoded in uint16 !!!
		printInfoMessage("LdmrsSopasLayer::evalCaseResultDecoder: This frame contains " + ::toString(cases) + " cases to decode.", m_beVerbose);

		UINT16 length;
		for (UINT16 i = 0; i < cases; i++)
		{
			EvalCaseResult result;

			result.uiVersionNo = memread_UINT16 (bufferPos); // 16 17
			result.CaseHdr.usiNumber = memread_UINT8 (bufferPos); // 18
			result.CaseHdr.udiSysCount = memread_UINT32 (bufferPos); // 19 20 21 22
			result.CaseHdr.dDistScaleFactor = memread_float (bufferPos); // 23 24 25 26
			result.CaseHdr.dDistScaleOffset = memread_float (bufferPos); // 27 28 29 30
			result.CaseHdr.uiAngleScaleFactor = memread_UINT32 (bufferPos); // 31 32 33 34
			result.CaseHdr.iAngleScaleOffset = memread_INT32 (bufferPos); // 35 36 37 38
			UINT8 caseResultMRS = memread_UINT8 (bufferPos);
			EvalCaseResult::CaseResult caseResult;
			std::string resTxt;
			switch (caseResultMRS)
			{
			case 1:
				caseResult = EvalCaseResult::ECR_DONT_CARE;
				resTxt = "dont_care";
				break;
			case 2:
				// NOTE: LD-MRS signals always ECR_FALLING instead of LOW
				caseResult = EvalCaseResult::ECR_FALLING;
				caseResult = EvalCaseResult::ECR_LOW;
				resTxt = "low";
				break;
			case 3:
				caseResult = EvalCaseResult::ECR_LOW;
				resTxt = "low";
				break;
			case 4:
				caseResult = EvalCaseResult::ECR_DETECTING;
				resTxt = "detecting";
				break;
			case 5:
				caseResult = EvalCaseResult::ECR_INVALID;
				resTxt = "invalid";
				break;
			case 6:
				// NOTE: LD-MRS signals always ECR_RAISING instead of HIGH
				caseResult = EvalCaseResult::ECR_RAISING;
				caseResult = EvalCaseResult::ECR_HIGH;
				resTxt = "high";
				break;
			case 7:
				caseResult = EvalCaseResult::ECR_HIGH;
				resTxt = "high";
				break;
			default:
				caseResult = EvalCaseResult::ECR_INVALID;
				resTxt = "invalid";
				break;
			}
			result.m_eCaseResult = caseResult; // 39
			printInfoMessage("LdmrsSopasLayer::evalCaseResultDecoder: CaseResult is <" + resTxt + ">.", m_beVerbose);

			// Dummy read for aFieldInfringement (always 0). Currently not in use, use the
			// result from above instead.
			length = memread_UINT16 (bufferPos); // 40 41
// 			if (length == 0)
// 			{
// 				printInfoMessage("LdmrsSopasLayer::evalCaseResultDecoder: FieldInfringement found.", m_beVerbose);
// 				std::string fieldInfringement = colab::getStringFromBuffer(bufferPos, length);
// 				printInfoMessage("LdmrsSopasLayer::evalCaseResultDecoder: FieldInfringement: " + fieldInfringement + ".", m_beVerbose);
// 			}

			length = memread_UINT16 (bufferPos); // 42 43
			if (length == 0)
			{
				result.m_sCaseName = "";
			}
			else
			{
				result.m_sCaseName = colab::getStringFromBuffer(bufferPos, length);
				printInfoMessage("LdmrsSopasLayer::evalCaseResultDecoder: Case name: " + result.m_sCaseName + ".", m_beVerbose);
			}

			length = memread_UINT16 (bufferPos); // 44 45
			if (length == 0)
			{
				result.m_sComment = "";
			}
			else
			{
				result.m_sComment = colab::getStringFromBuffer(bufferPos, length);
				printInfoMessage("LdmrsSopasLayer::evalCaseResultDecoder: Comment: "+ result.m_sComment + ".", m_beVerbose);
			}

			length = memread_UINT16 (bufferPos); // 46 47
			if (length == 1)
			{
				result.aTimeBlock.uiYear = memread_UINT16 (bufferPos);
				result.aTimeBlock.usiMonth = memread_UINT8 (bufferPos);
				result.aTimeBlock.usiDay = memread_UINT8 (bufferPos);
				result.aTimeBlock.usiHour = memread_UINT8 (bufferPos);
				result.aTimeBlock.usiMinute = memread_UINT8 (bufferPos);
				result.aTimeBlock.usiSec = memread_UINT8 (bufferPos);
				result.aTimeBlock.udiUSec = memread_UINT32 (bufferPos);
			}

			//traceDebug(LDMRS_VERSION) << result << std::endl;
			evalCases->add(result);
		}

		// notify only in case of data change
		if ((evalCases->size() > 0) &&
			((*evalCases) != m_lastEvalCaseResults))
		{
			m_lastEvalCaseResults = *evalCases;
			m_manager->setDeviceData((BasicData*)evalCases);
		}
	}
	else
	{
		printWarning("LdmrsSopasLayer::evalCaseResultDecoder: Received eval case, but we do not want to listen to eval cases!");

		unregisterEvent(index_event_aEvalCaseResult);
	}
}

//
//
//
SensorStateInfo LdmrsSopasLayer::getSensorStateInfo()
{
	SensorStateInfo sensorStateInfo;
	sensorStateInfo.setSourceId(m_deviceId);
	sensorStateInfo.setEvalCases(m_evalCases);
	sensorStateInfo.setFields(m_fields);
	
	MeasurementList measureList;
	measureList.setSourceId(m_deviceId);
	Measurement meas;
//	measureList.setGroupName("SensorState");
//	measureList.setListName("SensorStates");
	
	meas.m_measType = Meastype_DeviceName;
	meas.m_textValue = m_scannerName;
	measureList.m_list.push_back(meas);
	
	meas.m_measType = Meastype_DeviceVersion;
	meas.m_textValue = m_scannerVersion;
	measureList.m_list.push_back(meas);
	
	meas.m_measType = Meastype_ScanFreq;
	meas.m_doubleValue = m_scanFreq;
	measureList.m_list.push_back(meas);
	
	meas.m_measType = Meastype_ScanStartAngle;
	meas.m_doubleValue = m_scanStartAngle;
	measureList.m_list.push_back(meas);

	meas.m_measType = Meastype_ScanStopAngle;
	meas.m_doubleValue = m_scanEndAngle;
	measureList.m_list.push_back(meas);

	meas.m_measType = Meastype_ScanResolution;
	meas.m_doubleValue = m_angleResolution;
	measureList.m_list.push_back(meas);
	
//	measureList.add(Measurement::Temperature, m_temperature);

	// no sensor state available
//	std::bitset<16> stateBits(state);
//	std::bitset<16> inputBits(inputs);
//	std::bitset<16> outputBits(outputs);

	// build sensorStateInfo
	// traceDebug("") << "state: " << stateBits << " - inputs: " << inputBits << " - outputs: " << outputBits << std::endl;
//	SensorStateInfo::StateMap stateMap;


//	stateMap[STATENAME_DEVICE_ERROR] = stateBits[0];
//	stateMap[STATENAME_CONTAMINATION_WARNING] = stateBits[1];
//	stateMap[STATENAME_CONTAMINATION_ERROR] = stateBits[2];
//	stateMap["N_TO_1_FILTER_ENABLED"] = m_bNto1FilterEnable;
//	stateMap["PARTICLE_FILTER_ENABLED"] = m_particleFilter.bEnable;
//	stateMap["MEAN_FILTER_ENABLED"] = m_meanFilter.bEnable;
//	sensorStateInfo.setStateMap(stateMap);

//	SensorStateInfo::StateVector inputStates(16, SensorStateInfo::OFF);
//	SensorStateInfo::StateVector outputStates(16, SensorStateInfo::OFF);
//	for (UINT32 j = 0; j < 16; ++j)
//	{
//		inputStates[j] = inputBits[j] ? SensorStateInfo::ON : SensorStateInfo::OFF;
//		outputStates[j] = outputBits[j] ? SensorStateInfo::ON : SensorStateInfo::OFF;
//	}
//	sensorStateInfo.setInputStates(inputStates);
//	sensorStateInfo.setOutputStates(outputStates);

//	measureList.add(Measurement::SerialNumber, serialNo);
	sensorStateInfo.setMeasurementList(measureList);
	sensorStateInfo.setLastKnownEvalCaseResults(m_lastEvalCaseResults);

	return sensorStateInfo;
}


//
// Encode a FieldParameter structure into CoLa-B-data. This structure can then be sent to the MRS to
// parametrize one of the 16 possible fields.
// Returns the number of bytes that have been written into the buffer.
//
UINT32 LdmrsSopasLayer::colaB_fieldEncoder(BYTE* buffer, const FieldParameter& field)
{
	bool beVerboseHere = m_beVerbose;
//	beVerboseHere = true;
	
	// Make a copy of the original buffer pos
	BYTE* bufferOrg = buffer;
	
	// Print some information about the field
	printInfoMessage("LdmrsSopasLayer::colaB_fieldEncoder: DistScaleFactor: " + toString(field.getDistScaleFactor(), 2) + ".", beVerboseHere);
	printInfoMessage("LdmrsSopasLayer::colaB_fieldEncoder: DistScaleOffset: " + toString(field.getDistScaleOffset(), 2) + ".", beVerboseHere);
	printInfoMessage("LdmrsSopasLayer::colaB_fieldEncoder: AngleScaleFactor: " + toString(field.getAngleScaleFactor()) + ".", beVerboseHere);
	printInfoMessage("LdmrsSopasLayer::colaB_fieldEncoder: AngleScaleOffset: " + toString(field.getAngleScaleOffset()) + ".", beVerboseHere);
	printInfoMessage("LdmrsSopasLayer::colaB_fieldEncoder: FieldTypeIntern: " + toString((INT32)(field.getFieldTypeIntern())) + ".", beVerboseHere);
	printInfoMessage("LdmrsSopasLayer::colaB_fieldEncoder: Field number: " + toString(field.getFieldNumber()) + ".", beVerboseHere);

	// Encode the first part
	memwrite_float(buffer, field.getDistScaleFactor());
	memwrite_float(buffer, field.getDistScaleOffset());
	memwrite_UINT32(buffer, field.getAngleScaleFactor());
	memwrite_INT32(buffer, field.getAngleScaleOffset());
	memwrite_UINT8(buffer, field.getFieldTypeIntern());
	memwrite_UINT8(buffer, field.getFieldNumber());

	// Segmented?
	if (field.getFieldType() == FieldDescription::Segmented)
	{
		// Yes
		printInfoMessage("LdmrsSopasLayer::colaB_fieldEncoder: Writing a segmented field.", beVerboseHere);
		// There is 1 segmented field structure
		memwrite_UINT16(buffer, 1);

		// Number of points
		FieldSegmented* seg = (FieldSegmented*)field.getField();
		memwrite_UINT16(buffer, seg->getNumberOfPoints());

		// The points
		for (UINT16 p=0; p<seg->getNumberOfPoints(); p++)
		{
			// Angle index (transform to 1/32° tics)
//			double angle = angleToRad(field.getAngleScaleOffset() + (double)angleIdx * field.getAngleScaleFactor());
			double angle = seg->getPoints().at(p).getAngle() * rad2deg;
			angle = -angle;
			angle *= 32;
			INT32 angleInt = (INT32)angle;
			INT32 angleDiff = angleInt - field.getAngleScaleOffset();
			angleDiff /= field.getAngleScaleFactor();
			UINT16 angleIdx = (UINT16)angleDiff;
			printInfoMessage("LdmrsSopasLayer::colaB_fieldEncoder: AngleIdx = " + toString(angleIdx) + ".", beVerboseHere);

			memwrite_UINT16(buffer, angleIdx);
			
			// Start dist
			double startDist = seg->getPoints().at(p).getStartDist();
			if (startDist == NaN_double)
			{
				// Invalid
				memwrite_UINT16(buffer, 0xFFFF);
				printInfoMessage("LdmrsSopasLayer::colaB_fieldEncoder: StartDist = invalid.", beVerboseHere);
			}
			else
			{
				startDist *= 1000;
				startDist /= field.getDistScaleFactor();
				startDist -= field.getDistScaleOffset();
				UINT16 startDistInt = (UINT16)startDist;
				memwrite_UINT16(buffer, startDistInt);
				printInfoMessage("LdmrsSopasLayer::colaB_fieldEncoder: StartDist = " + toString(startDistInt) + ".", beVerboseHere);
			}
			
			// End dist
			double endDist = seg->getPoints().at(p).getEndDist();
			if (endDist == NaN_double)
			{
				// Invalid
				memwrite_UINT16(buffer, 0xFFFF);
				printInfoMessage("LdmrsSopasLayer::colaB_fieldEncoder: EndDist = invalid.", beVerboseHere);
			}
			else
			{
				endDist *= 1000;
				endDist /= field.getDistScaleFactor();
				endDist -= field.getDistScaleOffset();
				UINT16 endDistInt = (UINT16)endDist;
				memwrite_UINT16(buffer, endDistInt);
				printInfoMessage("LdmrsSopasLayer::colaB_fieldEncoder: EndDist = " + toString(endDistInt) + ".", beVerboseHere);
			}
		}
	}
	else
	{
		// It is not a segmented field
		memwrite_UINT16(buffer, 0);
	}
	
	// Rectangular?
	if (field.getFieldType() == FieldDescription::Rectangle)
	{
		printInfoMessage("LdmrsSopasLayer::colaB_fieldEncoder: Writing rectangular field.", beVerboseHere);
		memwrite_UINT16(buffer, 1);

		FieldRectangle* rect = (FieldRectangle*)field.getField();
		
		// RefPointAngle
		INT32 refPointAngle = rect->getRefPointAngle() * rad2deg * 32.0;	// ) .getField(). = memread_INT32(bufferPos);
		refPointAngle = -refPointAngle;
		memwrite_INT32(buffer, refPointAngle);

		// RefPointDist
		UINT16 refPointDist = (UINT16)(((rect->getRefPointDist() * 1000.0) / field.getDistScaleFactor()) + field.getDistScaleOffset());
		memwrite_UINT16(buffer, refPointDist);

		// RotAngle
		INT32 rotAngle = rect->getRotAngle() * rad2deg * 32.0;
		rotAngle = -rotAngle;
		memwrite_INT32(buffer, rotAngle);

		// Length
		UINT32 length = rect->getLength() * 1000;
		memwrite_UINT32(buffer, length);
		
		// Width
		UINT32 width = rect->getWidth() * 1000;
		memwrite_UINT32(buffer, width);
	}
	else
	{
		// It is not a rectangular field
		memwrite_UINT16(buffer, 0);
	}

	// It is not a radial field - the MRS does not support radial fields
	memwrite_UINT16(buffer, 0);
	
	// It is not a dynamic field - the MRS does not support dynamic fields.
	memwrite_UINT16(buffer, 0);

	// Version number
	memwrite_UINT16(buffer, (UINT16)field.getVersionNumber());
	
	// Field name length
	memwrite_UINT16(buffer, (UINT16)field.getFieldName().length());

	// Field name
	if (field.getFieldName().length() > 0)
	{
		memwrite_string(buffer, field.getFieldName());
	}

	// Comment length
	memwrite_UINT16(buffer, (UINT16)field.getComment().length());

	// Comment
	if (field.getComment().length() > 0)
	{
		memwrite_string(buffer, field.getComment());
	}
	
	// enable layer filter
	memwrite_UINT8(buffer, 0x00);	// 0 = disabled, 1 = enabled
	
	// layer bit field
	memwrite_UINT16(buffer, 0x000F);	// 0x000F = all layers enabled

	// How many bytes have been used?
	UINT32 len = (UINT32)((UINT64)buffer - (UINT64)bufferOrg);
	
	printInfoMessage("LdmrsSopasLayer::colaB_fieldEncoder: Encoded " + toString(len) + " bytes. All done, leaving.", beVerboseHere);
	
	//
	// Debug
	//
//	traceBuffer("Field buffer contents:", bufferOrg, len);
	
	// Debug: Decode this set of data
//	SopasAnswer a(bufferOrg, len);
//	colaB_fieldDecoder(&a);
	
	return len;
}


}	// namespace devices
