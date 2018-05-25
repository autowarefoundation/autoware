//
// LdmrsSopasLayer.hpp
//
#ifndef LDMRSSOPASLAYER_HPP
#define LDMRSSOPASLAYER_HPP

#include "../BasicDatatypes.hpp"
#include "../manager.hpp"
#include "../interfaces/tcp.hpp"
#include "SopasBase.hpp"
#include "../datatypes/EvalCase.hpp"
#include "../datatypes/EvalCases.hpp"
#include "../datatypes/EvalCaseResults.hpp"
#include "../datatypes/FieldParameter.hpp"
#include "../datatypes/Fields.hpp"
#include "../datatypes/SensorStateInfo.hpp"

// Note that this is not a device, but a device helper class
namespace devices
{
	
using namespace datatypes;

// *********************** LDMRSSopasLayer **************************** //


class LdmrsSopasLayer: public SopasBase
{
public:
	LdmrsSopasLayer(Manager* manager,
					const UINT8 deviceID,
					std::string ipAddress,
					UINT16 portNumber,
					bool weWantFieldData,
					bool weWantScanData,
					bool readOnlyMode);

	virtual ~LdmrsSopasLayer();

	bool init(Tcp::DisconnectFunction function, void* obj);	// UINT32 ipAddress, UINT16 portNumber, bool weWantFieldData, bool readOnlyMode, Tcp::DisconnectFunction function);
	bool run();

	// commands supported by LD-MRS
	bool action_login();
	bool action_logout();
	bool action_subscribeEvalCaseResults();
	bool action_unSubscribeEvalCaseResults();
	bool action_subscribeScanData();
	bool action_unSubscribeScanData();
	bool action_startMeasure();
	bool action_stopMeasure();
	bool action_readFields();
	bool action_readEvalCases();
	bool action_readSerialNumber();
	bool action_readScanConfig();
	bool action_writeField(UINT16 fieldNum, const FieldParameter& para);
	bool action_writeEvalCases(const EvalCases& evalCases);
	bool action_flashFieldParameters();
	bool action_flashMrsParameters();
	bool isFieldDataSubscribed() const
	{
		return m_fieldEventIsRegistered;
	}

//	SensorStateInfo getSensorStateInfo();

protected:

	enum ScanFreqEnum
	{
		ScanFreq1250 = 0,
		ScanFreq2500 = 1,
		ScanFreq5000 = 2
	};

	// ATTENTION: Assignment can be wrong after protocol change (see SRT/VarRep.h)
	enum SopasVariableByIndex_LDMRS
	{
		index_var_DeviceIdent = 0x00,
		index_var_SOPASVersion = 0x01,
		index_var_LocationName = 0x02,
		index_var_SerialNumber = 0x03,
		index_var_FirmwareVersion = 0x04,
		index_var_Scanning = 0x05,
		index_var_SopasInfo = 0x06,
		index_var_InternalFeedback = 0x07,
		index_var_TestScanFrequency = 0x08,
		index_var_CIDChecksum = 0x09,
		index_var_TestScanActive = 0x0a,
		index_var_ScanDataConfig = 0x0b,
		index_var_AngleDataConfig = 0x0c,
		index_var_LayerEchoConfig = 0x0d,
		index_var_ScanConfig = 0x0e,
		index_var_MeasMode = 0x0f,
		index_var_ApplRange = 0x10,
		index_var_DataOutputRange = 0x11,
		index_var_AutoStartMeasure = 0x12,
		index_var_field000 = 0x003d,
		index_var_field001 = 0x003e,
		index_var_field002 = 0x003f,
		index_var_field003 = 0x0040,
		index_var_field004 = 0x0041,
		index_var_field005 = 0x0042,
		index_var_field006 = 0x0043,
		index_var_field007 = 0x0044,
		index_var_field008 = 0x0045,
		index_var_field009 = 0x0046,
		index_var_field010 = 0x0047,
		index_var_field011 = 0x0048,
		index_var_field012 = 0x0049,
		index_var_field013 = 0x004a,
		index_var_field014 = 0x004b,
		index_var_field015 = 0x004c,
		index_var_numOfParamFields = 0x004d,
		index_var_evalCaseParam = 0x004e
	};

	static const UINT16 MAX_NUM_OF_FIELDS = 16;

	// Data taken from generated file S_Methds.c
	enum SopasMethodByIndex_LDMRS
	{
		index_meth_SetAccessMode = 0x0000,
		index_meth_GetAccessMode = 0x0001,
		index_meth_Run = 0x0002,
		index_meth_FlashFieldParameters = 0x0003,
		index_meth_GetDescription = 0x0004,
		index_meth_CheckPassword = 0x0005,
		index_meth_MthdFlashLUXParameters = 0x0006,
		index_meth_mStartMeasure = 0x000b,
		index_meth_mStopMeasure = 0x000c
	};

	enum SopasEventByIndex_LDMRS
	{
		index_event_Scanning = 0x0000,
		index_event_ScanDataMonitor = 0x0011,
		index_event_aEvalCaseResult = 0x0029
	};

private:
	SensorStateInfo getSensorStateInfo();
	
	UINT32 colaB_fieldEncoder(BYTE* buffer, const FieldParameter& fieldPara);
	FieldParameter* colaB_fieldDecoder(SopasAnswer* answer);
	void evalCaseResultDecoder(SopasEventMessage& frame);
	void scanDataDecoder(SopasEventMessage& frame);
	EvalCases colaB_evalCaseDecoder(SopasAnswer* answer);
	UINT32 colaB_evalCaseEncoder(BYTE* buffer, const EvalCases& evalCases);
	double angleToRad(INT32 angle);

	Manager* m_manager;
	UINT32 m_deviceId;
	bool m_beVerbose;

	EvalCaseResults m_lastEvalCaseResults; // needed for ldmrs
	EvalCases m_evalCases;
	Fields m_fields;

	double m_angleResolution;
	double m_scanStartAngle;
	double m_scanEndAngle;
	double m_scanFreq;
	
	// Local storage
	std::string m_ipAddress;
	UINT16 m_portNumber;
	bool m_readOnlyMode;
};

}	// namespace devices


#endif // LDMRSSOPASLAYER_HPP
