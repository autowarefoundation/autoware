//
// LD_MRS.hpp
// Device class of the LD-MRS class Laserscanners.
//
#ifndef LDMRS_HPP
#define LDMRS_HPP

#include "../BasicDatatypes.hpp"
#include "BasicDevice.hpp"
#include "../manager.hpp"

//#include "../sopas/SopasBase.hpp"
#include "../sopas/LdmrsSopasLayer.hpp"
#include "LuxBase.hpp"

#include "../interfaces/tcp.hpp"

namespace devices
{

using namespace datatypes;


class LDMRS: public BasicDevice
{

private:
	Manager* m_manager;
	LdmrsSopasLayer* m_sopas;
	LuxBase* m_lux;

	bool m_beVerbose;
	bool m_isRunning;

	// Config data for LDMRS
	bool m_weWantScanData;
	bool m_weWantFieldData;
	bool m_weWantObjectData;
	bool m_weWantScanDataFromSopas;
	
	bool m_readOnlyMode;
	std::string  m_ipAddress;
	UINT16 m_luxPortNumber;
	UINT16 m_SopasPortNumber;
	std::string m_inputFileName;

	// Config data for LuxBase
	double m_scanFrequency;
	double m_scanStartAngle;
	double m_scanEndAngle;
	double m_offsetX;
	double m_offsetY;
	double m_offsetZ;
	double m_yawAngle;
	double m_pitchAngle;
	double m_rollAngle;
	
protected:
	void disconnectFunction();
	void onScanReceived();

public:
	LDMRS(Manager* manager);
	~LDMRS();

	// Inherited from device
	virtual bool init();
	virtual void shutdown();
	virtual bool run();
	virtual bool stop();
	virtual bool isRunning();

	// returns true if command succeeded.
	bool getParameter(MrsParameterId id, UINT32* value);
	
	// returns true if command succeeded.
	bool setParameter(MrsParameterId id, UINT32 value);

	// Sets the MRS-internal clock to the given time.
	bool setNtpTime(UINT32 seconds, UINT32 fractionalSec);

	// Sets scan start and end angles.
	bool setScanAngles(double startAngle, double endAngle);

	// Sets sync angle offset.
	bool setSyncAngleOffset(double syncAngle);

	// Sets scan frequency.
	// valid values are 12.5, 25.0 and 50.0 [Hz].
	bool setScanFrequency(double scanFreq);

	std::string getIpAddress();

	void setIpAddress(std::string ipAddress);

	void setWeWantObjectData(bool weWantObjectData);

	// returns the serial number
	std::string getSerialNumber();

	// returns the firmware version
	std::string getFirmwareVersion();

	// Function will be called if the scanner loses its tcp connection.
	static void disconnectFunctionS(void* obj);

	// Function callback to be invoked when a scan was received.
	static void onScanReceivedS(void* obj);
	
	// Write a field to the sensor.
	bool writeField(UINT16 fieldNum, const FieldParameter& para);
	
	// Write the EvalCases to the sensor.
	bool writeEvalCases(const EvalCases& evalCases);
	
	// Stores the SOPAS config data (fields and eval cases) permanently.
	bool flashSopasConfig();
};

}	// namespace devices

#endif // LDMRS_HPP
