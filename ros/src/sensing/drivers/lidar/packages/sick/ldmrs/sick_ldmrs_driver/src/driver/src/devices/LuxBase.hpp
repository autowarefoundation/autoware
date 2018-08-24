//
// LuxBase.hpp
//
//  Created on: 18.07.2011
//      Author: sick
//

#ifndef LUXBASE_HPP
#define LUXBASE_HPP

#include "../BasicDatatypes.hpp"
#include "../manager.hpp"
#include "../interfaces/tcp.hpp"
#include "../interfaces/file.hpp"
#include "../tools/SickThread.hpp"
#include "../datatypes/Position3D.hpp"
#include "../datatypes/Point2D.hpp"
#include "../datatypes/Scan.hpp"


namespace devices
{

#define MRS_INPUTBUFFERSIZE 65536			// Size of the main input buffer
#define MRS_CMDREPLYBUFFERSIZE 8192			// Size of the command reply buffer (some bytes should be enough...)

using namespace datatypes;

enum MrsCommandId
{
	// Common MRS commands
	CmdMrsReset						= 0x0000	///< 0x0000 = ID of the Reset command
	, CmdMrsGetStatus				= 0x0001	///< 0x0001 = ID of the GetStatus command
	, CmdMrsSetMode					= 0x0002 	///< 0x0002 = ID of the SetMode command
	, CmdMrsSetConfig				= 0x0003	///< 0x0003 = ID of the SetConfig command
	, CmdMrsSaveConfig				= 0x0004	///< 0x0004 = ID of the SaveConfig command
	, CmdMrsReadConfig				= 0x0005	///< 0x0005 = ID of the ReadConfig command
	, CmdMrsFlashFirmware			= 0x0006	///< 0x0006 = ID of the FlashFirmware command

	// Specific commands
	, CmdMrsSetParameter			= 0x0010	///< 0x0010 = sets a parameter in the sensor
	, CmdMrsGetParameter			= 0x0011	///< 0x0011 = reads a parameter from the sensor
	, CmdMrsResetDefaultParameters	= 0x001A	///< 0x001A = resets all parameters to the factory defaults
	, CmdMrsStartMeasure			= 0x0020	///< 0x0020 = starts the measurement with the currenly configured settings
	, CmdMrsStopMeasure				= 0x0021	///< 0x0021 = stops the measurement
	, CmdMrsSetNTPTimestampSec		= 0x0030	///< 0x0030 = sets NTP seconds
	, CmdMrsSetNTPTimestampFracSec	= 0x0031	///< 0x0031 = sets NTP fractional seconds
};

// Parameters of the MRS
enum MrsParameterId
{
	ParaDataOutputFlag			= 0x1012		// Sets the output flags to enable or disable scans, objects, ...
	, ParaContourPointDensity   = 0x1014		// 0: closest point only, 1: low density, 2: high density
	, ParaMinimumObjectAge      = 0x1017		// Minimum tracking age (number of scans) of an object to be transmitted. valid: 0..65535
	, ParaMaximumPredictionAge  = 0x1018		// Maximum prediction age (number of scans) of an object to be transmitted after last observation. valid: 0..65535
	, ParaScanFrequency			= 0x1102		// Sets the scan frequency, in 1/256 Hz (valid = 3200 (12.5 Hz),6400 (25.0 Hz) and 12800 (50 Hz)
	, ParaStartAngle			= 0x1100		// 1/32 deg, Valid is 1600..-1919; Start angle > end angle!
	, ParaEndAngle				= 0x1101		// 1/32 deg, Valid is 1599..-1920; Start angle > end angle!
	, ParaSyncAngleOffset       = 0x1103        // 1/32 deg, Valid is -5760..5759; angle under which the LD-MRS measures at the time of the external sync pulse
	, ParaAngularResolutionType = 0x1104        // angular resolution type: 0=focused, 1=constant(0.25°), 2=FlexRes
	, ParaRangeReduction		= 0x1108		// Available for LDMRS800001.S01 only; 0: full sensitivity (default), 1: lower 4 layers reduced, 2: upper 4 layers reduced, 3: both reduced
	, ParaUpsideDownMode		= 0x1109		// UpsideDown mode on/off (0=off, 1=on)
	, ParaIgnoreNearRange		= 0x110A		// Available for LDMRS800001.S01 only; 0: do not ignore points in near range (up to 15m), 1: ignore points in near range if 0x1108 is 1
	, ParaSensitivityControl	= 0x110B		// 0: not active (default), 1: Sensitivity will be reduced dynamically down to 60% in case of direct sun light.
	, ParaMountingX				= 0x1200		// X-Pos of the scanner, in [cm]
	, ParaMountingY				= 0x1201		// Y-Pos of the scanner, in [cm]
	, ParaMountingZ				= 0x1202		// Z-Pos of the scanner, in [cm]
	, ParaMountingYaw			= 0x1203		// Yaw angle
	, ParaMountingPitch			= 0x1204		// Pitch angle
	, ParaMountingRoll			= 0x1205		// Roll angle
	, ParaBeamTilt				= 0x3302		// Beam tilt angle, in [compressedRadians]
	, ParaNumSectors            = 0x4000        // Flex Resolution number of sectors
	, ParaSector1StartAngle     = 0x4001        // Flex Resolution sector 1 start angle
	, ParaSector2StartAngle     = 0x4002        // Flex Resolution sector 2 start angle
	, ParaSector3StartAngle     = 0x4003        // Flex Resolution sector 3 start angle
	, ParaSector4StartAngle     = 0x4004        // Flex Resolution sector 4 start angle
	, ParaSector5StartAngle     = 0x4005        // Flex Resolution sector 5 start angle
	, ParaSector6StartAngle     = 0x4006        // Flex Resolution sector 6 start angle
	, ParaSector7StartAngle     = 0x4007        // Flex Resolution sector 7 start angle
	, ParaSector8StartAngle     = 0x4008        // Flex Resolution sector 8 start angle
	, ParaSector1Resolution     = 0x4009        // Flex Resolution sector 1 resolution
	, ParaSector2Resolution     = 0x400A        // Flex Resolution sector 2 resolution
	, ParaSector3Resolution     = 0x400B        // Flex Resolution sector 3 resolution
	, ParaSector4Resolution     = 0x400C        // Flex Resolution sector 4 resolution
	, ParaSector5Resolution     = 0x400D        // Flex Resolution sector 5 resolution
	, ParaSector6Resolution     = 0x400E        // Flex Resolution sector 6 resolution
	, ParaSector7Resolution     = 0x400F        // Flex Resolution sector 7 resolution
	, ParaSector8Resolution     = 0x4010        // Flex Resolution sector 8 resolution
	, ParaOperatingMinutes      = 0x3500        // Operating minutes
	, ParaDetailedError         = 0x7000        // detailed error number
};

// This list is not complete. Just an excerpt.
enum DetailedErrorNumber
{
	ErrFlexResNumShotsInvalid               = 0x006C
	, ErrFlexResSizeOneEighthSectorInvalid  = 0x006D
	, ErrFlexResFreqInvalid                 = 0x006E
	, ErrFlexResSectorsOverlapping          = 0x006F
	, ErrFlexResScannerNotIdle              = 0x0070
	, ErrFlexResResolutionInvalid           = 0x0071
	, ErrFlexResNumSectorsInvalid           = 0x0072
};

// Object classes
enum MrsObjectClasses
{
	ClassUnclassified	= 0
	, ClassUnknownSmall	= 1
	, ClassUnknownBig	= 2
	, ClassPedestrian	= 3
	, ClassBike			= 4
	, ClassCar			= 5
	, ClassTruck		= 6
};

enum AngularResolutionType
{
	ResolutionTypeFocused    = 0
	, ResolutionTypeConstant = 1
	, ResolutionTypeFlexRes  = 2
};


//
// Generic device class of the LD-MRS Laserscanner
//
class LuxBase
{
public:
	typedef void (*OnScanReceiveCallbackFunction)(void*);
//	typedef void ()  OnScanReceiveCallbackFunction;
	
	// Tools, in generelle Klasse verschieben und inline machen!
	void memreadLE(BYTE*& buffer, UINT16& value);
	void memreadLE(BYTE*& buffer, UINT32& value);

private:
	// ----- member variables -----

	UINT16 m_firmwareVersion;				// 0125
	UINT16 m_FPGAVersion;					// 5310
	UINT16 m_scannerStatus;					// 0343
	//
	// Attention: If reply is received from a LUX3, this member contains the SVN-Revision-Number
	// of current LUX3-Firmware.
	// The Error value is not needed anymore for LUX3.
	//
	UINT16 m_FPGAError;						// 0000
	//
	// Attention: If reply is received from a LUX3, this member contains the scanner type.
	// The Error value is not needed anymore for LUX3.
	//
	UINT16 m_DSPError;						// 0006
	UINT16 m_temperature;					// FFFF
	UINT16 m_serialNumber[3];				// 0608 5600 0F00
	UINT16 m_FPGATimestamp[3];				// 2007 1212 1840
	UINT16 m_firmwareTimestamp[3];			// 2007 1002 1000
	bool m_isRunning;
	Tcp m_tcp;
	File m_file;
	Tcp::DisconnectFunction m_tcpDisconnectFunction;
	File::DisconnectFunction m_fileDisconnectFunction;
	void* m_disconnectFunctionObjPtr;
	OnScanReceiveCallbackFunction m_onScanReceiveCallback;
	void* m_onScanReceiveCallbackObjPtr;

	// Updates the device status including temperature
	void updateThreadFunction(bool& endThread, UINT16& sleepTimeMs);
	SickThread<LuxBase, &LuxBase::updateThreadFunction> m_updateThread;
	Mutex m_updateMutex; ///< Access mutex for update

	Manager* m_manager;
	std::string m_longName;
	UINT8 m_deviceId;
	std::string m_ipAddress;
	UINT16 m_tcpPortNumber;
	bool m_readOnlyMode;
	std::string m_inputFileName;

	double  m_scanFrequency;
	double  m_scanStartAngle;
	double  m_scanEndAngle;
	double  m_offsetX;
	double  m_offsetY;
	double  m_offsetZ;
	double  m_yawAngle;
	double  m_pitchAngle;
	double  m_rollAngle;
	double 	m_beamTiltAngle;
	bool m_upsideDownActive;

	// Input stuff
	UINT8  m_inputBuffer[MRS_INPUTBUFFERSIZE];
	UINT32 m_inBufferLevel;	// Bytes in input buffer
	Mutex  m_inputBufferMutex;
	// The CMD REPLY buffer is a separate buffer for everything except scans and object data
	UINT32 m_cmdBufferLevel;	// Bytes in reply input buffer
	UINT8  m_cmdReplyBuffer[MRS_CMDREPLYBUFFERSIZE];

	UINT16 m_errorRegister1;
	UINT16 m_errorRegister2;
	UINT16 m_warnRegister1;
	UINT16 m_warnRegister2;

	// error values
	static const UINT16 Err_IF_SPORT			= 0x0001;
	static const UINT16 Err_IF_FPGAcontrol		= 0x0002;
	static const UINT16 Err_SPORTdata			= 0x0004;
	static const UINT16 Err_FPGAkonfiguration	= 0x0008;
	static const UINT16 Err_ConfReg				= 0x0010;
	static const UINT16 Err_Parameter			= 0x0020;
	static const UINT16 Err_Timing				= 0x0040;
	static const UINT16 Err_TrackingTimeout		= 0x0080;
	static const UINT16 Err_CANMessageLost		= 0x0100;
	static const UINT16 Err_FlexResParameter	= 0x0200;

	// ----- Command and decode functions -----
	bool    sendMrsCommand(MrsCommandId cmd, UINT16 para = 0, UINT32 value = 0);
	bool    receiveMrsReply(MrsCommandId cmd, UINT32 timeoutMs, UINT32* value = NULL);
	UINT32  buildDataHeader(UINT8* buffer, UINT32 sizeofThisMsg, UINT8 deviceId, UINT16 dataType);
	UINT32  writeValueToBufferBE(UINT8* buffer, UINT32 value, UINT8 bytes);
	UINT32  writeValueToBufferLE(UINT8* buffer, UINT32 value, UINT8 bytes);
	UINT32  readUValueLE(UINT8* buffer, UINT8 bytes);
	UINT64  readUINT64ValueLE(UINT8* buffer);
	INT32   readValueLE(UINT8* buffer, UINT8 bytes);
	UINT32  readUValueBE(UINT8* buffer, UINT8 bytes);
	UINT16  decodeAnswerInInputBuffer();		// Decodes the general input buffer
	UINT16  decodeAnswerInCmdReplyBuffer();	// Decodes the reply buffer (everything except scnas & objects)
	void    removeAnswerFromInputBuffer();
	bool    decodeGetStatus();
	bool    decodeGetParameter(UINT32* value);
	void    decodeObjects();
	void    decodeScan();
	void    decodeSensorInfo();
	void    decodeErrorMessage();
	double   convertTicktsToAngle(INT16 angleTicks);
	Point2D readPoint2D(UINT8* buffer);
	Point2D readSize2D(UINT8* buffer);
	bool 	readBeamTilt();
	bool	readUpsideDown();
	double	getVAngleOfLayer(bool isRearMirrorSide, UINT8 layerNumber, double hAngle);



	static void readCallbackFunctionS(void* obj, BYTE* buffer, UINT32& numOfBytes);
	void    readCallbackFunction(BYTE* buffer, UINT32& numOfBytes);
	void    removeDataFromInputBuffer(UINT32 bytesToBeRemoved);
	void    moveDataFromInputToCmdBuffer(UINT32 bytesToBeMoved);
	void    makeIntValueEven(INT16& value);

	static std::string int2Version (UINT16 val);
	static std::string version2string (UINT16 version, const UINT16 timestamp[3]);
	static bool isValidVersion (UINT16 version) { return (version != 0x0000) && (version != 0xFFFF); }

	bool m_weWantScanData;		// Flag if received data is to be decoded.
	bool m_weWantObjectData;	// Flag if received data is to be decoded.
//	TimeOffsetFilter m_timeOffsetFilter;
	bool m_beVerbose;

public:
	LuxBase (Manager* manager, const UINT8 deviceID, const std::string longName,
			 std::string ipAddress, UINT16 tcpPortNumber,
			 double scanFrequency, double scanStartAngle, double scanEndAngle, double offsetX,
			 double offsetY, double offsetZ, double yawAngle, double pitchAngle, double rollAngle,
			 bool beVerbose, std::string inputFileName);
	virtual ~LuxBase();

	virtual bool initFile(File::DisconnectFunction function, void* obj); // , bool beVerbose = false);
	virtual bool initTcp(Tcp::DisconnectFunction function, void* obj); // , bool beVerbose = false);
	virtual bool run(bool weWantScanData = true, bool weWantObjectData = true);
	virtual bool isRunning();
	virtual bool stop();

	void setOnScanReceiveCallbackFunction(OnScanReceiveCallbackFunction function, void* obj);

	bool cmd_getStatus();
	bool cmd_stopMeasure();
	bool cmd_startMeasure(bool weWantScanData = true, bool weWantObjectData = true);
	bool cmd_getParameter(MrsParameterId parameter, UINT32* value);
	bool cmd_setParameter(MrsParameterId parameter, UINT32 value);
	bool cmd_setMountingPos(Position3D mp);
	bool cmd_setScanAngles(double startAngle, double endAngle);
	bool cmd_setSyncAngleOffset(double syncAngle);
	bool cmd_setScanFrequency(double scanFreq);
	bool cmd_setDataOutputFlags();
	bool cmd_saveConfiguration();
	bool cmd_setNtpTimestamp(UINT32 seconds, UINT32 fractionalSec);

	std::string getFPGAVersion    () { ScopedLock lock(&m_updateMutex); return version2string (m_FPGAVersion    , m_FPGATimestamp    ); }
	std::string getFirmwareVersion() { ScopedLock lock(&m_updateMutex); return version2string (m_firmwareVersion, m_firmwareTimestamp); }
	UINT16 getFPGAError()         { ScopedLock lock(&m_updateMutex); return m_FPGAError; }
	UINT16 getDSPError()          { ScopedLock lock(&m_updateMutex); return m_DSPError; }
	double getTemperature();
	UINT16 getScannerStatus()    { ScopedLock lock(&m_updateMutex); return m_scannerStatus; }
	std::string getSerialNumber();

	UINT16 getErrorRegister1() { return m_errorRegister1; }
	UINT16 getErrorRegister2() { return m_errorRegister2; }
	UINT16 getWarnRegister1() { return m_warnRegister1; }
	UINT16 getWarnRegister2() { return m_warnRegister2; }

	void dumpHeader();
	void dumpMessage();

	static const UINT32 ANGULAR_TICKS_PER_ROTATION = 11520;	// [ticks], 360*32= 11520 ticks;
};

}	// namespace devices

#endif // LUXBASE_HPP
