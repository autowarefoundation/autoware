//
// BasicDatatypes.hpp
//
// Defines very basic structures and types
// Copyright (c) Sick AG
// created: 31.05.2010
//
// HISTORY
//
// 1.0.0	31.05.2010, VWi
//			Initial version.


#ifndef BASICDATATYPES_HPP
#define BASICDATATYPES_HPP

#include <string>	// for std::string
#include <vector>	// for std::vector
#include <stdint.h>

//
// Standard-Datentypen
//
typedef uint64_t      UINT64;
typedef int32_t       INT32;
typedef uint32_t      UINT32;
typedef uint16_t      UINT16;
typedef int16_t       INT16;
typedef uint8_t       UINT8;
typedef int8_t        INT8;
typedef unsigned char BYTE;

#ifndef PI
	#define PI 3.141592653589793238462
#endif
#ifndef deg2rad
	#define deg2rad 0.01745329251994329576923690768 	// (PI / 180.0)
#endif
#ifndef rad2deg
	#define rad2deg 57.29577951308232087679815481		// (180.0 / PI)
#endif


//
enum Datatypes
{
	Datatype_Unknown 				= 0x0000,
	
	Datatype_MeasurementList		= 0x0001,
	Datatype_Box2D					= 0x0002,
	Datatype_Line2D					= 0x0003,
	Datatype_Polygon2D				= 0x0004,
	Datatype_Polygon3D				= 0x0005,
	Datatype_Point2D				= 0x0006,
	Datatype_Point3D				= 0x0007,
	Datatype_Circle2D				= 0x0008,
	Datatype_Ellipse2D				= 0x0009,
	Datatype_Msg					= 0x000A,
	Datatype_Scan					= 0x000B,
	Datatype_Objects				= 0x000C,
	Datatype_Scannerinfo			= 0x000D,
	Datatype_Trigger				= 0x000E,
	Datatype_EvalCaseResult			= 0x000F,
	Datatype_EvalCaseResults		= 0x0010,
	Datatype_EvalCase				= 0x0011,
	Datatype_EvalCases				= 0x0012,
	Datatype_FieldParameter			= 0x0013,
	Datatype_FieldDescription		= 0x0014,
	Datatype_Fields					= 0x0015,
	Datatype_SensorStateInfo		= 0x0016
};

//
// Type-IDs of modules (devices, applications, ...)
//
enum Sourcetype
{
	Sourcetype_Unknown 				= 0x0000,
	
	// Devices = 0x0001 - 0x0FFF
	Sourcetype_LDMRS				= 0x0003,

	// Applications = 0x1000 - 0x1FFF
	Sourcetype_MrsApp				= 0x1002,
	Sourcetype_MrsChangeApp			= 0x1003,
	Sourcetype_MrsFieldApp			= 0x1004,
	Sourcetype_MrsNtpTimeApp		= 0x1005,
	Sourcetype_MrsScanpointCoordinateApp = 0x1006
};

namespace datatypes
{

// Datencontainer fuer alle transportierbaren Daten
class BasicData
{
public:
	BasicData() {m_datatype = Datatype_Unknown; m_sourceId = Sourcetype_Unknown;}
	virtual ~BasicData() {}
	
	UINT16 getDatatype() {return m_datatype;}
	UINT16 getSourceId() {return m_sourceId;}
	virtual void setSourceId(UINT16 id) {m_sourceId = id;}
	virtual const UINT32 getUsedMemory() const = 0;
	
protected:
	UINT16 m_datatype;		// Typ dieses Datums
	UINT16 m_sourceId;		// Unique ID of the data source

private:
};


}	// namespace datatypes

#endif // BASICDATATYPES_HPP
