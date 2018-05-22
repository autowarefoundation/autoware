//
// LdmrsFieldApp.cpp
//
// Demo application.
//

#include "LdmrsFieldApp.hpp"
#include "../tools/errorhandler.hpp"	// for printInfoMessage()
#include "../tools/toolbox.hpp"			// for toString()
#include "../datatypes/Scan.hpp"
#include "../datatypes/Object.hpp"
#include "../datatypes/Msg.hpp"
#include "../datatypes/Measurement.hpp"
#include "../datatypes/Fields.hpp"
#include "../datatypes/EvalCases.hpp"
#include "../datatypes/EvalCaseResults.hpp"
#include "../devices/LD_MRS.hpp"

namespace application
{

//
// Constructor
//
LdmrsFieldApp::LdmrsFieldApp(Manager* manager)
{
	m_beVerbose = true;
	m_manager = manager;
	
	// Start the thread
	if (m_changeThread.isRunning() == false)
	{
		m_changeThread.run(this);
	}


	printInfoMessage("LdmrsFieldApp constructor done.", m_beVerbose);
}


// Destructor
// Clean up all dynamic data structures
LdmrsFieldApp::~LdmrsFieldApp()
{
	printInfoMessage("LdmrsFieldApp says Goodbye!", m_beVerbose);
}


//
// Receiver for new data from the manager.
//
void LdmrsFieldApp::setData(BasicData& data)
{
	//
	// Do something with it.
	//
	// Here, we just print what we've got.
	//
	std::string datatypeStr;
	std::string sourceIdStr;

	switch (data.getDatatype())
	{
		case Datatype_Scan:
			datatypeStr = "Scan (" + ::toString(((Scan&)data).getNumPoints()) + " points)";
 			{
				// Print the scan start timestamp (NTP time) 
 				Scan* scan = dynamic_cast<Scan*>(&data);
 				const ScannerInfo* info = scan->getScannerInfoByDeviceId(1);
 
 				if (info != NULL)
 				{
 					const Time& time = info->getStartTimestamp();
 					printInfoMessage("LdmrsApp::setData(): Scan start time: " + time.toString(), m_beVerbose);
 				}
 			}
			break;
		case Datatype_Objects:
			datatypeStr = "Objects (" + ::toString(((ObjectList&)data).size()) + " objects)";
			break;
		case Datatype_Fields:
			datatypeStr = "Fields (" + ::toString(((Fields&)data).getFields().size()) + " fields, " +
							::toString(((Fields&)data).getNumberOfValidFields()) + " of which are valid)";
			break;
		case Datatype_EvalCases:
			datatypeStr = "EvalCases (" + ::toString(((EvalCases&)data).getEvalCases().size()) + " cases)";
			break;
		case Datatype_EvalCaseResults:
			datatypeStr = "EvalCaseResults (" + ::toString(((EvalCaseResults&)data).size()) + " case results)";
			break;
		case Datatype_Msg:
			datatypeStr = "Msg (" + ((Msg&)data).toString() + ")";
			break;
		case Datatype_MeasurementList:
			datatypeStr = "MeasurementList (" + ::toString(((MeasurementList&)data).m_list.size()) +" entries)";
			break;
		default:
			datatypeStr = "(unknown)";
	}
	
	sourceIdStr = ::toString(data.getSourceId());
	
	printInfoMessage("LdmrsApp::setData(): Called with data of type " + datatypeStr + " from ID " + sourceIdStr + ".", m_beVerbose);
}


//
// Demo function that removes all fields by overwriting them with segmented fields,
// and setting them to invalid.
//
void LdmrsFieldApp::thread_removeAllEvalCases(devices::LDMRS* ldmrs)
{
	bool beVerboseHere = m_beVerbose;
	beVerboseHere = true;
	
	printInfoMessage("LdmrsFieldApp::thread_removeAllEvalCases: Called.", beVerboseHere);

	bool result;
	EvalCases cases;
	result = ldmrs->writeEvalCases(cases);
	if (result == false)
	{
		// Failure
		printError("LdmrsFieldApp::thread_removeAllEvalCases: Failed to write empty EvalCase structure!");
	}

	printInfoMessage("LdmrsFieldApp::thread_removeAllEvalCases: All done, leaving.", beVerboseHere);
}


//
// Demo function that removes all fields by overwriting them with segmented fields,
// and setting them to invalid.
//
void LdmrsFieldApp::thread_removeAllFields(devices::LDMRS* ldmrs)
{
	bool beVerboseHere = m_beVerbose;
	beVerboseHere = true;
	
	printInfoMessage("LdmrsFieldApp::thread_removeAllFields: Called.", beVerboseHere);

	bool result;
	
	// Create an empty field which is invalid
	FieldParameter field;
	field.setDistScaleFactor(10);	// With a value of 10, coordinate unit is [cm]
	field.setDistScaleOffset(0);	// No offset
	field.setAngleScaleFactor(8);	// Angle unit is [1/4 deg]
	field.setAngleScaleOffset(-1920);	// -1920 = -60 deg (=60 deg due to other coordinate system)
	field.setComment("");
	field.setFieldName("");
	field.setEnableLayerFilter(false);
	field.setVersionNumber(1);		// Version number is 1
	field.setFieldTypeIntern(FieldParameter::FieldTypeIntern_SEGMENTED);	// Segmented = 2
	field.setFieldNumber(0);		// 0 is an invalid field
	field.setField(NULL);			// (FieldDescription*)
	
	for (UINT16 i = 0; i<16; i++)
	{
		printInfoMessage("LdmrsFieldApp::thread_removeAllFields: Removing field " + toString(i) + ".", beVerboseHere);
		result = ldmrs->writeField(i, field);
		if (result == false)
		{
			// Failure
			printError("LdmrsFieldApp::thread_removeAllFields: Failed to clear field " + toString(i) + "!");
		}
	}
	
	printInfoMessage("LdmrsFieldApp::thread_removeAllFields: All done, leaving.", beVerboseHere);
}

//
// Demo function that creates an eval case.
//
void LdmrsFieldApp::thread_createEvalCase(devices::LDMRS* ldmrs)
{
	bool beVerboseHere = m_beVerbose;
	beVerboseHere = true;
	
	printInfoMessage("LdmrsFieldApp::thread_createEvalCase(): Called.", beVerboseHere);

	bool result;
	
	// The single eval case
	EvalCase* c = new EvalCase;
	c->setVersionNumber(1);
	c->setCaseNumber(1);					// Case number must be >0 for a valid case
	c->setStrategy(EvalCase::BLANKING);
	c->setResultNegation(false);
	c->setResponseTime(500);				// ms
	c->setResponseTimeExtended(500);		// ms
	c->setOutputNumber(0);				// 0 = none
	c->setLogicalInputState_from_UINT8(0x00);	// 0 = no inputs
	c->setDistDependent(false);
	c->setMaxRadialCorridor(2.0);
	c->setManipulationPrevention(EvalCase::ECS_INACTIVE);
	c->setBlankingSize(0.2);
	c->setMinFieldExp(0);
	c->setFieldNumber(1);
	c->setFilterType(EvalCase::UNFILTERED);
	c->setCaseName("DemoCase-1");
	c->setComment("");
	
	// The cases carrier structure
	EvalCases cases;
	cases.add(c);
	
	result = ldmrs->writeEvalCases(cases);
	if (result == false)
	{
		// Failure
		printError("LdmrsFieldApp::thread_createEvalCase: Failed to write the EvalCase structure!");
	}

	printInfoMessage("LdmrsFieldApp::thread_createEvalCase: All done, leaving.", beVerboseHere);
}

//
// Demo function that creates a rectangular field.
//
void LdmrsFieldApp::thread_createRectangularField(devices::LDMRS* ldmrs)
{
	bool beVerboseHere = m_beVerbose;
	beVerboseHere = true;
	
	printInfoMessage("LdmrsFieldApp::thread_createRectangularField(): Called.", beVerboseHere);

	// Create an empty field which is invalid
	FieldParameter field;
	field.setDistScaleFactor(10);	// With a value of 10, coordinate unit is [cm]
	field.setDistScaleOffset(0);	// No offset
	field.setAngleScaleFactor(8);	// Angle unit is [1/4 deg]
	field.setAngleScaleOffset(-1920);	// -1920 = -60 deg (=60 deg due to other coordinate system)
	field.setComment("");
	field.setFieldName("Rect_1");
	field.setEnableLayerFilter(false);
	field.setVersionNumber(1);		// Version number is 1
	field.setFieldTypeIntern(FieldParameter::FieldTypeIntern_RECTANGLE);	// Rectangle = 1
	field.setFieldNumber(1);		// A field number 0 marks the field as invalid
	
	// Create a rectangular field sub-structure
	FieldRectangle rectField;
//	FieldSegmentedPoint segPt;

	rectField.setRefPointDist(2.06);	// Dist to ref point is 2.06 m
	rectField.setRefPointAngle(13.8 * deg2rad);	// Angle of 13.8 deg
	rectField.setRotAngle(0.0 * deg2rad);			// 0 = No rotation
	rectField.setLength(5.0);			// Length 5 m (x direction)
	rectField.setWidth(2.0);			// Width 2 m (y direction)
	
	// Add the rectangular field to the field
	field.setField(&rectField);	// (FieldDescription*)

	// Write the field as field 1 to the sensor
	bool result = false;
	result = ldmrs->writeField(0, field);
	if (result == false)
	{
		// Failure
		printError("LdmrsFieldApp::thread_createRectangularField(): Failed to write the field!");
	}
	
	printInfoMessage("LdmrsFieldApp::thread_createRectangularField(): All done, leaving.", beVerboseHere);
}

//
// Thread that does the actual changing of parameters.
//
void LdmrsFieldApp::changeThreadFunction(bool& endThread, UINT16& waitTimeMs)
{
	bool beVerboseHere = m_beVerbose;
	beVerboseHere = true;
		
	printInfoMessage("LdmrsFieldApp::changeThreadFunction(): started", beVerboseHere);

	devices::LDMRS* ldmrs;
	
	// Wait until the LD-MRS is present
	for (UINT16 i = 0; i<10; i++)
	{
		ldmrs = dynamic_cast<devices::LDMRS*>(m_manager->getFirstDeviceByType(Sourcetype_LDMRS));
		if (ldmrs == NULL)
		{
			// Sleep for a second
			usleep(1000000);
		}
	}

	if (ldmrs == NULL)
	{
		// Scanner device not found
		printError("LdmrsFieldApp::changeThreadFunction(): Failed to read device pointer, aborting!");
		endThread = true;
		waitTimeMs = 0;
		return;
	}
	
	// Now wait until the connection is established
	for (UINT16 i = 0; i<10; i++)
	{
		if (ldmrs->isRunning() == false)
		{
			// Sleep for a second
			usleep(1000000);
		}
	}
	// Sleep some more, just to be sure
	usleep(3 * 1000000);
	
	UINT32 sleepTimeS;

	
	//
	// Remove all existing eval cases
	//
	printInfoMessage("LdmrsFieldApp::changeThreadFunction(): Now removing all eval cases.", beVerboseHere);
	thread_removeAllEvalCases(ldmrs);
	

 	//
	// Remove all exisiting fields by overwriting them with invalid fields.
	//
	printInfoMessage("LdmrsFieldApp::changeThreadFunction(): Now removing all fields.", beVerboseHere);
	thread_removeAllFields(ldmrs);
	
	// sleep some time
	sleepTimeS = 3;
	usleep(sleepTimeS * 1000 * 1000);

	
	//
	// Write a single rectangular field to the sensor.
	//
	printInfoMessage("LdmrsFieldApp::changeThreadFunction(): Now writing a rectangular field.", beVerboseHere);
	thread_createRectangularField(ldmrs);
	
	// sleep some time
	sleepTimeS = 1;
	usleep(sleepTimeS * 1000 * 1000);

	
	//
	// Write a single eval case to the sensor.
	//
	printInfoMessage("LdmrsFieldApp::changeThreadFunction(): Now writing an eval case for the rectangular field.", beVerboseHere);
	thread_createEvalCase(ldmrs);
	
	
	//
	// Save the current field configuration permanently.
	//
//	printInfoMessage("LdmrsFieldApp::changeThreadFunction(): Now flashing the current field configuration.", beVerboseHere);
//	ldmrs->flashSopasConfig();
	
	// sleep some time
	sleepTimeS = 5;
	usleep(sleepTimeS * 1000 * 1000);

	
	endThread = true;
	waitTimeMs = 0;
	printInfoMessage("LdmrsFieldApp::changeThreadFunction(): All done, leaving.", m_beVerbose);
}


}	// namespace application
