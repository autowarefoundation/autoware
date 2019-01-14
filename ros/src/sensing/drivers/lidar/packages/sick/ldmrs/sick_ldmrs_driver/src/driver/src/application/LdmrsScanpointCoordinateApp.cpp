//
// LdmrsScanpointCoordinateApp.cpp
//
// Demo application. Receives scans, and searches each layer for the point at a certain horizontal scan angle.
// Adds up some scans and then prints the distance and coordinates of these points.
// This demo shows the scanpoint coordinates of scanner layers, e.g. the z coordinates. It can be used to verify the
// calculation of the coordinates.
// Also shows how to work with the mirror sides and 4- and 8-layer-scanners.
//
// In the constructor, set the angle and number of scans to the desired values.
// Note that scans must be enabled in the MRS device in order for this application to work.
//

#include "LdmrsScanpointCoordinateApp.hpp"
#include "../tools/errorhandler.hpp"	// for printInfoMessage()
#include "../tools/toolbox.hpp"			// for toString()
#include "../datatypes/Scan.hpp"
#include "../datatypes/Object.hpp"
#include "../datatypes/Msg.hpp"
#include "../datatypes/Measurement.hpp"
#include "../datatypes/Fields.hpp"
#include "../datatypes/EvalCases.hpp"
#include "../datatypes/EvalCaseResults.hpp"

namespace application
{

//
// Constructor
//
LdmrsScanpointCoordinateApp::LdmrsScanpointCoordinateApp(Manager* manager)
{
	// Enable this flag for *very* verbose debug output
	m_beVerbose = false;	// true;
	
	// Number of scans to build the statistics with. Note that for 8-layer scanners, the number of points in each layer
	// will be half of this value as the scanner needs two scans for a complete 8-layer-scan.
	m_numScansToCount = 20;
	
	// Horizontal scan angle at which the sampling is done.
	// +degrees: "to the left",
	// 0.0 : "straight forward",
	// -degrees: "to the right"
	m_wantedHorzAngle = 0.0 * deg2rad;
	
	// Some other stuff
	initCycle();
	m_isFourLayerScanner = true;

	printInfoMessage("LdmrsScanpointCoordinateApp constructor done.", m_beVerbose);
}


// Destructor
// Clean up all dynamic data structures
LdmrsScanpointCoordinateApp::~LdmrsScanpointCoordinateApp()
{
	printInfoMessage("LdmrsScanpointCoordinateApp says Goodbye!", m_beVerbose);
}

//
// Initialize all data for a new collection cycle.
//
void LdmrsScanpointCoordinateApp::initCycle()
{
	m_scanCounter = 0;
	for (UINT8 i = 0; i<8; i++)
	{
		m_numPoints[i] = 0;
		m_meanDist[i] = 0.0;
		m_meanXyz[i] = Point3D(0.0, 0.0, 0.0);
	}
}


//
// Receiver for new data from the manager.
//
void LdmrsScanpointCoordinateApp::setData(BasicData& data)
{
	//
	// Do something with it.
	//
	// Here, we sort out scan data and hand it over to the processing.
	//
	std::string datatypeStr;
	std::string sourceIdStr;

	if (data.getDatatype() == Datatype_Scan)
	{
		Scan* scan = dynamic_cast<Scan*>(&data);

		// Search the points and sum them up
		processScan(scan);
		m_scanCounter++;
		
		if (m_scanCounter >= m_numScansToCount)
		{
			// Calculate and print the statistics
			calcAndShowStatistics();
			
			// Begin new cycle, reset all variables
			initCycle();
		}
	}
	
	printInfoMessage("LdmrsScanpointCoordinateApp::setData(): Done.", m_beVerbose);
}

//
// Calculate the mean values of the coordinates, and print the values.
//
void LdmrsScanpointCoordinateApp::calcAndShowStatistics()
{
	// Calc statistics
	for (UINT8 layer = 0; layer < 8; layer++)
	{
		if (m_numPoints[layer] > 1)
		{
			m_meanXyz[layer] /= static_cast<double>(m_numPoints[layer]);
			m_meanDist[layer] /= static_cast<double>(m_numPoints[layer]);
		}
	}
	
	
	// Show statistics
	printInfoMessage(" ", true); 
	printInfoMessage("Statistics for " + toString(m_numScansToCount) + " scans at angle " +
						toString(m_wantedHorzAngle * rad2deg, 1) + " degrees.", true); 

	// 4 or 8 layers?
	INT16 startLayer = 3;
	if (m_isFourLayerScanner == false)
	{
		// 8 layers
		startLayer = 7;
	}
	
	for (INT16 layer = startLayer; layer >= 0; layer -= 1)
	{
		printInfoMessage("Layer " + toString(layer + 1) + ": " + toString(m_numPoints[layer]) + " pts;" + 
							" x=" + toString(m_meanXyz[layer].getX(), 2) + " m," +
							" y=" + toString(m_meanXyz[layer].getY(), 2) + " m," +
							" z=" + toString(m_meanXyz[layer].getZ(), 2) + " m," +
							"; dist=" + toString(m_meanDist[layer], 2) + " m.", true);
	}
	printInfoMessage(" ", true); 
}

//
// Returns true if the scan originates from a 4-layer-scanner. 
//
bool LdmrsScanpointCoordinateApp::isFourLayerScanner(Scan* scan)
{
	printInfoMessage("LdmrsScanpointCoordinateApp::isFourLayerScanner: Beam tilt=" + toString(fabs(scan->getScannerInfos().at(0).getBeamTilt() * rad2deg), 1) + " degrees.", m_beVerbose);
	
	// Check beam tilt
	if (fabs(scan->getScannerInfos().at(0).getBeamTilt()) < (0.2 * deg2rad))
	{
		// Beam tilt is near 0, so it is a 4-layer-scanner 
		return true;
	}
	
	return false;
}

//
// Decode and print scan point coordinates.
//
void LdmrsScanpointCoordinateApp::processScan(Scan* scan)
{
	ScanPoint point;
	bool success;
	UINT8 layerOffset = 0;
	
	// 4-layer or 8-layer scanner?
	if (isFourLayerScanner(scan) == true)
	{
		// No beam tilt, it is a 4-layer scanner
		m_isFourLayerScanner = true;
		layerOffset = 0;
	}
	else
	{
		// 8-layer
		m_isFourLayerScanner = false;

		// What layers are we in? Upper or lower 4?
		if (scan->getScannerInfos().at(0).isRearMirrorSide() == true)
		{
			// Rear mirror side is facing upward in 8-layer scanners -> Upper 4 layers
			layerOffset = 4;
		}
		else
		{
			// Lower 4 layers
			layerOffset = 0;
		}
	}
	

	// Find the points at the desired horizontal angle
	for (UINT8 layer = 0; layer < 4; layer++)
	{
		success = getClosestScanpoint(layer, m_wantedHorzAngle, scan, &point);
		if (success == true)
		{
//			printInfoMessage("LdmrsScanpointCoordinateApp::processScan: Point found, hAngle is " + toString(point.getHAngle() * rad2deg, 1) +
//			" degrees in layer " + toString(layer) + ".", true);
		}
		else
		{
			printInfoMessage("LdmrsScanpointCoordinateApp::processScan: No point found for hAngle " + toString(m_wantedHorzAngle * rad2deg, 1) +
			" degrees in layer " + toString(layer) + ".", true);
		}

		// Sum up the point coordinates
		m_numPoints[layer + layerOffset]++;
		m_meanXyz[layer + layerOffset] += point.toPoint3D();
		m_meanDist[layer + layerOffset] += point.getDist();
	}
}

//
// Searches the scan for the best matching scanpoint in the given layer and close to the given horizontal scan angle.
// If no point is found within 2 degrees around the wanted angle, no point is returned.
// Returns true if a point was found, false if not.
//
bool LdmrsScanpointCoordinateApp::getClosestScanpoint(const UINT8 layer, const double horzAngle, const Scan* scan, ScanPoint* point)
{
	UINT32 i;
	ScanPoint bestPoint;
	ScanPoint currentPoint;
	bool success;
	
	// Set coordinates to a point far, far away
	bestPoint.setPolar(1000, 90 * deg2rad, 90 * deg2rad);
	
	// Search for best match to search parameters
	for (i=0; i<scan->getNumPoints(); i++)
	{
		currentPoint = scan->at(i);
		if (currentPoint.getLayer() == layer)
		{
//			printInfoMessage("Checking angle " + toString(currentPoint.getHAngle() * rad2deg, 1) + " against wanted " +
//								toString(horzAngle * rad2deg, 1) + ".", true); 
			if (fabs(currentPoint.getHAngle() - horzAngle) < fabs(bestPoint.getHAngle() - horzAngle))
			{
				bestPoint = currentPoint;
//				printInfoMessage("Best point yet!", true); 
			}
		}
	}
	
	// Is the match good enough?
	if (fabs(bestPoint.getHAngle() - horzAngle) < (2.0 * deg2rad))
	{
		// Good enough
		*point = bestPoint;
		success = true;
	}
	else
	{
		// No match found
		success = false;
	}
	
	return success;
}

}	// namespace application
