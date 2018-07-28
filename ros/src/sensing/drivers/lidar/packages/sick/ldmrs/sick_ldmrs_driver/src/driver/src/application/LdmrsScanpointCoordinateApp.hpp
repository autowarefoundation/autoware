//
// LdmrsScanpointCoordinateApp.hpp
//

#ifndef LDMRSSCANPOINTCOORDINATEAPP_HPP
#define LDMRSSCANPOINTCOORDINATEAPP_HPP

#include "../manager.hpp"
#include "../datatypes/Scan.hpp"

namespace application
{
	
//
// LdmrsScanpointCoordinateApp
//
class LdmrsScanpointCoordinateApp : public BasicApplication
{
public:
	LdmrsScanpointCoordinateApp(Manager* manager);
	virtual ~LdmrsScanpointCoordinateApp();

protected:
	void setData(BasicData& data);	// Receiver
	
private:
	bool m_beVerbose;
	UINT16 m_scanCounter;
	bool m_isFourLayerScanner;
	UINT16 m_numScansToCount;
	double m_wantedHorzAngle;
	UINT16 m_numPoints[8];
	double m_meanDist[8];
	Point3D m_meanXyz[8];
	
	void initCycle();
	void processScan(Scan* scan);
	bool getClosestScanpoint(const UINT8 layer, const double horzAngle, const Scan* scan, ScanPoint* point);
	void calcAndShowStatistics();
	bool isFourLayerScanner(Scan* scan);
	
};
}	// namespace application

#endif
