//
// Position3D.cpp
//
// 3D-Punkt mit Orientierung.
//
// SICK AG, 11.11.2011, willhvo
//

#include "Position3D.hpp"
#include "Point2D.hpp"
#include "Point3D.hpp"
#include <cmath>	// for sin, cos, ...
#include <cassert>
#include <iostream>
#include <sstream>


namespace datatypes
{
	
Position3D::Position3D()
	: m_yawAngle(0)
	, m_pitchAngle(0)
	, m_rollAngle(0)
	, m_point(0.0, 0.0, 0.0)
{
}

Position3D::Position3D(value_type yaw, value_type pitch, value_type roll,
								   value_type x, value_type y, value_type z)
	: m_yawAngle(yaw)
	, m_pitchAngle(pitch)
	, m_rollAngle(roll)
	, m_point(x, y, z)
{
}

Position3D::Position3D(value_type yaw, value_type pitch, value_type roll,
								   const Point3D& pt)
	: m_yawAngle(yaw)
	, m_pitchAngle(pitch)
	, m_rollAngle(roll)
	, m_point(pt)
{
}

//
// **********************************************
//

Vector::Vector(UINT16 numOfElements)
{
	if (numOfElements > 0)
	{
		m_elements = new double[numOfElements];
	}
	else
	{
		m_elements = NULL;
	}
	m_numOfElements = numOfElements;
}

Vector::~Vector()
{
	if (m_elements != NULL)
	{
		delete m_elements;
		m_elements = NULL;
	}
}

// --------------------------------------------------

Matrix::Matrix(UINT16 numOfRows, UINT16 numOfColumns)
{
	if ((numOfRows > 0) && (numOfColumns > 0))
	{
		m_elements = new double*[numOfRows];
		for (UINT16 r=0; r < numOfRows; r++)
		{
			m_elements[r] = new double[numOfColumns];
		}
		m_numOfRows = numOfRows;
		m_numOfColumns = numOfColumns;
	}
	else
	{
		m_elements = NULL;
		m_numOfRows = 0;
		m_numOfColumns = 0;
	}
}

Matrix::~Matrix()
{
	if (m_elements != NULL)
	{
		for (UINT16 r=0; r < m_numOfRows; r++)
		{
			delete m_elements[r];
		}
		
		delete m_elements;
		m_elements = NULL;
	}
}

//
// ***********************************************
//

bool Position3D::transformToVehicle(Point3D* pt)	// double *dXPos, double *dYPos, double *dZPos, const CRect& r)
{
	//Punkt in Laserscannkoordinaten
	Vector P_LS(4);

	//Scanpunkt in Scannerkoordinaten
	P_LS[0]	= pt->getX();	// *dXPos;
	P_LS[1]	= pt->getY();	// *dYPos;
	P_LS[2]	= pt->getZ();	// *dZPos;
	P_LS[3] = 1.0;

	// Sensorposition in Carcoordinates
	double dXOffset_LS, dYOffset_LS, dZOffset_LS;

	// Nick-, Gier- und Wankwinkel des Laserscanners
	double dPitch_LS, dYaw_LS, dRoll_LS;

	//Laserscanner to Vehicle transformation Matrix
	Matrix H_LS_Car(4,4);

	dRoll_LS		= m_rollAngle;
	dYaw_LS			= m_yawAngle;
	dPitch_LS		= m_pitchAngle;
	
	dXOffset_LS		= m_point.getX();
	dYOffset_LS		= m_point.getY();
	dZOffset_LS		= m_point.getZ();
		
	//sin und cos der Nick-, Gier- und Wankwinkel des Laserscanners
	double dSPitch_LS	= sin(dPitch_LS);
	double dCPitch_LS	= cos(dPitch_LS);
	double dSYaw_LS		= sin(dYaw_LS);
	double dCYaw_LS		= cos(dYaw_LS);
	double dSRoll_LS	= sin(dRoll_LS);
	double dCRoll_LS	= cos(dRoll_LS);

	//Trafo-Matrix von Laserscannkoordinaten in Fahrzeugkoordinaten
	H_LS_Car(0,0) = dCYaw_LS*dCPitch_LS - dSYaw_LS*dSPitch_LS*dSRoll_LS;
	H_LS_Car(0,1) = -dSYaw_LS*dCRoll_LS;
	H_LS_Car(0,2) = dCYaw_LS*dSPitch_LS + dSYaw_LS*dSRoll_LS*dCPitch_LS;
	H_LS_Car(0,3) = dXOffset_LS;
	
	H_LS_Car(1,0) = dSYaw_LS*dCPitch_LS + dCYaw_LS*dSRoll_LS*dSPitch_LS;
	H_LS_Car(1,1) = dCYaw_LS*dCRoll_LS;
	H_LS_Car(1,2) = dSYaw_LS*dSPitch_LS - dCYaw_LS*dSRoll_LS*dCPitch_LS;
	H_LS_Car(1,3) = dYOffset_LS;
	
	H_LS_Car(2,0) = -dCRoll_LS*dSPitch_LS;
	H_LS_Car(2,1) = dSRoll_LS;
	H_LS_Car(2,2) = dCRoll_LS*dCPitch_LS;
	H_LS_Car(2,3) = dZOffset_LS;
	
	H_LS_Car(3,0) = 0;
	H_LS_Car(3,1) = 0;
	H_LS_Car(3,2) = 0;
	H_LS_Car(3,3) = 1;


	Vector pt_vehicle(4);
	pt_vehicle = H_LS_Car * P_LS;
	pt->setXYZ(pt_vehicle[0], pt_vehicle[1], pt_vehicle[2]);

	return true;
}



bool Position3D::operator==(const Position3D& other) const
{
	return
		m_yawAngle == other.m_yawAngle
		&& m_pitchAngle == other.m_pitchAngle
		&& m_rollAngle == other.m_rollAngle
		&& m_point == other.m_point
		;
}

Position3D& Position3D::set(value_type yaw, value_type pitch, value_type roll,
										value_type x, value_type y, value_type z)
{
	m_yawAngle = yaw;
	m_pitchAngle = pitch;
	m_rollAngle = roll;
	m_point.setXYZ(x, y, z);

	return *this;
}


Point3D Position3D::toPoint3D() const
{
	return m_point;
}

Point2D Position3D::toPoint2D() const
{
	Point2D result(m_point.getX(), m_point.getY());
	return result;
}


std::string Position3D::toString() const
{
	std::ostringstream ostr;
	ostr << "(x " << getX()
		 << ", y " << getY()
		 << ", z " << getZ() << "m"
		 << "; yaw " << getYawAngle() * rad2deg
		 << ", pitch " << getPitchAngle() * rad2deg
		 << ", roll " << getRollAngle() * rad2deg
		 << "deg)";
	return ostr.str();
}


void Position3D::normalizeAngles()
{
	m_yawAngle = normalizeRadians(m_yawAngle);
	m_pitchAngle = normalizeRadians(m_pitchAngle);
	m_rollAngle = normalizeRadians(m_rollAngle);
}

}	// namespace datatypes
