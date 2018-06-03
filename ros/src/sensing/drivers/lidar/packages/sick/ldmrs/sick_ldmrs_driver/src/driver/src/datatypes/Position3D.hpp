//
// Position3D.hpp
//

#ifndef POSITION3D_HPP
#define POSITION3D_HPP

#include "../BasicDatatypes.hpp"
#include "Point3D.hpp"
#include "../tools/errorhandler.hpp"

namespace datatypes
{
	
class Point2D;
//class Point3D;

class Matrix;

	
//
// ***********************************************
//
class Vector
{
public:
	Vector(UINT16 numOfElements);
	~Vector();
	
	inline const double operator[](const UINT16 elementNr) const;
	inline double& operator[](const UINT16 elementNr);
	inline Vector& operator=(const Vector& in);
//	inline const Vector operator*(Matrix& matrix);
	
private:
	UINT16 m_numOfElements;
	double* m_elements;
};

// Keine Pruefung des Index wg. Laufzeit
inline Vector& Vector::operator=(const Vector& in)
{
	if (in.m_numOfElements != m_numOfElements)
	{
		dieWithError("Vector::operator=: Vector size does not match!");
		return (*this);
	}
	
	for (UINT16 i = 0; i<m_numOfElements; i++)
	{
		m_elements[i] = in[i];
	}
	return (*this);
}

// Keine Pruefung des Index wg. Laufzeit
inline const double Vector::operator[](const UINT16 elementNr) const
{
	return (m_elements[elementNr]);
}

// Fuer v[p] = x
inline double& Vector::operator[](const UINT16 elementNr)
{
	return (m_elements[elementNr]);
}

class Matrix
{
public:
	Matrix(UINT16 numOfRows, UINT16 numOfColumns);
	~Matrix();
	
	inline const double operator()(const UINT16 rowNr, const UINT16 columnNr) const;
	inline double& operator()(const UINT16 rowNr, const UINT16 columnNr);
	inline const Vector operator*(Vector& vector);

private:
	UINT16 m_numOfRows;
	UINT16 m_numOfColumns;
	double** m_elements;
};

// Fuer x = m(r,c);
// Keine Pruefung des Index wg. Laufzeit
inline const double Matrix::operator()(const UINT16 rowNr, const UINT16 columnNr) const
{
	return (m_elements[rowNr][columnNr]);
}

// Fuer m(r,c) = x;
// Keine Pruefung des Index wg. Laufzeit
inline double& Matrix::operator()(const UINT16 rowNr, const UINT16 columnNr)
{
	return (m_elements[rowNr][columnNr]);
}

// Fuer v2 = m * v1;
// Vektor-Laenge und Spaltenanzahl der Matrix muessen gleich sein!
inline const Vector Matrix::operator*(Vector& vector)
{
	UINT16 row, col;
	double sum;
	Vector result(m_numOfRows);
	
	for (row = 0; row < m_numOfRows; row++)
	{
		sum = 0.0;
		for (col=0; col < m_numOfColumns; col++)
		{
			sum += m_elements[row][col] * vector[col];
		}
		result[row] = sum;
	}
	
	return result;
}


// Fuer v2 = v1 * m;
/*
inline const Vector Vector::operator*(Matrix& matrix)
{
	UINT16 row, col;
	double sum;
	Vector result(m_numOfElements);
	
	for (row = 0; row < m_numOfElements; row++)
	{
		sum = 0.0;
		for (col=0; col < m_numOfElements; col++)
		{
			sum += matrix(row, col) * m_elements[col];
		}
		result[row] = sum;
	}
	
	return result;
}
*/

//
// *************************************************
//

/// A Position with orientation
/**
 */
class Position3D : public BasicData
{
	typedef double value_type;
	
public:
	/// Empty constructor.
	/**
	 * All values are initialized to zero.
	 */
	Position3D();

	/// Constructor with all values given.
	/**
	 * (Note: The angles are not yet normalized into the interval
	 * [-pi,pi].  If you require normalized angles here, you must call
	 * normalizeAngles() yourself.)
	 */
	Position3D(value_type yaw, value_type pitch, value_type roll,
					 value_type x, value_type y, value_type z);

	/// Constructor with all values given by a Point3D
	/**
	 * Note: The Point3D contains the values in double
	 * precision, but in this class they will be stored in float
	 * precision only.
	 *
	 * (Note: The angles are not yet normalized into the interval
	 * [-pi,pi].  If you require normalized angles here, you must call
	 * normalizeAngles() yourself.)
	 */
	Position3D(value_type yaw, value_type pitch, value_type roll,
					 const Point3D& point);

	// Estimate the memory usage of this object
	inline virtual const UINT32 getUsedMemory() const {return sizeof(*this);};
	
	// Die eigene Position3D wird als Laserscanner-Mountingposition interpretiert
	bool transformToVehicle(Point3D* pt);	// double *dXPos, double *dYPos, double *dZPos, const CRect& r)
	
	
	/// Equality predicate
	bool operator==(const Position3D& other) const;

	/// Yaw angle [rad] of the sensor in the vehicle coordinate system
	value_type getYawAngle() const { return m_yawAngle; }
	/// Pitch angle [rad] of the sensor in the vehicle coordinate system
	value_type getPitchAngle() const { return m_pitchAngle; }
	/// Roll angle [rad] of the sensor in the vehicle coordinate system
	value_type getRollAngle() const { return m_rollAngle; }

	/// x-coordinate [m] of the sensor in the vehicle coordinate system
	value_type getX() const { return m_point.getX(); }
	/// y-coordinate [m] of the sensor in the vehicle coordinate system
	value_type getY() const { return m_point.getY(); }
	/// z-coordinate [m] of the sensor in the vehicle coordinate system
	value_type getZ() const { return m_point.getZ(); }

	/// Returns the content of this class, converted into a Position3D object.
	Position3D toPosition3D() const;

	/// Returns the x/y/z components of this class, converted into a Point3D object.
	Point3D toPoint3D() const;

	/// Returns the x/y components of this class, converted into a Point2D object.
	Point2D toPoint2D() const;

	/// x-coordinate [m] of the sensor in the vehicle coordinate system
	void setX(value_type x) { m_point.setX(x); }
	/// y-coordinate [m] of the sensor in the vehicle coordinate system
	void setY(value_type y) { m_point.setY(y); }
	/// z-coordinate [m] of the sensor in the vehicle coordinate system
	void setZ(value_type z) { m_point.setZ(z); }
	/// Yaw angle [rad] of the sensor in the vehicle coordinate system
	void setYawAngle(value_type angle) { m_yawAngle = angle; }
	/// Pitch angle [rad] of the sensor in the vehicle coordinate system
	void setPitchAngle(value_type angle) { m_pitchAngle = angle; }
	/// Roll angle [rad] of the sensor in the vehicle coordinate system
	void setRollAngle(value_type angle) { m_rollAngle = angle; }

	/// Set all values.
	/**
	 * (Note: The angles are not yet normalized into the interval
	 * [-pi,pi].  If you require normalized angles here, you must call
	 * normalizeAngles() yourself.)
	 *
	 * \return A reference to this object after the new values
	 * have been set.
	 */
	Position3D& set(value_type yaw, value_type pitch, value_type roll,
						  value_type x, value_type y, value_type z);

	/// Normalize all three internal angles to the range [-PI..PI]
	void normalizeAngles();

	// For debugging output: Conversion to string.
	std::string toString() const;

private:
	// Orientierung im 3D-Raum
	/// Yaw angle [rad]
	value_type m_yawAngle;
	/// Pitch angle [rad]
	value_type m_pitchAngle;
	/// Roll angle [rad]
	value_type m_rollAngle;

	Point3D m_point;	// Koordinaten im 3D-Raum
};

}	// namespace datatypes


#endif // POSITION3D_HPP
