//
// Object.hpp
//
// Container for objects.
//


#ifndef OBJECT_HPP
#define OBJECT_HPP


#include <vector>
#include "../BasicDatatypes.hpp"
#include "Point2D.hpp"
#include "Polygon2D.hpp"
#include "Box2D.hpp"
#include "../tools/Time.hpp"

namespace datatypes
{

// Represents a tracked object in our environmental model.
/**
 * This class contains all that is known about objects in the
 * environmental model: %Object position, velocity, course angle, and
 * classification.
 *
 * \image html objectcoords.png "Tracked object at position A, Course Angle Psi, Velocity vector vh, and Size sx, xy"
 *
 * The available information about each object is shown in the figure
 * above.  The host vehicle's axis system is X, Y with the origin at
 * H.  The center point A of the object is given in the host vehicle's
 * axis system X, Y by getCenterPoint(). (In exact ISO 8855
 * terminology, the used axis system X,Y is the host vehicle's
 * "intermediate axis system")
 *
 * The orientation of the object is given by the course angle Psi
 * (\f$\psi\f$) by getCourseAngle(), which is the angle from the host
 * vehicle's X axis to the object's Xo axis. Other names for this
 * angle are the "object's yaw angle" \f$\psi\f$ or heading or
 * orientation.  This defines the object's axis system Xo, Yo.
 *
 * The velocity of the object is given by the velocity vector vh
 * (\f$v_h\f$) by getAbsoluteVelocity(), which gives the object's
 * velocity specified in the host vehicle's axis system. (In exact ISO
 * 8855 terminology, the vector vh is the object's "horizontal
 * velocity".)
 *
 * The size of the object is given by the side lengths sx, sy of the
 * rectangle in the object's axis system by getObjectBox(). Position,
 * orientation, and size are given altogether by getBox().
 *
 * (Note: In exact ISO 8855 terminology, the object's axis system
 * Xo,Yo might point into a slightly different direction than the
 * velocity vector, in which case \f$v_h\f$ is rotated from X_o by the
 * sideslip angle \f$\beta\f$. The rotation of the velocity vector
 * compared to the host vehicle's axis system, the course angle
 * \f$\nu\f$, is then given by \f$\nu=\psi+\beta\f$. However,
 * depending on the used tracking algorithms the sideslip angle is
 * neglected and \f$\nu=\psi\f$, so there should be no difference
 * between the course angle and the yaw angle.)
 *
 * Note: The current %Laserscanner tracking algorithms can fill in only
 * a subset of the data fields that exist in this class. But those
 * additional data fields are needed as soon as we deal with data
 * fusion from WLAN data sources. Since those fusion algorithms are of
 * importance to our algorithms as well, we already have reserved the
 * member variables for those fields.
 *
 * Internal note to self: For the next revision one might consider
 * adding the following data fields:
 *
 * - centerOfGravity as opposed to center of geometry in getCenterPoint()
 * - maybe a list of available referencePoints
 * - maybe an enum of the referencePoint types
 * - centerOfGeometry BoundingBox
 */
class Object
{
public:
	enum ObjectClassification
	{
		Unclassified = 0,
		UnknownSmall = 1,
		UnknownBig = 2,
		Pedestrian = 3,
		Bike = 4,
		Car = 5,
		Truck = 6,
		Structure_Pylon = 7,
		Structure_Beacon = 8,
		Structure_GuardRail = 9,
		Structure_ConcreteBarrier = 10,
		NumClasses,
		Unknown = 15
	};

public:
	Object();
	~Object();

	/// Equality predicate
	bool operator==(const Object& other) const;

	/** Returns the index number of this object.
	 *
	 * Watch out: In some algorithms, the object id 0 (Null) is used
	 * as the special value of a non-valid object, but in other
	 * algorithms the id 0 is regarded as just a normal value as any
	 * other. However, invalid objects should be marked with
	 * setValid() instead of a Null-Value here.
	 */
	UINT16 getObjectId() const { return m_objectId; }

	/** Sets the index number of this object.
	 *
	 * For setting an object to invalid, use setValid().
	 */
	void setObjectId(UINT16 v) { m_objectId = v; }


	/** Returns the flags that have been set in this object. Currently
	 * used bits are as follows: bit#0 = basic information is available;
	 * bit#1 = contour information has been set, bit#2 = boundingBox
	 * has been set; bit#3 = object contains fused data from other
	 * sources (WLAN etc.); bit#4 = relative velocity has been set;
	 * bit#5 = CAN Object Data only (protocol version: 1; either
	 * bounding box or object box is available (see bit#2);
	 * analog see bit#4 if relative or absolute velocity has been set);
	 * bit#6...15 = reserved */
	UINT16 getFlags() const { return m_flags; }
	void setFlags(UINT16 v) { m_flags = v; }

	/** Returns the number of scans in which this object has been
	 * tracked. */
	UINT32 getObjectAge() const { return m_objectAge; }
	void setObjectAge(UINT32 v) { m_objectAge = v; }

	/** Returns the number of scans in which the object has not been
	 * observed by measurement (i.e. it was hidden) but instead it has
	 * only been predicted. */
	UINT16 getHiddenStatusAge() const { return m_hiddenStatusAge; }
	/** Returns true if the object is not being observed in the very
	 * last measurement (i.e. it was hidden) but instead it has only
	 * been predicted. */
	bool isHiddenStatus() const { return m_hiddenStatusAge > 0; }
	void setHiddenStatusAge(UINT16 v) { m_hiddenStatusAge = v; }

	/** Returns the time of when the center point of this object was
	 * observed. */
	const Time& getTimestamp() const { return m_timestamp; }
	void setTimestamp(const Time& v) { m_timestamp = v; }

	/** Returns the object class that is most likely for this
	 * object. */
	ObjectClassification getClassification() const { return m_classification; }
	void setClassification(ObjectClassification v) { m_classification = v; }

	/** Returns the number of scans in which the object has has been
	 * classified in the current classification. */
	UINT32  getClassificationAge() const { return m_classificationAge; }
	void setClassificationAge(UINT32 v) { m_classificationAge = v; }

	/** Returns the quality measure ("Guete") of the current
	 * classification in [0 .. 1]. */
	double  getClassificationQuality() const { return m_classificationQuality; }
	void setClassificationQuality(double v);


	/**
	 * Returns the tracked center point of geometry ("Mittelpunkt") of
	 * this object in [meter], relative to our vehicle's coordinate
	 * system.
	 *
	 * This estimated center point is as close to the actual center
	 * point as possible with the respective tracking algorithm. To be
	 * more precise, if the tracking algorithm tracks the center of
	 * gravity (COG) point, the COG point will be given here instead
	 * of the actual center of geometry. In those cases the actual
	 * center of geometry is unknown, unfortunately.
	 *
	 * More information about the position of the object might be
	 * obtained from the getContourPoints() pointlist or through
	 * accessing the SegmentList by getSegment(), but those will
	 * always give unfiltered (non-tracked) results.
	 */
	const Point2D&  getCenterPoint() const { return m_centerPoint; }
	void setCenterPoint(const Point2D& v) { m_centerPoint = v; }

	/** Returns the standard deviation (i.e. the uncertainty,
	 * "Mittelpunkt-Standardabweichung") of the center point of geometry
	 * estimation of this object, given in Vehicle coordinates in
	 * [meter]. */
	const Point2D&  getCenterPointSigma() const { return m_centerPointSigma; }
	void setCenterPointSigma(const Point2D& v);

	/** Returns the course angle ("Kurswinkel") of this object's
	 * movement in [radian], in the interval [-pi, pi). This is named
	 * conforming to ISO 8855; elsewhere this value is also called the
	 * Orientation or the Heading.
	 *
	 * This angle is the angle from the host vehicle's x-coordinate
	 * axis to the object's x-coordinate axis (which in most cases is
	 * identical to the object's velocity vector).  It is also the sum
	 * of yaw angle ("Gierwinkel") and sideslip angle
	 * ("Schwimmwinkel") of this object.  */
	double  getCourseAngle() const { return m_courseAngle; }

	/** Sets the course angle ("Kurswinkel") of this object's
	 * movement in [radian], in the interval [-pi, pi). This is named
	 * conforming to ISO 8855; elsewhere this value is also called the
	 * Orientation or the Heading.
	 *
	 * If the new course angle is outside of the defined interval
	 * [-pi, pi), a warning message will be printed and the value will
	 * be normalized into that interval by normalizeRadians().
	 */
	void setCourseAngle(double newCourseAngle);

	/** Returns the course angle standard deviation (i.e. the
	 * uncertainty, "Kurswinkel-Standardabweichung") in [radian]. This
	 * is named conforming to ISO 8855; elsewhere this value is also
	 * called the Orientation or the Heading.  */
	double getCourseAngleSigma() const { return m_courseAngleSigma; }
	void setCourseAngleSigma(double v);

	/** (Usually Unused.) Returns the velocity vector ("Geschwindigkeitsvektor") of this
	 * object in [meter/seconds], relative to our vehicle's coordinate
	 * system. Note: The currently implemented tracking will always
	 * track only the absolute velocity; hence, this field
	 * relativeVelocity will be unset and simply be zero (or some other irrelevant values). */
	const Point2D&  getRelativeVelocity() const { return m_relativeVelocity; }
	void setRelativeVelocity(const Point2D&  v) { m_relativeVelocity = v; }

	/** (Usually Unused.) Returns the velocity vector standard deviation (i.e. the
	 * uncertainty) of this object in [meter/seconds], relative to our
	 * vehicle's coordinate system. Note: The currently implemented
	 * tracking will always track only the absolute velocity; hence,
	 * this field relativeVelocity will be unset and simply be
	 * zero (or some other irrelevant values). */
	const Point2D&  getRelativeVelocitySigma() const { return m_relativeVelocitySigma; }
	void setRelativeVelocitySigma(const Point2D& v) { m_relativeVelocitySigma = v; }

	/** Returns the velocity vector ("Geschwindigkeitsvektor") of this
	 * object in [meter/seconds] as absolute value. The orientation is
	 * relative to our vehicle's coordinate system. */
	const Point2D&  getAbsoluteVelocity() const { return m_absoluteVelocity; }
	/** Sets the velocity vector as absolute value. Note: This also updates setMaxAbsoluteVelocity() accordingly. */
	void setAbsoluteVelocity(const Point2D&  v);

	/** Returns the velocity vector standard deviation (i.e. the
	 * uncertainty) of this object in [meter/seconds], absolute. */
	const Point2D&  getAbsoluteVelocitySigma() const { return m_absoluteVelocitySigma; }
	void setAbsoluteVelocitySigma(const Point2D& v);


	/** Returns the estimated size of the object in [meter].
	 *
	 * The returned size estimation models a rotated rectangular box
	 * around the object's center point (hence the name "object
	 * box"). Point2D::getX() returns the size of this object in
	 * x-direction of the object's coordinate system (i.e. the object
	 * length), Point2D::getY() the size in the y-direction (i.e. the
	 * object width). This value is the filtered size estimation of
	 * this object.
	 *
	 * This box contains (bounds) all of this object's scanpoints and
	 * is in parallel to the object's coordinate system, i.e. it is using
	 * the getCourseAngle() orientation. [meter]
	 *
	 * \see getCourseAngle(), getCenterPoint(), setObjectBox()
	 */
	const Point2D& getObjectBox() const { return m_objectBox; }

	/** Set the size of the rectangular box of this object. \see
	 * getObjectBox() */
	void setObjectBox(const Point2D& v);

	/** Returns a rectangular box around the object's center point in
	 * [meter]. This method is just shorthand for obtaining the center
	 * point by getCenterPoint(), the course angle (orientation) by
	 * getCourseAngle(), and the size of the object by
	 * getObjectBox(). Box2D::getSize()::getX() returns the size of
	 * this object in x-direction of the object's coordinate system
	 * (i.e. the object length), Box2D::getSize()::getY() the size of
	 * this object in y-direction (i.e. the object width).
	 *
	 * \see getCenterPoint(), getCenterPoint(), getObjectBox()
	 */
	Box2D getBox() const;

	/** Returns the object size estimation's standard deviation [meter].
	 *
	 * This is given in the object's coordinate system! Watch out for
	 * necessary coordinate transformations (rotations) if you want to
	 * use this value in the host vehicle's coordinate system. */
	const Point2D& getObjectBoxSigma() const { return m_objectBoxSigma; }
	void setObjectBoxSigma(const Point2D& v);

	/** Writes the object box x and y variance (squared standard
	 * deviation) and their covariance into the given variables in the
	 * host vehicle's coordinate system [meter^2].
	 *
	 * In contrast to getObjectBoxSigma(), here the x and y variance
	 * is rotated from the object coordinate system into the host
	 * system. Hence, if there was a zero covariance beforehand, a
	 * non-zero covariance will result after the rotation. */
	void getObjectBoxVarCovar(double &var_x, double &var_y, double &covar_xy) const;

	/** Returns the size of a rectangle around the object's/bounding box center
	 * point that contains (bounds) all of this object's scanpoints,
	 * in parallel to our vehicle's coordinate system axis (also
	 * called a paraxial rectangle). */
	const Point2D& getBoundingBox() const { return m_boundingBox; }
	void setBoundingBox(const Point2D& v);

	/**
	 * Returns the center of the bounding box. \sa{getBoundingBox}
	 */
	const Point2D& getBoundingBoxCenter() const { return m_boundingBoxCenter; }
	void setBoundingBoxCenter(const Point2D& v);

	/** Returns the point of this object that is closest to the origin
	 * of our vehicle's coordinate system.
	 *
	 * If this is not set, returns a zero-valued point.
	 */
	const Point2D& getClosestPoint() const { return m_closestPoint; }
	void setClosestPoint(const Point2D& v) { m_closestPoint = v; }

	/** Returns a vector of points that describes a polygon outline of
	 * the current object's measurement points. */
	const Polygon2D& getContourPoints() const { return m_contourPoints; }
	void setContourPoints(const Polygon2D& v);
	void addContourPoint(const Point2D cp);

	/** An identifier to be used by WLAN fusion algorithms */
	UINT64 getVehicleWLANid() const { return m_vehicleWLANid; }
	void setVehicleWLANid(UINT64 v) { m_vehicleWLANid = v; }

	/** The height of this object in [m] (most probably received
	 * through WLAN data) */
	double getObjectHeight() const { return m_objectHeight; }
	void setObjectHeight(double v) { m_objectHeight = v; }

	/** The standard deviation of the height of this object in [m]
	 * (most probably received through WLAN data) */
	double getObjectHeightSigma() const { return m_objectHeightSigma; }
	void setObjectHeightSigma(double v);

	/** The mass of this object in [kilogram] (as received
	 * e.g. through WLAN data) */
	double getObjectMass() const { return m_objectMass; }
	void setObjectMass(double v);

	/** True, if this object is valid.
	 *
	 * This flag will only be used to decide whether this Object is
	 * included in the serialization, i.e. if an Object has "false"
	 * here, it will not be serialized and will not be received by a
	 * receiver. Hence, this flag by itself is not included in the
	 * serialization.
	 *
	 * Again: Invalid objects (those which return false here) will
	 * *not* be included in the serialization!
	 */
	bool isValid() const { return m_isValid; }

	/** Set whether this object is valid.
	 *
	 * This flag will only be used to decide whether this Object is
	 * included in the serialization, i.e. if an Object has "false"
	 * here, it will not be serialized and will not be received by a
	 * receiver. Hence, this flag by itself is not included in the
	 * serialization.
	 *
	 * Again: Invalid objects (those which have false here) will
	 * *not* be included in the serialization!
	 */
	void setValid(bool newValue = true) { m_isValid = newValue; }

	/** Returns the maximum observed absolute velocity [m/s]
	 * (Classification feature). The value is NaN if it hasn't been
	 * set so far. The value is always non-negative, or NaN. */
	double getMaxAbsoluteVelocity() const { return m_maxAbsoluteVelocity; }
	void setMaxAbsoluteVelocity(double v);

	/** Returns the normalized mean distance [m] between scanpoints in
	 * the segment, or zero if the object is currently hidden.
	 * (Classification feature) Always non-negative. */
	double getNormalizedMeanPointDist() const { return m_normalizedMeanPointDist; }
	void setNormalizedMeanPointDist(double v);

	/** Returns the total duration for which this object has been
	 * tracked in [seconds]. (Classification feature, needed for mean
	 * velocity) Always non-negative. */
	double getTotalTrackingDuration() const { return m_totalTrackingDuration; }
	void setTotalTrackingDuration(double v);

	/** Returns the total path length of object movement that has been
	 * tracked [m]. (Classification feature) Always non-negative. */
	double getTotalTrackedPathLength() const { return m_totalTrackedPathLength; }
	void setTotalTrackedPathLength(double v);

	/** Returns the mean velocity during the whole time over which the
	 * object has been tracked [m/s], which is basically just
	 * getTotalTrackedPathLength() divided by
	 * getTotalTrackingDuration(). Always non-negative. */
	double getMeanAbsoluteVelocity() const;


	// Size of the object in memory
	const UINT32 getUsedMemory() const { return sizeof(Object); };

	/// Size of the serialized representation of this object
	/**
	 * \param version 1,2 == compressed meter values; 3,4 == double values
	 */
	std::streamsize getSerializedSize(UINT32 version) const;

	/** Returns a human-readable form of the content of this object (for debugging output)
	 * The actual output is merely a conversion to string of the
	 * return value of toConfigValues().
	 */
	std::string toString() const;
	//\}

	/// Returns the given classification value as a string.
	static const char* objectClassificationToString(ObjectClassification v);

	/// Returns the given classification value as a string with the integer number included.
	static std::string objectClassificationToStringWithNum(ObjectClassification v);

	/// Returns the given classification value as a short string.
	static const char* objectClassificationToShortString(ObjectClassification v);

	/// Returns the classification value as converted from the given
	/// string. This accepts the output of both
	/// objectClassificationToString() and
	/// objectClassificationToShortString().
	static Object::ObjectClassification stringToObjectClassification(const std::string& s);

	/// Just increment objectAge by one.
	void incrementObjectAge();

private:
	UINT16	m_objectId;
	UINT16  m_flags; ///< reserved
	// Data from the tracking/classification:
	UINT32 	m_objectAge;            ///< number of scans in which this object has been tracked, or instead time?
	UINT16	m_hiddenStatusAge;      ///< Counts how long the object has not been observed but only predicted.
	Time 	m_timestamp; 		///< Time of when the center point of this object was observed.

	ObjectClassification m_classification; ///< The object class that is most likely for this object.
	UINT32  m_classificationAge;           ///< Counts how long the object has been classified in the current classification.
	double   m_classificationQuality;       ///< The quality of the current classification.

	Point2D m_centerPoint;      ///< Center point of object rectangle, given in Vehicle coordinate system.
	Point2D m_centerPointSigma;
	double   m_courseAngle;      ///< named by ISO 8855; also called Orientation or Heading [rad]
	double   m_courseAngleSigma; // in [rad]
	Point2D m_relativeVelocity; ///< Velocity of this object [meter/seconds], relative to the vehicle coordinate system.
	Point2D m_relativeVelocitySigma;
	Point2D m_absoluteVelocity; ///< Velocity of this object [meter/seconds] as absolute velocity; the orientation is relative to the vehicle coordinate system.
	Point2D m_absoluteVelocitySigma;

	Point2D m_objectBox;         ///< The object's length and width as a rectangle, relative to the object's coordinate system.
	Point2D m_objectBoxSigma;
	Point2D m_boundingBoxCenter; ///< Center of the bounding box.
	Point2D m_boundingBox;       ///< A rectangle in parallel to the vehicle coordinate system (a paraxial rectangle) that contains (bounds) all of this object's points

	// These components are also proposed
	Point2D m_closestPoint; ///< The point of this object that is closest to the origin of the vehicle coordinate system.

	// This can also be calculated
	Polygon2D m_contourPoints; ///< A poly-line that describes the outline of the current object measurement.

	UINT64 m_vehicleWLANid;    ///< An identifier to be used by WLAN fusion algorithms.
	double m_objectHeight;      ///< The height of this object in [m] (most probably received through WLAN data).
	double m_objectHeightSigma; ///< The standard deviation of the height of this object in [m] (most probably received through WLAN data).

	double m_objectMass; ///< The mass of this object in [kilogram] (as received e.g. through WLAN data)


	/// Classification feature: The maximum observed absolute velocity [m/s]
	double m_maxAbsoluteVelocity;

	/// Classification feature: Normalized mean distance [m] between
	/// scanpoints in the segment, or zero if the object is currently
	/// hidden.
	double m_normalizedMeanPointDist;

	/// Classification feature (needed for mean velocity): Duration of
	/// object being tracked in [s]
	double m_totalTrackingDuration;

	/// Classification feature: Total accumulated path length over which this
	/// object has moved during being tracked. [m]
	double m_totalTrackedPathLength;

	/** Pointer to the Segment that belongs to this object. (Note:
	 * This assumes that each object was associated to only one
	 * segment. If more than one segment is associated to this object,
	 * we're out of luck and need a new data structure.) */
//	const Segment *m_segment;

	// True, if this object is valid.
	bool m_isValid;
};


// ////////////////////////////////////////////////////////////

//
// List of Objects
//
//
class ObjectList : public std::vector<Object>,
					public BasicData
{
public:

	typedef std::vector<Object> base_class;

	ObjectList();

	/// Equality predicate
	bool operator==(const ObjectList& other) const;

	// Size of the object in memory
	const UINT32 getUsedMemory() const { return sizeof(*this) + size()*sizeof(Object); };

	// Get the timestamp of the object list. Typically it should be the midTimestamp of the corresponding scan.
	const Time& getTimestamp() const { return m_timestamp; }

	// Set the timestamp of the object list. Typically it should be the midTimestamp of the corresponding scan.
	void setTimestamp(const Time& timestamp);

	/// Just increment objectAge of all objects by one.
	void incrementObjectAge();

protected:

	// The timestamp of the ObjectList
	Time m_timestamp;
};

}	// namespace datatypes


#endif // OBJECT_HPP
