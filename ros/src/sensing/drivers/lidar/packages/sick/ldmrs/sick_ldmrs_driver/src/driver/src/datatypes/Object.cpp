//
// Object.cpp
//

#include "Object.hpp"
#include "../tools/errorhandler.hpp"
#include "../tools/MathToolbox.hpp"	// for NaN_double

namespace datatypes
{


Object::Object()
	: m_objectId(0)
	, m_flags(0)
	, m_objectAge(0)
	, m_hiddenStatusAge(0)
	, m_timestamp()
	, m_classification(Object::Unclassified)
	, m_classificationAge(0)
	, m_classificationQuality(0.0f)
	, m_centerPoint(Point2D(0, 0))
	, m_centerPointSigma(Point2D(0, 0))
	, m_courseAngle(0.0f)
	, m_courseAngleSigma(0.0f)
	, m_relativeVelocity(Point2D(0, 0))
	, m_relativeVelocitySigma(Point2D(0, 0))
	, m_absoluteVelocity(Point2D(0, 0))
	, m_absoluteVelocitySigma(Point2D(0, 0))
	, m_objectBox(Point2D(0, 0))
	, m_objectBoxSigma(Point2D(0, 0))
	, m_boundingBoxCenter(Point2D(0, 0))
	, m_boundingBox(Point2D(0, 0))
	, m_closestPoint(Point2D(0, 0))
	, m_contourPoints()
	, m_vehicleWLANid(0)
	, m_objectHeight(0)
	, m_objectHeightSigma(0)
	, m_objectMass(0)
	, m_maxAbsoluteVelocity(NaN_double)
	, m_normalizedMeanPointDist(0)
	, m_totalTrackingDuration(0)
	, m_totalTrackedPathLength(0)
	, m_isValid(true)
{
}

Object::~Object()
{
}

bool Object::operator==(const Object& other) const
{
	bool b =
		m_objectId == other.m_objectId
		&& m_flags == other.m_flags
		&& m_objectAge == other.m_objectAge
		&& m_hiddenStatusAge == other.m_hiddenStatusAge
		&& m_timestamp == other.m_timestamp
		&& m_classification == other.m_classification
		&& m_classificationAge == other.m_classificationAge
		&& m_classificationQuality == other.m_classificationQuality
		&& m_centerPoint == other.m_centerPoint
		&& m_centerPointSigma == other.m_centerPointSigma
		&& m_courseAngle == other.m_courseAngle
		&& m_courseAngleSigma == other.m_courseAngleSigma
		&& m_relativeVelocity == other.m_relativeVelocity
		&& m_relativeVelocitySigma == other.m_relativeVelocitySigma
		&& m_absoluteVelocity == other.m_absoluteVelocity
		&& m_absoluteVelocitySigma == other.m_absoluteVelocitySigma
		&& m_objectBox == other.m_objectBox
		&& m_objectBoxSigma == other.m_objectBoxSigma
		&& m_boundingBoxCenter == other.m_boundingBoxCenter
		&& m_boundingBox == other.m_boundingBox
		&& m_closestPoint == other.m_closestPoint
		&& m_contourPoints == other.m_contourPoints
		&& m_objectHeight == other.m_objectHeight
		&& m_objectHeightSigma == other.m_objectHeightSigma
		&& m_objectMass == other.m_objectMass
		;
	return b;
}


Box2D Object::getBox() const
{
	return Box2D(getCenterPoint(), getObjectBox(), getCourseAngle());
}

// The set-functions with checks for value ranges
void Object::setClassificationQuality(double v)
{
	assert(v >= 0);
	assert(v <= 1);
	m_classificationQuality = v;
}

void Object::setCenterPointSigma(const Point2D& v)
{
	assert((v.getX() >= 0) || isNaN(v.getX()));
	assert((v.getY() >= 0) || isNaN(v.getY()));
	m_centerPointSigma = v;
}

void Object::setCourseAngle(double new_angle)
{
	double normd_angle = normalizeRadians(new_angle);
	if (!fuzzyCompare(normd_angle, new_angle))
	{
		printWarning("Object::setCourseAngle was called with angle " + ::toString(new_angle, 2) +
						" which is outside of its [-pi,pi] definition range (but the calling function should have ensured to normalize into that interval) - normalizing it back into that range now (" +
						::toString(normd_angle, 2) + ").");
		new_angle = normd_angle;
		// Note: We intentionally do not throw an exception here
		// because erroneous normalizations might occur very seldomly
		// (especially the exact value of +PI), hence might go through
		// development unnoticed, but an exception causes a crash in
		// front of the customer, which is bad. Also, we *do* print
		// the warning even though we also normalize it so that the
		// developers have a chance to notice this problem and fix
		// their calling code.
	}
	m_courseAngle = new_angle;
}

void Object::setCourseAngleSigma(double v)
{
	assert(v >= 0);
	m_courseAngleSigma = v;
}

void Object::setAbsoluteVelocity(const Point2D&  v)
{
	m_absoluteVelocity = v;
	double absolute = v.dist();
	if (isNaN(getMaxAbsoluteVelocity())
		|| absolute > getMaxAbsoluteVelocity())
	{
		setMaxAbsoluteVelocity(absolute);
	}
}

void Object::setAbsoluteVelocitySigma(const Point2D& v)
{
	assert((v.getX() >= 0) || isNaN(v.getX()));
	assert((v.getY() >= 0) || isNaN(v.getY()));
	m_absoluteVelocitySigma = v;
}

void Object::setObjectBox(const Point2D& v)
{
	assert((v.getX() >= 0) || isNaN(v.getX()));
	assert((v.getY() >= 0) || isNaN(v.getY()));
	m_objectBox = v;
}

void Object::setObjectBoxSigma(const Point2D& v)
{
	assert((v.getX() >= 0) || isNaN(v.getX()));
	assert((v.getY() >= 0) || isNaN(v.getY()));
	m_objectBoxSigma = v;
}

void Object::setBoundingBox(const Point2D& v)
{
	assert((v.getX() >= 0) || isNaN(v.getX()));
	assert((v.getY() >= 0) || isNaN(v.getY()));
	m_boundingBox = v;
}

void Object::setBoundingBoxCenter(const Point2D& v)
{
	m_boundingBoxCenter = v;
}

void Object::setObjectHeightSigma(double v)
{
	assert(v >= 0);
	m_objectHeightSigma = v;
}

void Object::setObjectMass(double v)
{
	assert(v >= 0);
	m_objectMass = v;
}

void Object::setContourPoints(const Polygon2D& v)
{
	assert(v.size() <= 0xff);
	m_contourPoints = v;
}

void Object::addContourPoint(const Point2D cp)
{
	assert(m_contourPoints.size() < 0xff);
	m_contourPoints.push_back(cp);
}


std::string Object::toString() const
{
	std::string text = "Object: ";
	text += "id=" + ::toString(m_objectId);
	text += ", flags=" + ::toString(m_flags);
	text += ", objAge=" + ::toString(m_objectAge);
	text += ", hiddenStatusAge=" + ::toString(m_hiddenStatusAge);
	text += ", timestamp=" + m_timestamp.toString();
	text += std::string(", classification=") + objectClassificationToString(m_classification);

	text += ", classificationAge=" + ::toString(m_classificationAge);

	text += ", classificationQuality=" + ::toString(m_classificationQuality, 2);

	text += ", centerPoint=" + m_centerPoint.toString();
	text += ", centerPointSigma=" + m_centerPointSigma.toString();

	text += ", courseAngle=" + ::toString(m_courseAngle, 2);
	text += ", courseAngleSigma=" + ::toString(m_courseAngleSigma, 2);

	text += ", relativeVelocity=" + m_relativeVelocity.toString();
	text += ", relativeVelocitySigma=" + m_relativeVelocitySigma.toString();

	text += ", absoluteVelocity=" + m_absoluteVelocity.toString();
	text += ", absoluteVelocitySigma=" + m_absoluteVelocitySigma.toString();

	text += ", objectBox=" + m_objectBox.toString();
	text += ", objectBoxSigma=" + m_objectBoxSigma.toString();

	text += ", boundingBox=" + m_boundingBox.toString();
	text += ", closestPoint=" + m_closestPoint.toString();

	text += ", contourPointsNum=" + ::toString(m_contourPoints.size());
	text += ", contourPoints=" + Polygon2D(m_contourPoints).toString();

	text += ", vehicleWLANid=" + ::toString(m_vehicleWLANid, 2);
	text += ", objectHeight=" + ::toString(m_objectHeight, 2);
	text += ", objectHeightSigma=" + ::toString(m_objectHeightSigma, 2);
	text += ", objectMass=" + ::toString(m_objectMass, 2);

	text += ", maxAbsoluteVelocity=" + ::toString(m_maxAbsoluteVelocity, 2);
	text += ", normalizedMeanPointDist=" + ::toString(m_normalizedMeanPointDist, 2);
	text += ", totalTrackingDuration=" + ::toString(m_totalTrackingDuration, 2);
	text += ", totalTrackedPathLength=" + ::toString(m_totalTrackedPathLength, 2);

	return text;
}

#define TYPE_TO_STR(tstr) tstr : return #tstr

const char* Object::objectClassificationToString(ObjectClassification v)
{
	switch (v)
	{
	case TYPE_TO_STR ( Unclassified );
	case TYPE_TO_STR ( UnknownSmall );
	case TYPE_TO_STR ( UnknownBig );
	case TYPE_TO_STR ( Pedestrian );
	case TYPE_TO_STR ( Bike );
	case TYPE_TO_STR ( Car );
	case TYPE_TO_STR ( Truck );
	case TYPE_TO_STR ( Structure_Pylon );
	case TYPE_TO_STR ( Structure_Beacon );
	case TYPE_TO_STR ( Structure_GuardRail );
	case TYPE_TO_STR ( Structure_ConcreteBarrier );
	case TYPE_TO_STR ( NumClasses );
	case TYPE_TO_STR ( Unknown );
	default:
		throw std::invalid_argument("Unknown object classification " + ::toString(int(v)));
		return "<unknown>";
	}
}

Object::ObjectClassification Object::stringToObjectClassification(const std::string& s)
{
	if (s == "Unclassified" || s == "Unclass")
		return Unclassified;
	else if (s == "UnknownSmall" || s == "us")
		return UnknownSmall;
	else if (s == "UnknownBig" || s == "UB")
		return UnknownBig;
	else if (s == "Pedestrian" || s == "Ped")
		return Pedestrian;
	else if (s == "Bike")
		return Bike;
	else if (s == "Car")
		return Car;
	else if (s == "Truck")
		return Truck;
	else if (s == "Structure_Pylon" || s == "Pylon")
		return Structure_Pylon;
	else if (s == "Structure_Beacon" || s == "Bcn")
		return Structure_Beacon;
	else if (s == "Structure_GuardRail" || s == "GrdRl")
		return Structure_GuardRail;
	else if (s == "Structure_ConcreteBarrier" || s == "CBar")
		return Structure_ConcreteBarrier;
	else if (s == "NumClasses" || s == "#Classes")
		return NumClasses;
	else if (s == "Unknown")
		return Unknown;
	else
	{
		throw std::invalid_argument("Unknown object classification string \"" + s + "\"");
		return Unknown;
	}
}

std::string Object::objectClassificationToStringWithNum(ObjectClassification v)
{
	const std::string s = objectClassificationToString(v);
	return s + " (" + ::toString((int)v) + ")";
}

const char* Object::objectClassificationToShortString(ObjectClassification v)
{
	switch (v)
	{
	case Unclassified:
		return "Unclass";
	case UnknownSmall:
		return "us";
	case UnknownBig:
		return "UB";
	case Pedestrian:
		return "Ped";
	case Bike:
		return "Bike";
	case Car:
		return "Car";
	case Truck:
		return "Truck";
	case Structure_Pylon:
		return "Pylon";
	case Structure_Beacon:
		return "Bcn";
	case Structure_GuardRail:
		return "GrdRl";
	case Structure_ConcreteBarrier:
		return "CBar";
	case NumClasses:
		return "#Classes";
	case Unknown:
		return "Unknown";
	default:
		throw std::invalid_argument("Unknown object classification " + ::toString(int(v)));
		return "<unknown>";
	}
}

// ////////////////////////////////////////////////////////////

void Object::getObjectBoxVarCovar(double &var_x, double &var_y, double &covar_xy) const
{
	// Square the stored standard deviation to get the variances
	double var_x_obj = sqr(getObjectBoxSigma().getX());
	double var_y_obj = sqr(getObjectBoxSigma().getY());

	// Rotate the variances back by the angle given by the courseAngle
	double dCos = std::cos(getCourseAngle());
	double dSin = std::sin(getCourseAngle());
	double cos_sq = sqr(dCos);
	double sin_sq = sqr(dSin);
	double cos_sin = dCos * dSin;

	// And rotate the covariance matrix C_o: To rotate C_o by the
	// rotation matrix R, we need to calculate C_v = R * C_o * R^T
	var_x    =   cos_sq * var_x_obj +  sin_sq * var_y_obj;
	covar_xy = -cos_sin * var_x_obj + cos_sin * var_y_obj;
	var_y    =   sin_sq * var_x_obj +  cos_sq * var_y_obj;
}

/*
void Object::compensateEgoMovement(const Point2D& deltaPos, double deltaAngle)
{
	m_centerPoint -= deltaPos;
	m_centerPoint = m_centerPoint.rotated(-deltaAngle);

	m_centerPointSigma = m_centerPointSigma.rotated(-deltaAngle);

	m_courseAngle -= deltaAngle;
	m_courseAngleSigma -= deltaAngle;

	m_relativeVelocity = m_relativeVelocity.rotated(-deltaAngle);
	m_relativeVelocitySigma = m_relativeVelocitySigma.rotated(-deltaAngle);
	m_absoluteVelocity = m_absoluteVelocity.rotated(-deltaAngle);
	m_absoluteVelocitySigma = m_absoluteVelocitySigma.rotated(-deltaAngle);

	m_boundingBoxCenter -= deltaPos;
	m_boundingBoxCenter = m_boundingBoxCenter.rotated(-deltaAngle);

	// m_boundingBox TODO: anpassen
	m_closestPoint -= deltaPos;
	m_closestPoint = m_closestPoint.rotated(-deltaAngle);

	// transformation of the contour points - otherwise the contour points are invalid!

	Point2DVector::iterator pt;
	for (pt = m_contourPoints.begin(); pt != m_contourPoints.end(); pt++)
	{
		*pt -= deltaPos;
		*pt = pt->rotated(-deltaAngle);
	}
}
*/

void Object::incrementObjectAge()
{
	m_objectAge++;
}

void Object::setMaxAbsoluteVelocity(double v)
{
//	if (v < 0)
//		throw InvalidArgumentException("setMaxAbsoluteVelocity called with negative argument " + ::toString(v, 2) + ", but must be non-negative.");
	m_maxAbsoluteVelocity = v;
}
void Object::setNormalizedMeanPointDist(double v)
{
//	if (v < 0)
//		throw InvalidArgumentException("setNormalizedMeanPointDist called with negative argument " + ::toString(v, 2) + ", but must be non-negative.");
	m_normalizedMeanPointDist = v;
}
void Object::setTotalTrackingDuration(double v)
{
//	if (v < 0)
//		throw InvalidArgumentException ("setTotalTrackingDuration called with negative argument " + ::toString(v, 2) + ", but must be non-negative.");
	m_totalTrackingDuration = v;
}
void Object::setTotalTrackedPathLength(double v)
{
//	if (v < 0)
//		throw InvalidArgumentException("setTotalTrackedPathLength called with negative argument " + ::toString(v,2) + ", but must be non-negative.");
	m_totalTrackedPathLength = v;
}

double Object::getMeanAbsoluteVelocity() const
{
	// mean velocity
	return (m_totalTrackingDuration > 0)
		   ? (m_totalTrackedPathLength / m_totalTrackingDuration)
		   : 0.0f;
}

//////////////////////////////////////////////////////////////
//
// ******************* ObjectList ***************************
//
//////////////////////////////////////////////////////////////

ObjectList::ObjectList()
	: base_class()
{
	m_datatype = Datatype_Objects;
	m_timestamp.set(0.0);
}

bool ObjectList::operator==(const ObjectList& other) const
{
	return (m_timestamp == other.m_timestamp)
		   && (static_cast<const base_class&>(*this)
			   == static_cast<const base_class&>(other));
}


void ObjectList::setTimestamp(const Time& timestamp)
{
	m_timestamp = timestamp;
}

/*
void ObjectList::compensateEgoMovement(const Point2D& deltaPos, double deltaAngle)
{
	iterator obj;
	for (obj = begin(); obj != end(); obj++)
	{
		obj->compensateEgoMovement(deltaPos, deltaAngle);
	}
}
*/

//
// Alter aller Objekte um 1 erhoehen.
//
void ObjectList::incrementObjectAge()
{
	iterator obj;
	for (obj = begin(); obj != end(); obj++)
	{
		obj->incrementObjectAge();
	}
}

}	// namespace datatypes
