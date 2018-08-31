// This is -*-c++-*-
//
//
// Polygon3d.hpp
//

#ifndef POLYGON3D_HPP
#define POLYGON3D_HPP

#include "../BasicDatatypes.hpp"
#include <iosfwd> // for istream, ostream
#include <vector>
#include "Point3D.hpp"


namespace datatypes
{
	
//
// A polygon of 3D-points.
//
// The points can be accessed through the std::vector interface.
class Polygon3D : public std::vector<Point3D>,
								public BasicData
{
public:
	/// The base type. (Naming according to boost convention.)
	typedef std::vector<Point3D> base_class;

	/// The type of the stored x, y coordinates of each of the points
	/// in this polygon.
	typedef Point3D::floatingpoint_type floatingpoint_type;

	/// Constructor for an empty polygon.
	Polygon3D();

	/// Convenience constructor for a polygon with one Point
	Polygon3D(const Point3D&);

	/// Convenience constructor for a polygon with two Points
	Polygon3D(const Point3D&, const Point3D&);

	/// Convenience constructor for a polygon with three Points
	Polygon3D(const Point3D&, const Point3D&, const Point3D&);

	/// Copy constructor from a std::vector.
	Polygon3D(const base_class&);

	// Estimate the memory usage of this object
	inline virtual const UINT32 getUsedMemory() const {return ((sizeof(*this)) + sizeof(Point3D)*size());};

	void sortForAngleXYPlane();

	/// Equality predicate
	bool operator==(const Polygon3D& other) const
	{
		return static_cast<const base_class&>(*this)
			   == static_cast<const base_class&>(other);
	}

	/**
	 * DEPRECATED - the semantics of this function is very very
	 * unclear and it will be removed soon.
	 *
	 * Get a polygon which has a constant distance to this polygon.
	 * This function is intended to create a path with a more or less constant width.
	 * The polygon should not have succeeding points with reversing direction (zick-zack).
	 * This function operates in the x-y-plane only.
	 */
	Polygon3D getMovedPath(double dist) const;

	/** \name Serialization */
	//\{

	std::istream& read (std::istream& is, UINT32 version); 	///< Reads this object from an input stream
	void read (const BYTE*& buf, UINT32 version);			///< Reads this object from a memory buffer
	std::ostream& write (std::ostream& os, UINT32 version) const; ///< Writes this object to an output stream
	void write (BYTE*& buf, UINT32 version) const;			///< Writes this object to a memory buffer and increments the buffer pointer
	std::streamsize getSerializedSize(UINT32 version) const;	///< Returns the number of bytes this object needs in serialized form
	std::string toString() const;	///< Text output for debugging; opposite of fromString().

	/** \brief Fill this Polygon3D by parsing the given string and
	 * extracting a list of points from the string.
	 *
	 * This is the opposite of toString().
	 *
	 * Syntax example: (0.0 -2.0 0) (10 -1 0) (10 1 0) (0.0 2.0 0) (0.0 -2.0 0)
	 *
	 * This results in a polygon with five points where the last is
	 * identical to the first so that the polygon is a closed polygon.
	 *
	 * Internal syntax rules: The polygon is parsed from a list of
	 * floating point literals, separated by any of the following
	 * characters: &quot; [ ( , ; ) ] as given by
	 * Polygon2D::getSeparatorCharacters(). If the number of floating
	 * point literals is not divisible by three, a std::runtime_error
	 * is thrown.
	 */
	void fromString(const std::string& polyString);

	//\}
};

std::ostream& operator<<(std::ostream& os, const Polygon3D& point); ///< Text output for debugging

}	// namespace datatypes

#endif
