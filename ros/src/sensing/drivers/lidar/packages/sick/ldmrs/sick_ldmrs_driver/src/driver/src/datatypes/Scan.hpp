//
// Scan.hpp
//
// A simple data structure for scan data.
//

#ifndef SCAN_HPP
#define SCAN_HPP

#include <vector>
#include "ScannerInfo.hpp"
#include "ScanPoint.hpp"
#include "../BasicDatatypes.hpp"

namespace datatypes
{
	
//
// Sehr einfacher Container fuer Scandaten.
//
// Ein Objekt dieser Klasse repraesentiert einen Einzelscan eines
// Laserscanners.
//
class Scan : public BasicData
{
public:
	// Type of the scan point list
	typedef std::vector<ScanPoint> PointList;

	// Type of the scan point list
	typedef std::vector<ScannerInfo> ScannerInfoVector;

	// Type of a reference to an element
	typedef ScanPoint& reference;

	/// Type of a const reference to an element
	typedef const ScanPoint& const_reference;

	// Type of container size
	typedef PointList::size_type size_type;

	// Type of constant iterator
	typedef PointList::const_iterator const_iterator;

	// Type of iterator
	typedef PointList::iterator iterator;

	// Property flags
	enum ScanFlags
	{
		FlagVehicleCoordinates  = 0x00000800    ///< Bit 11: Scanpoint coordinate system; 0 = scanner coordinates, 1 = vehicle / reference coordinates
	};

	// Estimate the memory usage of this object
	virtual const UINT32 getUsedMemory() const;

protected:
	// Properties of this scan.
	// See ScanFlags for a list of available property flags.
	UINT32 m_flags;

	// Consecutive scan number
	UINT16 m_scanNumber;

	// The list of scan points
	//
	// At Scan creation, enough points are reserved in this vector in
	// order to avoid memory re-allocations during runtime. This is
	// now all handled by std::vector<> directly.
	PointList m_points;

	/// The ScannerInfo collection.
	ScannerInfoVector m_scannerInfos;

	// WATCH OUT: If you add more member variables here, you MUST also
	// adapt the method Scan::copy(), addScan() and add the variables there.

	// ////////////////////////////////////////////////////////////

public:

	// Constructor
	Scan (size_type maxPoints = 5280); // 5280 = Maxpoints with 1/8deg resolution, for 50- -60deg

	/** Copy constructor. Copies the given right hand side object
	 * into this object. An alias for copy(). */
	Scan (const Scan&);

	// Copy the given right hand side object into this object. An
	// alias for copy().
	Scan& operator= (const Scan&);

	// Copy the given right hand side object into this object.
	Scan& copy(const Scan&);

	/// Default destructor
	~Scan();

	/// Resets all members of this object
	void clear ();


	// Equality predicate
//	bool operator==(const Scan& other) const;

	// Returns the number of points in this scan. The same number can be retrieved by size().
	UINT16 getNumPoints() const { return size(); }

	// Returns the number of points in the scan. An alias for getNumPoints().
	size_type size() const { return m_points.size(); }

	// Returns true if this scan contains no points.
	bool empty() const { return m_points.empty(); }

	/** \brief  Resizes the scan to the specified number of points.
	 *  \param  new_size  Number of points the scan should contain.
	 *  \param  default_point  Data with which new elements should be populated.
	 *
	 * This function will resize the Scan to the specified number of
	 * scan points.  If the number is smaller than the scan's current
	 * size the scan is truncated, otherwise the scan is extended and
	 * new elements are populated with given data.
	 *
	 * A Scan must not hold more than 0xFFFF scan points (65536 in
	 * decimal), which is the maximum number an UINT16 can hold. If
	 * the \c new_capacity is larger than this value (65536), a
	 * std::out_of_range exception will be thrown.
	 *
	 * (Naming scheme according to std::vector.) */
	void resize(size_type new_size, const ScanPoint& default_point = ScanPoint());

	// Returns the size of the memory (in number of points) allocated.
	//
	// Note: This is probably not the number of currently available
	// scan points! See size() or getNumPoints() for this.
	//
	size_type capacity() const { return m_points.capacity(); }

	/** \brief Allocates memory for a total of new_capacity points.
	 *
	 * A Scan must not hold more than 0xFFFF scan points (65536 in
	 * decimal), which is the maximum number an UINT16 can hold. If
	 * the \c new_capacity is larger than this value (65536), a
	 * std::out_of_range exception will be thrown.
	 *
	 * (Naming scheme according to std::vector.) */
	void reserve(size_type new_capacity);



	// Returns the consecutive number of this scan.
	// Intended to detect missing scans.
	UINT16 getScanNumber() const { return (UINT16)m_scanNumber; }

	// Set the consecutive number of this scan.
	void setScanNumber	  (UINT16 val) { m_scanNumber	 = val; }


	// Returns the total size of the current scan object, in [bytes]. This value is the actual memory
	// usage, including all sub-structures. Intended usage: Find out how much memory will be used when
	// storing this object in a buffer.
	//
	// Note that the returned value is an estimation that may not match up to the last byte...
	UINT32 getTotalObjectSize();

	// Returns all property flags of this scan in one number
	// isGroundLabeled(), isDirtLabeled(), isRainLabeled() etc.
	UINT32 getFlags() const { return m_flags; }

	// Returns true if the ground detection ran on this scan.
//	bool isGroundLabeled()	 const { return ((m_flags & FlagGroundLabeled) != 0); }

	// Returns true if the dirt detection ran on this scan.
//	bool isDirtLabeled() 	 const { return ((m_flags & FlagDirtLabeled) != 0); }

	// Returns true if the background detection ran on this scan.
//	bool isBackgroundLabeled() const { return ((m_flags & FlagBackgroundLabeled) != 0); }

	// Returns true if the scan points are given in vehicle coordinates, or false if the scan points are given in the Laserscanner coordinate system.
//	bool isVehicleCoordinates() const { return ((m_flags & FlagVehicleCoordinates) != 0); }

	// Set all flags in one number
	void setFlags		  (UINT32 val) { m_flags		 = val; }

	// Clears the given processing flag in the scan.
	//
	// Note: This only changes the flags of the Scan object, not the
	// flags in the individual scan points!  The flags in the
	// individual points are unchanged and must be modified in a
	// manual iteration loop.
	void clearLabelFlag(Scan::ScanFlags scanFlag);

//	void setGroundLabeled(bool isGroundLabeled = true); ///< Set the Scan has ground labeled.
//	void setDirtLabeled(bool isDirtLabeled = true); ///< Set the Scan has dirt labeled.
//	void setRainLabeled(bool isRainLabeled = true); ///< Set the Scan has rain labeled.
//	void setCoverageLabeled(bool isCoverageLabeled = true); ///< Set the Scan has coverage labeled.
//	void setBackgroundLabeled(bool isBackgroundLabeled = true); ///< Set the Scan has background labeled.
//	void setReflectorLabeled(bool isReflectorLabeled = true); ///< Set the Scan has reflector labeled.

	/// Set whether the scanpoints are given in vehicle coordinates
	void setVehicleCoordinates(bool inVehicleCoordinates);	


	// Returns the list of scan points (read only)
	const PointList& getPointList() const { return m_points; }
	// Returns the list of scan points (read/write)
	PointList& getPointList() { return m_points; }

	// Returns the iterator of the first scan point (read only)
	PointList::const_iterator getPointListBegin() const { return m_points.begin(); }
	// Returns the iterator of the first scan point (read/write)
	PointList::iterator getPointListBegin() { return m_points.begin(); }

	// Returns the iterator of one behind the last scan point (read only)
	PointList::const_iterator getPointListEnd() const { return m_points.end(); }

	// Returns the iterator of one behind the last scan point (read/write)
	PointList::iterator getPointListEnd() { return m_points.end(); }

	// Returns the iterator of the first scan point (read only)
	const_iterator begin() const { return m_points.begin(); }

	// Returns the iterator of the first scan point (read/write)
	iterator begin() { return m_points.begin(); }

	// Returns the iterator of one behind the last scan point (read only)
	const_iterator end() const { return m_points.end(); }

	// Returns the iterator of one behind the last scan point (read/write)
	iterator end() { return m_points.end(); }

	// Returns the n-th scan point (read/write). No range checking to avoid runtime costs.
	reference operator[](size_type n) { return m_points[n]; }

	// Returns the n-th scan point (read only). No range checking to avoid runtime costs.
	const_reference operator[](size_type n) const { return m_points[n]; }

	// Returns the n-th scan point (read/write) with range checking.
	reference at(size_type n) { return m_points.at(n); }

	/// Returns the n-th scan point (read only) with range checking.
	const_reference at(size_type n) const { return m_points.at(n); }

	// Returns the n-th scan point (read only). No range checking to avoid runtime costs.
	const ScanPoint& getPoint (UINT16 n) const { return m_points[n]; }
	// Returns the n-th scan point (read/write). No range checking to avoid runtime costs.
	ScanPoint& getPoint (UINT16 n) { return m_points[n]; }

	// Adds a new point to the list of scan points
	ScanPoint& addNewPoint();


	// Returns the ScannerInfo collection.
	//
	// If this Scan contains points on 8 Layers (because of the
	// interlaced fusion of an 8-Layer scanner), the returned list
	// contains two ScannerInfo objects with the same deviceID. In all
	// other cases there will be only one ScannerInfo object for each
	// deviceID.
	//
	const ScannerInfoVector& getScannerInfos() const { return m_scannerInfos; }
	ScannerInfoVector& getScannerInfos();
	const ScannerInfo* getScannerInfoByDeviceId(UINT8 id) const;

	// Sets the ScannerInfo collection.
	void setScannerInfos(const ScannerInfoVector& v);


	//
	// Sorts the scan points of this scan by angle in descending
	// order. (Angles from high to low.)
	//
	// The first scan point will be the one with the largest
	// (leftmost) angle. The last scan point will be the one with
	// smallest (rightmost) angle, most probably a negative value.
	//
	// (Is used for the scan fusion.)
	//
	void sort();


	//
	// Coordinate system transformations
	//
	
	void addCartesianOffset(double offsetX, double offsetY, double offsetZ);
	void addPolarOffset(double distOffset, double hAngleOffset, double vAngleOffset);


	/// Transforms this scan (i.e. the scan points) to the vehicle coordinates.
	/**
	 * Note: This operation also calls
	 * sort(), so that the transformed scan is sorted by angle as
	 * well. (Previously, it was not sorted, but apparently everyone
	 * who used this needed the sorting and it got forgotten too
	 * often.) For transformation without sorting, use
	 * transformToVehicleCoordinatesUnsorted().
	 *
	 * \return True if the scan has been converted successfully. False
	 * if no conversion has been done, which could be caused by
	 * missing ScannerInfo information.
	 */
	bool transformToVehicleCoordinates();

	/// Transforms this scan (i.e. the scan points) to the vehicle coordinates.
	/**
	 * This method does not sort the resulting scan. Use
	 * transformToVehicleCoordinates() if transformation with
	 * additional sorting is needed.
	 *
	 *
	 * \return True if the scan has been converted successfully. False
	 * if no conversion has been done, which could be caused by
	 * missing ScannerInfo information.
	 */
	bool transformToVehicleCoordinatesUnsorted();


private:
	
	bool m_beVerbose;
};

}	// namespace datatypes


#endif // SCAN_HPP
