/**
 * \file
 * \brief Contains the classes Trigger
 */

#ifndef DATATYPE_TRIGGER_HPP
#define DATATYPE_TRIGGER_HPP


#include "../tools/errorhandler.hpp"
#include <stdexcept>
#include "../BasicDatatypes.hpp"

namespace datatypes
{

/// Class for a simple serializable trigger signal
class Trigger : public BasicData
{
public:
	/// Default constructor
	Trigger ();

	/// Constructor with number and sourceId
	Trigger (UINT32 number, UINT8 sourceId);

	/// Destructor
	virtual ~Trigger();

	// Estimate the memory usage of this object
	virtual const UINT32 getUsedMemory() const { return sizeof(*this); };

	// Returns the number of the Trigger
	UINT32 getNumber() const { return m_number; }

	// Sets the number.
	void setNumber (UINT32 number);

	// For debug
	
	std::string toString();


private:
	UINT32 m_number;  ///< The trigger counter

	////////////////////////////////////////////////////////////////////////////
};


}	// namespace datatypes


#endif // TRIGGER_HPP
