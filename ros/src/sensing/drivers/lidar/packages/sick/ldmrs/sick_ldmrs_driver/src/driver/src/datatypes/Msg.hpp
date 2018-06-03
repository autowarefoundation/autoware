// This is -*-c++-*-
//
// Msg.hpp
//

#ifndef MSG_HPP
#define MSG_HPP

#include "../BasicDatatypes.hpp"
#include <iosfwd> // for istream, ostream

namespace datatypes
{
	
//
// A text message.
//
class Msg : public BasicData
{
public:
	/// Constructor
	Msg(const UINT16 sourceId);

	/// Convenience constructor for the message
	Msg(const UINT16 sourceId, const std::string message);

	// Estimate the memory usage of this object
	virtual const UINT32 getUsedMemory() const { return sizeof(*this) + m_msgText.length(); };
	
	std::string getMsg() const;
	void setMsg(std::string message);

	std::string toString() const;
	
	/// Equality predicate
	bool operator==(const Msg& other) const
	{
		return (m_msgText == other.m_msgText);
	}
	
private:
	std::string m_msgText;
};

std::ostream& operator<<(std::ostream& os, const Msg& msg); ///< Text output for debugging

}	// namespace datatypes

#endif
