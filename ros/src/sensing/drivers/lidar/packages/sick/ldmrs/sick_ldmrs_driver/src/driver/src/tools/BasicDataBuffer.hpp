/**
 * \file BasicDataBuffer.hpp
 */

#ifndef BASICDATABUFFER_HPP
#define BASICDATABUFFER_HPP

#include "../BasicDatatypes.hpp"
#include "Mutex.hpp"
#include <vector>	// for std::vector
#include <list>		// for std::list

using namespace datatypes;

//
// Buffer for Data of base-type BasicData.
//
class BasicDataBuffer
{
public:
	/// Default constructor.
	BasicDataBuffer();

	/// Destructor.
	~BasicDataBuffer();
	
	void setLimit(UINT32 maxBytesToBeUsed);
	bool pushData(BasicData* data);
	BasicData* popData();
	UINT32 getUsedBytes();
	UINT32 getBufferSize();	// # gespeicherter Datensaetze
	

private:
	bool m_beVerbose;
	UINT32 m_bytesMax;
	std::list<BasicData*> m_buffer;
	UINT32 m_bytesUsed;
	Mutex m_mutex;	// Thread-Safety
};



#endif
