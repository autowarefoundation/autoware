/**
 * \file BasicDataBuffer.cpp
 */

#include "BasicDataBuffer.hpp"
#include "errorhandler.hpp"
#include "toolbox.hpp"


// ****************************************************************************
//     Buffer fuer BasicData
// ****************************************************************************
BasicDataBuffer::BasicDataBuffer()
	: m_beVerbose(false)
	, m_bytesMax(0)
	, m_bytesUsed(0)
{
	infoMessage("BasicDataBuffer: Starting constructor.", m_beVerbose);
	
	infoMessage("BasicDataBuffer(): Constructor is done.", m_beVerbose);
}

BasicDataBuffer::~BasicDataBuffer()
{
	infoMessage("~BasicDataBuffer(): Destructor called.", m_beVerbose);
	
	
	infoMessage("~BasicDataBuffer(): Destructor is done - object is dead.", m_beVerbose);
}

/**
 * Setze das Speicher-Limit, in Bytes. 0 = Kein Limit.
 * 
 * Falls bereits zu viel Speicher benutzt wird, wird dieser nicht sofort freigegeben,
 * aber keine weiteren Elemente akzeptiert.
 */
void BasicDataBuffer::setLimit(UINT32 maxBytesToBeUsed)
{
	m_bytesMax = maxBytesToBeUsed;
}


/**
 * Liefert die aktuell genutzte Speichergroesse.
 * 
 */
UINT32 BasicDataBuffer::getUsedBytes()
{
	return m_bytesUsed;
}

/**
 * Liefert die Anzahl der aktuell gespeicherten Datensaetze.
 * 
 */
UINT32 BasicDataBuffer::getBufferSize()
{
	return m_buffer.size();
}

//
// Speichert den Zeiger (!) auf das Datum im Puffer, sofern der Platz im Puffer (mitgerechnet
// wird der Speicherplatz des Daten-Objekts) noch reicht.
// 
// true: Datum wurde gespeichert.
//
bool BasicDataBuffer::pushData(BasicData* data)
{
	ScopedLock lock(&m_mutex);	// .lock();
	
	// Gibt es ein Limit?
	UINT32 sizeOfNewData = data->getUsedMemory();
	if (m_bytesMax > 0)
	{
		// Es gibt ein Limit, also pruefen
		UINT32 newSize = m_bytesUsed + sizeOfNewData;	// sizeof(*data);
		if (newSize > m_bytesMax)
		{
			// Das Limit wird ueberschritten, also dieses Datum ablehnen.
			m_mutex.unlock();
			return false;
		}
	}
	
	// Datum speichern
	m_buffer.push_back(data);
	m_bytesUsed += sizeOfNewData;
	
//	m_mutex.unlock();
	return true;
}

//
// Liefert den Zeiger auf das aelteste Datum im Puffer. War kein Datum vorhanden, ist der Zeiger NULL.
//
// Die Bytes-Used-Verwaltung funktioniert nicht, falls die Daten im Puffer von der Applikation veraendert werden.
// Dann werden naemlich eine andere Anzahl Bytes entfernt, als hinzugefuegt wurden...
//
BasicData* BasicDataBuffer::popData()
{
	ScopedLock lock(&m_mutex);
	
	BasicData* data = NULL;
	
	// Sind noch Daten im Puffer?
	if (m_buffer.size() > 0)
	{
		data = m_buffer.front();
		m_buffer.pop_front();
		m_bytesUsed -= data->getUsedMemory();
	}
	
//	m_mutex.unlock();
	return data;
}
