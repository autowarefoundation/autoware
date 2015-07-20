/**
 * @file
 *
 * @~english
 * @brief	Generic ring buffer class implementation.
 *
 * @author	Masaki SegaWa
 *
 * @~japanese
 * @brief	汎用リングバッファクラス実装.
 *
 * @author	瀬川正樹
 *
 * @~
 * @date	2009-04-07
 *
 * Copyright (c) 2009 ZMP Inc. All rights reserved.
 *
 */

#include <stdio.h>
#include <string.h>
#include "RingBuffer.h"

namespace zmp {
    namespace hev {


/*------------------------------------------------------------
 * Constructor and destructor
 *
 */
CRingBuffer::CRingBuffer(int buffer_size)
	: m_buff(NULL)
	, m_head(0)
	, m_tail(0)
{
	m_buffer_size = buffer_size;

	m_buff = new char[m_buffer_size];
	memset(m_buff, 0, m_buffer_size);

}

/*
 * m_bufをdelete
 */
CRingBuffer::~CRingBuffer(void)
{
	if (m_buff) {
		delete[] m_buff;
		m_buff = NULL;
	}
	m_head = m_tail = 0;
	m_buffer_size = 0;
}


/*------------------------------------------------------------
 * Public methods
 *
 */


/*
 *
 */
bool CRingBuffer::Put(const char *buf, int size)
{
	if (GetQueueLength() + size >= m_buffer_size-1) {
		return false;
	}
	const char *p = buf;
	for (int i = 0; i < size; i++) {
		m_buff[m_tail++] = *p++;
		m_tail %= m_buffer_size;
	}
	return true;
}

/*
 *
 */
bool CRingBuffer::Get(char *buf, int size)
{
	if (GetQueueLength() < size) {
		return false;
	}

	char *p = buf;
	for (int i = 0; i < size; i++) {
		*p++ = m_buff[m_head++];
		m_head %= m_buffer_size;
	}

	return true;
}

/*
 *
 */
int CRingBuffer::GetQueueLength() const
{
	int len = m_tail - m_head;
	if (len < 0) {
		len += m_buffer_size;
	}
	return len;
}

/*
 *
 */
int CRingBuffer::GetSpaceLength() const
{
	return m_buffer_size - GetQueueLength() - 1;
}

/*
 *
 */
int CRingBuffer::Empty()
{
	m_head = m_tail = 0;
	return 0;
}

/*
 *
 */
CTokenizedRingBuffer::CTokenizedRingBuffer(int buffer_size)
	: CRingBuffer(buffer_size)
{
}

/*
 *
 */
bool CTokenizedRingBuffer::GetToken(char *buf, char delim, int max_size, int *read_size)
{
	int len;
	bool b_find = GetTokenLength(delim, &len);

	len = (len < max_size) ? len : max_size;

	char *p = buf;

	int i;
	for (i = 0; i < len; i++) {
		*p++ = m_buff[m_head++];
		m_head %= m_buffer_size;
	}
	// 長さを返す
	*read_size = i;

	return b_find;
}

/*
 *
 */
bool CTokenizedRingBuffer::GetTokenLength(char delim, int *size) const
{
	int head = m_head;
	int tail = m_tail;
	int cnt = 0;
	bool b_find = false;
	while (head != tail) {
		char c = m_buff[head++];
		head %= m_buffer_size;
		cnt++; // デリミタも含む
		if (c == delim) {
			b_find = true;
			break;
		}
	}
	*size = cnt;
	return b_find;
}




	}
}
