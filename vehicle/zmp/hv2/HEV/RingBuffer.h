/**
 * @file
 *
 * @~english
 * @brief	Generic ring buffer class header.
 *
 * @author	Masaki SegaWa
 *
 * @~japanese
 * @brief	汎用リングバッファクラスヘッダ.
 *
 * @author	瀬川正樹
 *
 * @~
 * @date	2009-04-07
 *
 * Copyright (c) 2009 ZMP Inc. All rights reserved.
 *
 */


#ifndef __RING_BUFFER_H__
#define __RING_BUFFER_H__

namespace zmp {
    namespace hev {



/**
 * @~english
 * Ring buffer. For general receive buffer.
 * @~japanese
 * リングバッファ.受信データのバッファ用。.
 */
class CRingBuffer
{
public:
	/**
	 * @~english
	 * Constructor.
	 * Initialize with specified buffer size. the internal buffer filled with 0.
	 * @param			buffer_size The buffer size.
	 * @~japanese
	 * Constructor.
	 * サイズを指定して初期化する。メモリは0クリアされる。
	 * @param 		buffer_size バッファサイズ。
	 **/
	CRingBuffer(int buffer_size);

	/**
	 * Destructor
	 */
	~CRingBuffer();

	/**
	 * @~english
	 * Put new data to the buffer.
	 * If there is enough space, the new data added the end of buffer. Otherwise it failed with no data added.
	 * @param[in] 	buf The new data.
	 * @param			size The length of the data.
	 * @retval		true Successful.
	 * @retval		false Failed. the buffer full.
	 * @~japanese
	 * バッファにデータを追加する.
	 * 指定したサイズ分空きがあれば追加なければ、1byteも追加せずに、失敗する。
	 * @param[in] 	buf 追加するデータ
	 * @param			size バッファサイズ
	 * @retval		true 成功
	 * @retval		false 失敗。バッファが容量オーバー
	 */
	bool Put(const char *buf, int size);
	/**
	 * @~english
	 * Get the data from buffer.
	 * If there is data specified size or more, return the data and delete from buffer.
	 * If no data or less than size, it failed and all data remains.
	 * @param[out] 	buf Buffer.
	 * @param			size The length of the data to get.
	 * @retval		true Successful.
	 * @retval		false Failed. Data in buffer less than the specified size.
	 * @~japanese
	 * データ指定したサイズ分データを取得。なければ、1byteも取得せずに、失敗する。
	 * @param[out]	buf	データストアするバッファ
	 * @param			size 取得するデータサイズ
	 * @retval		true	データがあれば取得できて、成功
	 * @retval		false	データがないか、指定したサイズより小さいので失敗
	 */
	bool Get(char *buf, int size);
	/**
	 * @~english
	 * Return the length of data in buffer.
	 * @return		The length in byte.
	 * @~japanese
	 * 現在のバッファ内のデータの長さを返す.
	 * @return		バッファの長さをbyteで。
	 */
	int GetQueueLength() const;
	/**
	 * @~english
	 * Return the length of the free spece.
	 * @return		The length in byte.
	 * @~japanese
	 * 現在のバッファ内空き領域の長さを返す.
	 * @return		バッファの長さをbyteで。
	 */
	int GetSpaceLength() const;
	/**
	 * @~english
	 * To empty.
	 * @return		always 0.
	 * @~japanese
	 * バッファを空にする.
	 * @return		常に0を返す。
	 */
	int Empty();
protected:
	/**
	 * バッファ
	 */
	char *m_buff;
	/**
	 * バッファサイズ
	 */
	int m_buffer_size;
	/**
	 * 読み出し位置
	 */
	int m_head;
	/**
	 * 書き込み位置
	 */
	int m_tail;
};

/**
 * @~english
 * Ringbuffer, supports deviding token.
 * @~japanese
 * リングバッファ トークン切り出し機能つき.
 * トークン切り出して読み込める文字列用。排他制御なし
 */
class CTokenizedRingBuffer : public CRingBuffer
{
public:
	/**
	 * @~english
	 * Constructor.
	 * Initialize with specified buffer size. the internal buffer filled with 0.
	 * @param			buffer_size The buffer size.
	 * @~japanese
	 * Constructor.
	 * サイズを指定して初期化する。メモリは0クリアされる。
	 * @param 		buffer_size バッファサイズ。
	 */
	CTokenizedRingBuffer(int buffer_size);
	/**
	 * @~english
	 * Get token from buffer. Get data specified size or specified delimiter.
	 * @param[out]	buf The buffer to store read data．
	 * @param			delim The delimiter. Specify a character.
	 * @param			max_size Maximun read size.
	 * @param[out]	read_size Size of data read, including a delimiter character.
	 * @retval		true Successful.
	 * @retval		false Failed.
	 * @~japanese
	 *  トークンの読み込み.
	 * バッファからトークンを切り出して取得する。取得するデータの最後にデリミタ1byteも含む。
	 * デリミタが見つからない場合、普通のGetをする。
	 * デリミタのあるなしにかかわらず、max_sizeで指定したサイズで切る。
	 * @param[out]	buf 読み込んだデータをストアするためのバッファ
	 * @param			delim 区切り文字、1文字指定する
	 * @param			max_size 最大読み取りサイズ
	 * @param[out]	read_size 読み込まれたデータのサイズ。デリミタ1文字含む。
	 */
	bool GetToken(char *buf, char delim, int max_size, int *read_size);
	/**
	 * @~english
	 * Retrun the size of token that find first.
	 * Returns the length of the token from the buffer. Including a delimiter at the end of data.
	 * If delimiter is not found, returns the length of the whole data.
	 * @return		Returns find token or not.
	 * @retval		true Find token.
	 * @retval		false No delimiter in the buffer.
	 * @~japanese
	 * バッファ内の最初のトークンの長さを返す.
	 * バッファからトークンを切り出して長さを返す。取得するデータの最後にデリミタ1byteも含む。
	 * デリミタが見つからない場合、全てのデータの長さを返す。
	 * デリミタのあるなしを戻り値とする。
	 * @param			delim	切り出すデリミタ
	 * @param[out]	size	デリミタまでの長さ、または全部の長さ
	 * @return		トークンが見つかったかどうかを返す。
	 * @retval		true	デリミタがあった。
	 * @retval		false	なかった
	 *  デリミタで指定したトークンの長さを返す
	 */
	bool GetTokenLength(char delim, int *size) const;
};



	}
}

#endif

