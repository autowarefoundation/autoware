/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  File:    MatlabIO.cpp
 *  Author:  Hilton Bristow
 *  Created: Jun 27, 2012
 */
#include <vector>
#include <cerrno>
#include <cstring>
#include <zlib.h>
#include <iostream>
#include <exception>
#include "MatlabIO.hpp"
using namespace std;
using namespace cv;

/*! @brief Open a filestream for reading or writing
 *
 * @param filename the full name and filepath of the file
 * @param mode either "r" for reading or "w" for writing
 * @return true if the file open succeeded, false otherwise
 */
bool MatlabIO::open(string filename, string mode) {

    // open the file
	filename_ = filename;
    if (mode.compare("r") == 0) fid_.open(filename.c_str(), fstream::in  | fstream::binary);
    if (mode.compare("w") == 0) fid_.open(filename.c_str(), fstream::out | fstream::binary);
    return !fid_.fail();
}

/*! @brief close the filestream and release all resources
 *
 * @return true if the filestream was successfully closed,
 * false otherwise. Even in the case of failure, the filestream
 * will no longer point to a valid object
 */
bool MatlabIO::close(void) {

    // close the file and release any associated objects
    fid_.close();
    return !fid_.fail();
}


/*! @brief product of the elements of a vector
 *
 * The function is useful for calculating the total number
 * of elements in an array given a vector of dims.
 * @param vec the input vector
 * @return the product of elements in the input
 */
template<typename T>
T product(const vector<T>& vec) {
	T acc = 1;
	for (unsigned int n = 0; n < vec.size(); ++n) acc *= vec[n];
	return acc;
}

/*! @brief transpose a multi-channel matrix
 *
 * The OpenCV builtin transpose method cannot tranpose multi-dimensional
 * matrices. This function provides that capability by splitting the matrix
 * into a vector of its channels, tranposing each channel, then merging
 * the result back into a single multi-channel matrix
 *
 * @param src the input matrix
 * @param dst the output matrix, where dst(i,j,k) == src(j,i,k)
 */
void transposeMat(const Mat& src, Mat& dst) {
	if (src.channels() > 1) {
		vector<Mat> vec;
		split(src, vec);
		for (unsigned int n = 0; n < vec.size(); ++n) {
			transpose(vec[n], vec[n]);
		}
		merge(vec, dst);
	} else {
		transpose(src, dst);
	}
}

/*! @brief convert the type of a variable
 *
 * Given a vector of type char, interpret the data as an
 * vector of type T1, and convert it to a vector of type
 * T2.
 * @param in the input char vector
 * @return the same data, reinterpreted as type T2 through
 * storage type T1
 */
template<class T1, class T2>
vector<T2> convertPrimitiveType(const vector<char>& in) {

	// firstly reinterpret the input as type T1
	const unsigned int T1_size = in.size() / sizeof(T1);
	const T1* in_ptr = reinterpret_cast<const T1*>(&(in[0]));

	// construct the new vector
	vector<T2> out(in_ptr, in_ptr+T1_size);
	return out;

}

/*! @brief get the .Mat file header information
 *
 * The fields read are:
 * header_ the matlab header as a human readable string,
 * subsys_ subsystem specific information,
 * version_ the .Mat file version (5 or 73),
 * endian_ the bye ordering of the .Mat file. If the byte ordering
 * needs reversal, this is automatically handled by esfstream.
 */
void MatlabIO::getHeader(void) {
    // get the header information from the Mat file
    for (unsigned int n = 0; n < HEADER_LENGTH+1; ++n) header_[n] = '\0';
    for (unsigned int n = 0; n < SUBSYS_LENGTH+1; ++n) subsys_[n] = '\0';
    for (unsigned int n = 0; n < ENDIAN_LENGTH+1; ++n) endian_[n] = '\0';
    fid_.read(header_, sizeof(char)*HEADER_LENGTH);
    fid_.read(subsys_, sizeof(char)*SUBSYS_LENGTH);
    fid_.read((char *)&version_, sizeof(int16_t));
    fid_.read(endian_, sizeof(char)*ENDIAN_LENGTH);

    // get the actual version
    if (version_ == 0x0100) version_ = VERSION_5;
    if (version_ == 0x0200) version_ = VERSION_73;

    // get the endianess
    if (strcmp(endian_, "IM") == 0) byte_swap_ = false;
    if (strcmp(endian_, "MI") == 0) byte_swap_ = true;
    // turn on byte swapping if necessary
    fid_.setByteSwap(byte_swap_);

    //printf("Header: %s\nSubsys: %s\nVersion: %d\nEndian: %s\nByte Swap: %d\n", header_, subsys_, version_, endian_, byte_swap_);
    bytes_read_ = 128;
}


/*! @brief interpret the variable header information
 *
 * Given a binary data blob, determine the data type and number of bytes
 * that constitute the data blob. This internally handles whether the
 * header is in long or short format.
 *
 * @param data_type the returned data type
 * @param dbytes the returned number of bytes that constitute the data blob
 * @param wbytes the whole number of bytes that include the header size,
 * the size of the data and any padding to 64-bit boundaries. This is equivalent
 * to the entire number of bytes effectively used by a variable
 * @param data the input binary blob
 * @return a pointer to the beginning of the data segment of the binary blob
 */
const char * MatlabIO::readVariableTag(uint32_t &data_type, uint32_t &dbytes, uint32_t &wbytes, const char *data) {
    
	bool small = false;
    const uint32_t *datai = reinterpret_cast<const uint32_t *>(data);
    data_type = datai[0];

    if ((data_type >> 16) != 0) {
        // small data format
        dbytes = data_type >> 16;
        data_type = (data_type << 16) >> 16;
        small = true;
    } else {
        // regular format
        dbytes = datai[1];
    }

    // get the whole number of bytes (wbytes) consumed by this variable, including header and padding
    if (small) wbytes = 8;
    else if (data_type == MAT_COMPRESSED) wbytes = 8 + dbytes;
    else wbytes = 8 + dbytes + ((8-dbytes) % 8);

    // return the seek head positioned over the data payload
    return data + (small ? 4 : 8);
}

/*! @brief construct a structure
 *
 * TODO: implement this
 * @param name
 * @param dims
 * @param real
 * @return
 */
MatlabIOContainer MatlabIO::constructStruct(vector<char>& name, vector<uint32_t>& dims, vector<char>& real) {

	vector<vector<MatlabIOContainer> > array;
	const char* real_ptr = &(real[0]);
	// get the length of each field
	uint32_t length_type;
	uint32_t length_dbytes;
	uint32_t length_wbytes;
	const char* length_ptr = readVariableTag(length_type, length_dbytes, length_wbytes, real_ptr);
	uint32_t length = reinterpret_cast<const uint32_t*>(length_ptr)[0];

	// get the total number of fields
	uint32_t nfields_type;
	uint32_t nfields_dbytes;
	uint32_t nfields_wbytes;
	const char* nfields_ptr = readVariableTag(nfields_type, nfields_dbytes, nfields_wbytes, real_ptr+length_wbytes);
	assert((nfields_dbytes % length) == 0);
	uint32_t nfields = nfields_dbytes / length;

	// populate a vector of field names
	vector<string> field_names;
	for (unsigned int n = 0; n < nfields; ++n) {
		field_names.push_back(string(nfields_ptr+(n*length)));
	}

	// iterate through each of the cells and construct the matrices
	const char* field_ptr = real_ptr+length_wbytes+nfields_wbytes;
	for (unsigned int m = 0; m < product<uint32_t>(dims); ++m) {
		vector<MatlabIOContainer> strct;
		for (unsigned int n = 0; n < nfields; ++n) {

			MatlabIOContainer field;
			uint32_t data_type;
			uint32_t dbytes;
			uint32_t wbytes;
			const char* data_ptr = readVariableTag(data_type, dbytes, wbytes, field_ptr);
			assert(data_type == MAT_MATRIX);
			field = collateMatrixFields(data_type, dbytes, vector<char>(data_ptr, data_ptr+dbytes));
			field.setName(field_names[n]);
			strct.push_back(field);
			field_ptr += wbytes;
		}
		array.push_back(strct);
	}
	return MatlabIOContainer(string(&(name[0])), array);
}

/*! @brief construct a cell array
 *
 * If the variable is of type MAT_CELL, construct a cell array. This is done by
 * iteratively calling collateMatrixFields() on each element of the cell, and
 * storing the result in a vector<MatlabIOContainer>.
 * Cell fields may not have a name, but are still required to have a name tag. In
 * this case, placeholder names are substituted. The dimensionality of the cell
 * array is ignored, and the size is linearized in column major format.
 *
 * @param name the variable name
 * @param dims the dimesionality of the cell array (ignored)
 * @param real the real part
 * @return the wrapped cell array
 */
MatlabIOContainer MatlabIO::constructCell(vector<char>& name, vector<uint32_t>& dims, vector<char>& real) {

	vector<MatlabIOContainer> cell;
	char* field_ptr = &(real[0]);
	for (unsigned int n = 0; n < product<uint32_t>(dims); ++n) {
		MatlabIOContainer field;
		uint32_t data_type;
		uint32_t dbytes;
		uint32_t wbytes;
		const char* data_ptr = readVariableTag(data_type, dbytes, wbytes, field_ptr);
		//printf("cell data_type: %d,  dbytes: %d\n", data_type, dbytes);
		assert(data_type == MAT_MATRIX);
		field = collateMatrixFields(data_type, dbytes, vector<char>(data_ptr, data_ptr+dbytes));
		cell.push_back(field);
		field_ptr += wbytes;
	}
	return MatlabIOContainer(string(&(name[0])), cell);
}

/*! @brief construct a sparse matrix
 *
 * TODO: implement this
 * @param name
 * @param dims
 * @param real
 * @param imag
 * @return
 */
MatlabIOContainer MatlabIO::constructSparse(vector<char>&, vector<uint32_t>&, vector<char>&, vector<char>&) {

	MatlabIOContainer variable;
	return variable;
}

/*! @brief construct a string from an extracted set of fields
 *
 * If the data is of type char, the data is stored as a string rather than a matrix.
 * The dimensionality is ignored (the data is linearized)
 *
 * @param name the variable name
 * @param dims the variable dimensionality (ignored)
 * @param real the string data
 * @return the wrapped string
 */
MatlabIOContainer MatlabIO::constructString(vector<char>& name, vector<uint32_t>&, vector<char>& real) {
	// make sure the data is null terminated
	real.push_back('\0');
	return MatlabIOContainer(string(&(name[0])), string(&(real[0])));
}


/*! @brief construct a matrix from an extracted set of fields
 *
 * Given the variable size, name, data and data type, construct a matrix.
 * Note that Matlab may store variables in a different data type to the
 * actual variable data type (T) to save space. For example matrix a = [1 2 3 4 5];
 * in Matlab will intrinsically be of type double (everything is unless otherwise
 * explicitly stated) but could be stored as a uint8_t to save space.
 * The type of the variable returned should necessarily be double, since
 * it's impossible to know at compile time which data types Matlab has decided
 * to store a set of variables in.
 *
 * @param name the variable name
 * @param dims the variable dimensionality (i, j, k, ...)
 * @param real the real part
 * @param imag the imaginary part (imag.size() == 0 if the data is real)
 * @param stor_type the storage type of the value
 * @return the wrapped matrix
 */
template<class T>
MatlabIOContainer MatlabIO::constructMatrix(vector<char>& name, vector<uint32_t>& dims, vector<char>& real, vector<char>& imag, uint32_t stor_type) {

	vector<T> vec_real;
	vector<T> vec_imag;
	vector<Mat> vec_mat;
	Mat flat;
	Mat mat;
	switch (stor_type) {
		case MAT_INT8:
			vec_real = convertPrimitiveType<int8_t, T>(real);
			vec_imag = convertPrimitiveType<int8_t, T>(imag);
			break;
		case MAT_UINT8:
			vec_real = convertPrimitiveType<uint8_t, T>(real);
			vec_imag = convertPrimitiveType<uint8_t, T>(imag);
			break;
		case MAT_INT16:
			vec_real = convertPrimitiveType<int16_t, T>(real);
			vec_imag = convertPrimitiveType<int16_t, T>(imag);
			break;
		case MAT_UINT16:
			vec_real = convertPrimitiveType<uint16_t, T>(real);
			vec_imag = convertPrimitiveType<uint16_t, T>(imag);
			break;
		case MAT_INT32:
			vec_real = convertPrimitiveType<int32_t, T>(real);
			vec_imag = convertPrimitiveType<int32_t, T>(imag);
			break;
		case MAT_UINT32:
			vec_real = convertPrimitiveType<uint32_t, T>(real);
			vec_imag = convertPrimitiveType<uint32_t, T>(imag);
			break;
		case MAT_INT64:
			vec_real = convertPrimitiveType<int64_t, T>(real);
			vec_imag = convertPrimitiveType<int64_t, T>(imag);
			break;
		case MAT_UINT64:
			vec_real = convertPrimitiveType<uint64_t, T>(real);
			vec_imag = convertPrimitiveType<uint64_t, T>(imag);
			break;
		case MAT_FLOAT:
			vec_real = convertPrimitiveType<float, T>(real);
			vec_imag = convertPrimitiveType<float, T>(imag);
			break;
		case MAT_DOUBLE:
			vec_real = convertPrimitiveType<double, T>(real);
			vec_imag = convertPrimitiveType<double, T>(imag);
			break;
		case MAT_UTF8:
			vec_real = convertPrimitiveType<char, T>(real);
			vec_imag = convertPrimitiveType<char, T>(imag);
			break;
		default:
			return MatlabIOContainer();
	}

	// assert that the conversion has not modified the number of elements
	uint32_t numel = 1;
	for (unsigned int n = 0; n < dims.size(); ++n) numel *= dims[n];
	assert(vec_real.size() == numel);

	// if the data is a scalar, don't write it to a matrix
	//if (vec_real.size() == 1 && vec_imag.size() == 0) return MatlabIOContainer(string(&(name[0])), vec_real[0]);

	// get the number of channels
	const unsigned int channels = dims.size() == 3 ? dims[2] : 1;
	bool complx = vec_imag.size() != 0;

	// put each plane of the image into a vector
	vector<Mat> sflat;
	flat = Mat(vec_real, true);
	for (unsigned int n = 0; n < channels; ++n)
		sflat.push_back(flat(Range(dims[0]*dims[1]*n, dims[0]*dims[1]*(n+1)), Range::all()));
	flat = Mat(vec_imag, true);
	for (unsigned int n = 0; n < channels*complx; ++n)
		sflat.push_back(flat(Range(dims[0]*dims[1]*n, dims[0]*dims[1]*(n+1)), Range::all()));

	// merge the planes into a matrix
	merge(sflat, flat);

	// reshape to the image dimensions
	mat = flat.reshape(flat.channels(), dims[1]);

	// transpose the matrix since matlab stores them in column major ordering
	transposeMat(mat, mat);

	return MatlabIOContainer(string(&(name[0])), mat);
}

/*! @brief interpret all fields of a matrix
 *
 * collateMatrixFields takes a binary blob of data and strips out the matrix fields.
 * These fields necessarily include: the variable dimensionality, the variable name
 * and the real part of the variable data. It optionally includes the imaginary part
 * of the variable data if that exists too. The extracted fields are used to either
 * construct a matrix, cell array or struct, or a scalar in the case where the variable
 * dimensionality is (1,1)
 *
 * @param data_type the type of the data stored in the binary blob
 * @param nbytes the number of bytes that constitute the binary blob
 * @param data the binary blob
 *
 * @return the variable (matrix, struct, cell, scalar) wrapped in a container
 */
MatlabIOContainer MatlabIO::collateMatrixFields(uint32_t, uint32_t, vector<char> data) {

    // get the flags
    bool complx  = data[9] & (1 << 3);
    //bool logical = data[9] & (1 << 1);
    
    // get the type of the encapsulated data
    char enc_data_type = data[8];
    // the preamble size is 16 bytes
    uint32_t pre_wbytes = 16;

    // get the dimensions
    uint32_t dim_type;
    uint32_t dim_dbytes;
    uint32_t dim_wbytes;
    const char* dim_data = readVariableTag(dim_type, dim_dbytes, dim_wbytes, &(data[pre_wbytes]));
    vector<uint32_t> dims(reinterpret_cast<const uint32_t *>(dim_data), reinterpret_cast<const uint32_t *>(dim_data+dim_dbytes));
    //printf("Complex?: %d\n", complx);
    //printf("Logical?: %d\n", logical);
    //printf("Dimensions: ");
    //for(int n = 0; n < dims.size(); ++n) printf("%d  ", dims[n]);
    //printf("\n");
    //printf("Dim bytes: %d\n", dim_dbytes);

    // get the variable name
    uint32_t name_type;
    uint32_t name_dbytes;
    uint32_t name_wbytes;
    const char* name_data = readVariableTag(name_type, name_dbytes, name_wbytes, &(data[pre_wbytes+dim_wbytes]));
    vector<char> name(name_data, name_data+name_dbytes);
    name.push_back('\0');
    //printf("The variable name is: %s\n", &(name[0]));

    // if the encoded data type is a cell array, bail out now
    if (enc_data_type == MAT_CELL_CLASS) {
    	vector<char> real(data.begin() + pre_wbytes+dim_wbytes+name_wbytes, data.end());
    	return constructCell(name, dims, real);
    } else if (enc_data_type == MAT_STRUCT_CLASS) {
    	vector<char> real(data.begin() + pre_wbytes+dim_wbytes+name_wbytes, data.end());
    	return constructStruct(name, dims, real);
    }

    // get the real data
    uint32_t real_type;
    uint32_t real_dbytes;
    uint32_t real_wbytes;
    const char* real_data = readVariableTag(real_type, real_dbytes, real_wbytes, &(data[pre_wbytes+dim_wbytes+name_wbytes]));
    vector<char> real(real_data,real_data+real_dbytes);
    //printf("The variable type is: %d\n", enc_data_type);
    //printf("Total number of bytes in data segment: %d\n", real_dbytes);

    vector<char> imag;
    if (complx) {
    	// get the imaginery data
    	uint32_t imag_type;
    	uint32_t imag_dbytes;
    	uint32_t imag_wbytes;
    	const char* imag_data = readVariableTag(imag_type, imag_dbytes, imag_wbytes, &(data[pre_wbytes+dim_wbytes+name_wbytes+real_wbytes]));
    	assert(imag_type == real_type);
    	for ( ; imag_data != imag_data+imag_dbytes; imag_data++) imag.push_back(*imag_data);
    }

    // construct whatever object we happened to get
    MatlabIOContainer variable;
    switch (enc_data_type) {
    	// integral types
    	case MAT_INT8_CLASS:      variable = constructMatrix<int8_t>(name, dims, real, imag, real_type); break;
        case MAT_UINT8_CLASS:     variable = constructMatrix<uint8_t>(name, dims, real, imag, real_type); break;
        case MAT_INT16_CLASS:     variable = constructMatrix<int16_t>(name, dims, real, imag, real_type); break;
        case MAT_UINT16_CLASS:    variable = constructMatrix<uint16_t>(name, dims, real, imag, real_type); break;
        case MAT_INT32_CLASS:     variable = constructMatrix<int32_t>(name, dims, real, imag, real_type); break;
        case MAT_UINT32_CLASS:    variable = constructMatrix<uint32_t>(name, dims, real, imag, real_type); break;
        case MAT_FLOAT_CLASS:     variable = constructMatrix<float>(name, dims, real, imag, real_type); break;
        case MAT_DOUBLE_CLASS:    variable = constructMatrix<double>(name, dims, real, imag, real_type); break;
        case MAT_INT64_CLASS:     variable = constructMatrix<int64_t>(name, dims, real, imag, real_type); break;
        case MAT_UINT64_CLASS:    variable = constructMatrix<uint64_t>(name, dims, real, imag, real_type); break;
        case MAT_CHAR_CLASS:      variable = constructString(name, dims, real); break;
        // sparse types
        case MAT_SPARSE_CLASS:    variable = constructSparse(name, dims, real, imag); break;
        // non-handled types
        case MAT_OBJECT_CLASS:	  break;
        default: 				  break;
    }
    return variable;
}

/*! @brief uncompress a variable
 *
 * If the data type of a variable is MAT_COMPRESSED, then the binary data blob
 * has been compressed using zlib compression. This function uncompresses the blob,
 * then calls readVariable() to interpret the actual data
 *
 * @param data_type the type of the data stored in the binary blob
 * @param dbytes the number of bytes that constitue the binary blob
 * @param wbytes the whole number of bytes that consistute the header,
 * the binary blob, and any padding to 64-bit boundaries
 * @param data the binary blob
 * @return the binary blob, uncompressed
 */
vector<char> MatlabIO::uncompressVariable(uint32_t& data_type, uint32_t& dbytes, uint32_t& wbytes, const vector<char> &data) {
    // setup the inflation parameters
    char buf[8];
    z_stream infstream;
    infstream.zalloc = Z_NULL;
    infstream.zfree  = Z_NULL;
    infstream.opaque = Z_NULL;
    int ok = inflateInit(&infstream);
    if (ok != Z_OK) { cerr << "Unable to inflate variable" << endl; exit(-5); }

    // inflate the variable header
    infstream.avail_in = data.size();
    infstream.next_in = (unsigned char *)&(data[0]);
    infstream.avail_out = 8;
    infstream.next_out = (unsigned char *)&buf;
    ok = inflate(&infstream, Z_NO_FLUSH);
    if (ok != Z_OK) { cerr << "Unable to inflate variable" << endl; exit(-5); }

    // get the headers
    readVariableTag(data_type, dbytes, wbytes, buf);

    // inflate the remainder of the variable, now that we know its size
    char *udata_tmp = new char[dbytes];
    infstream.avail_out = dbytes;
    infstream.next_out = (unsigned char *)udata_tmp;
    inflate(&infstream, Z_FINISH);
    inflateEnd(&infstream);

    // convert to a vector
    vector<char> udata(udata_tmp, udata_tmp+dbytes);
    delete [] udata_tmp;
    return udata;

}

/*! @brief Interpret a variable from a binary block of data
 *
 * This function may be called recursively when either uncompressing data or interpreting
 * fields of a struct or cell array
 *
 * @param data_type the type of the data stored in the binary blob
 * @param nbytes the number of bytes that constitute the binary blob
 * @param data the binary blob
 * @return an interpreted variable
 */
MatlabIOContainer MatlabIO::readVariable(uint32_t data_type, uint32_t nbytes, const vector<char> &data) {

    // interpret the data
    MatlabIOContainer variable;
    switch (data_type) {
    /*
        case MAT_INT8:      variable = primitiveFromBin<int8_t>(data, nbytes); break; 
        case MAT_UINT8:     variable = primitiveFromBin<uint8_t>(data, nbytes); break;
        case MAT_INT16:     variable = primitiveFromBin<int16_t>(data, nbytes); break;
        case MAT_UINT16:    variable = primitiveFromBin<uint16_t>(data, nbytes); break;
        case MAT_INT32:     variable = primitiveFromBin<int32_t>(data, nbytes); break;
        case MAT_UINT32:    variable = primitiveFromBin<uint32_t>(data, nbytes); break;
        case MAT_FLOAT:     variable = primitiveFromBin<float>(data, nbytes); break;
        case MAT_DOUBLE:    variable = primitiveFromBin<double>(data, nbytes); break;
        case MAT_INT64:     variable = primitiveFromBin<int64_t>(data, nbytes); break;
        case MAT_UINT64:    variable = primitiveFromBin<uint64_t>(data, nbytes); break;
        case MAT_UTF8:      variable = primitiveFromBin<char>(data, nbytes); break;
        case MAT_UTF16:     break;
        case MAT_UTF32:     break;
    */
        case MAT_COMPRESSED:
        {
            // uncompress the data
            uint32_t udata_type;
            uint32_t udbytes;
            uint32_t uwbytes;
            vector<char> udata = uncompressVariable(udata_type, udbytes, uwbytes, data);
            variable = readVariable(udata_type, udbytes, udata);
            break;
        }
        case MAT_MATRIX:
        {
            // deserialize the matrix
            variable = collateMatrixFields(data_type, nbytes, data);
            break;
        }
        default: break;
    }
    return variable;
}

/*! @brief read a block of data from the file being parsed
 *
 * This function attempts to read an entire variable from the file being parsed.
 * The data block is then encapsulated in a vector and passed onto readVariable()
 * for interpretation. This design means that the file is touched a minimal number
 * of times, and later manipulation of the data can make use of automatic memory
 * management, reference counting, etc.
 *
 * @return the block of data interpreted as a variable and stored in a generic container
 */
MatlabIOContainer MatlabIO::readBlock(void) {

    // allocate the output
    MatlabIOContainer variable;

    // get the data type and number of bytes consumed
    // by this variable. Check to see if it's using
    // the small data format (seriously, who thought of that? You save at best 8 bytes...)
    uint32_t data_type;
    uint32_t dbytes;
    uint32_t wbytes;
    char buf[8];
    fid_.read(buf, sizeof(char)*8);
    readVariableTag(data_type, dbytes, wbytes, buf);

    // read the binary data block
    //printf("\nReading binary data block...\n"); fflush(stdout);
    char *data_tmp = new char[dbytes];
    fid_.read(data_tmp, sizeof(char)*dbytes);
    vector<char> data(data_tmp, data_tmp+dbytes);
    delete [] data_tmp;

    // move the seek head position to the next 64-bit boundary
    // (but only if the data is uncompressed. Saving yet another 8 tiny bytes...)
    if (data_type != MAT_COMPRESSED) {
        //printf("Aligning seek head to next 64-bit boundary...\n");
        streampos head_pos = fid_.tellg();
        int padding = head_pos % 8;
        fid_.seekg(padding, fstream::cur);
    }

    // now read the variable contained in the block
    return readVariable(data_type, dbytes, data);
}


/*! @brief Read all variables from a file
 *
 * Reads every variable encountered when parsing a valid Matlab .Mat file.
 * If any of the variables is a function pointer, or other Matlab specific
 * object, it will be passed. Most integral types will be parsed successfully.
 * Matlab matrices will be converted to OpenCV matrices of the same type.
 * Note: Matlab stores images in RGB format whereas OpenCV stores images in
 * BGR format, so if displaying a parsed image using cv::imshow(), the
 * colours will be inverted.
 * @return a vector of containers storing the name and data of each variable
 * in the file
 */
std::vector<MatlabIOContainer> MatlabIO::read(void) {

    // allocate the output
    std::vector<MatlabIOContainer> variables;

    // read the header information
    getHeader();

    // get all of the variables
    while(hasVariable()) {

        MatlabIOContainer variable;
        variable = readBlock();
        variables.push_back(variable);
    }
    return variables;
}

/*! @brief Print a formatted list of the contents of a file
 *
 * Similar to the 'whos' function in matlab, this function prints to stdout
 * a list of variables and their C++ datatypes stored in the associated .Mat file
 * @param variables the variables read from the .Mat file using the read() function
 */
void MatlabIO::whos(vector<MatlabIOContainer> variables) const {

	// get the longest filename
	unsigned int flmax = 0;
	for (unsigned int n = 0; n < variables.size(); ++n) if(variables[n].name().length() > flmax) flmax = variables[n].name().length();

	printf("-------------------------\n");
	printf("File: %s\n", filename_.c_str());
	printf("%s\n", header_);
	printf("Variables:\n");
	for (unsigned int n = 0; n < variables.size(); ++n) {
		printf("%*s:  %s\n", flmax, variables[n].name().c_str(), variables[n].type().c_str());
	}
	printf("-------------------------\n");
	fflush(stdout);
}
