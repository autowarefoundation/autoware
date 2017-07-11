/* 
 *  Software License Agreement (BSD License)
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
 *  File:    typetraits.hpp
 *  Author:  Hilton Bristow
 *  Created: Jun 27, 2012
 */
#ifndef TYPETRAITS_HPP_
#define TYPETRAITS_HPP_

#include <string>
#include <vector>
#include <typeinfo>
#include <opencv2/core/core.hpp>
class MatlabIOContainer;


enum {
    MAT_INT8       = 1,
    MAT_UINT8      = 2,
    MAT_INT16      = 3,
    MAT_UINT16     = 4,
    MAT_INT32      = 5,
    MAT_UINT32     = 6,
    MAT_FLOAT      = 7,
    MAT_DOUBLE     = 9,
    MAT_INT64      = 12,
    MAT_UINT64     = 13,
    MAT_MATRIX     = 14,
    MAT_COMPRESSED = 15,
    MAT_UTF8       = 16,
    MAT_UTF16      = 17,
    MAT_UTF32      = 18
};

enum {
	MAT_CELL_CLASS 	   = 1,
	MAT_STRUCT_CLASS   = 2,
	MAT_OBJECT_CLASS   = 3,
	MAT_CHAR_CLASS     = 4,
	MAT_SPARSE_CLASS   = 5,
	MAT_DOUBLE_CLASS   = 6,
	MAT_FLOAT_CLASS    = 7,
	MAT_INT8_CLASS     = 8,
	MAT_UINT8_CLASS    = 9,
	MAT_INT16_CLASS    = 10,
	MAT_UINT16_CLASS   = 11,
	MAT_INT32_CLASS    = 12,
	MAT_UINT32_CLASS   = 13,
	MAT_INT64_CLASS    = 14,
	MAT_UINT64_CLASS   = 15
};

// default implementation
template <typename T>
struct TypeName {
	static const std::string toString() { return typeid(T).name(); }
};

// specialisations
template <>
struct TypeName<int8_t> {
	static const std::string toString() { return "int8_t"; }
};

template <>
struct TypeName<uint8_t> {
	static const std::string toString() { return "uint8_t"; }
};

template <>
struct TypeName<int16_t> {
	static const std::string toString() { return "int16_t"; }
};

template <>
struct TypeName<uint16_t> {
	static const std::string toString() { return "uint16_t"; }
};

template <>
struct TypeName<int32_t> {
	static const std::string toString() { return "int32_t"; }
};

template <>
struct TypeName<uint32_t> {
	static const std::string toString() { return "uint32_t"; }
};

template <>
struct TypeName<int64_t> {
	static const std::string toString() { return "int64_t"; }
};

template <>
struct TypeName<uint64_t> {
	static const std::string toString() { return "uint64_t"; }
};

template <>
struct TypeName<float> {
	static const std::string toString() { return "float"; }
};

template <>
struct TypeName<double> {
	static const std::string toString() { return "double"; }
};

template <>
struct TypeName<char> {
	static const std::string toString() { return "string"; }
};

template <>
struct TypeName<bool> {
	static const std::string toString() { return "logical"; }
};

template <>
struct TypeName<cv::Mat> {
	static const std::string toString() { return "Mat"; }
};

template <>
struct TypeName<MatlabIOContainer> {
	static const std::string toString() { return "MatlabIOContainer"; }
};

template <>
struct TypeName<std::vector<MatlabIOContainer> > {
	static const std::string toString() { return "vector<MatlabIOContainer>"; }
};

template <>
struct TypeName<std::vector<std::vector<MatlabIOContainer> > > {
	static const std::string toString() { return "vector<vector<MatlabIOContainer>>"; }
};

template <>
struct TypeName<std::vector<cv::Mat> > {
	static const std::string toString() { return "vector<Mat>"; }
};

template <>
struct TypeName<void> {
	static const std::string toString() { return "No stored value"; }
};


#endif /* TYPETRAITS_HPP_ */
