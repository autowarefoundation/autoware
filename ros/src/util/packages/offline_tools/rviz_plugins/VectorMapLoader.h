/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


/*
 * VectorMapLoader.h
 *
 *  Created on: Nov 14, 2018
 *      Author: sujiwo
 */

#ifndef _VECTORMAPLOADER_H_
#define _VECTORMAPLOADER_H_


#include <string>
#include <boost/filesystem.hpp>
#include <visualization_msgs/MarkerArray.h>

#include <vector_map/vector_map.h>


class VectorMapLoader : public vector_map::VectorMap
{
public:
	typedef decltype(vector_map::Point::bx) ptScalar;
	friend class VectorMapDisplay;

	VectorMapLoader(const std::string &directory);

//	inline visualization_msgs::MarkerArray::ConstPtr ConstPtr()
//	{ return visualization_msgs::MarkerArray::ConstPtr(&marker_array); }

	visualization_msgs::MarkerArray::ConstPtr getAsMessages();

	void setPointOffset (const ptScalar x_offset, const ptScalar y_offset, const ptScalar z_offset);

private:

	boost::filesystem::path vmDir;

	decltype(vector_map::VectorMap::point_.map_) pointOrig;

//	visualization_msgs::MarkerArray marker_array;

	void loadAll ();
};


#endif /* _VECTORMAPLOADER_H_ */
