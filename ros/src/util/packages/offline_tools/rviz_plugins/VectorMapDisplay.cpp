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
 * VectorMapDisplay.cpp
 *
 *  Created on: Nov 8, 2018
 *      Author: sujiwo
 */

#include "VectorMapDisplay.h"
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/int_property.h>


using namespace std;

const VectorMapLoader::ptScalar defaultPointOffset[3] =
{18500, 93800, -33};


VectorMapDisplay::VectorMapDisplay():
	rviz::MarkerArrayDisplay()
{
	vMapDir_ = new rviz::StringProperty(
		"Vector Map Directory",
		QString(),
		"Directory of Autoware Vector Map Files",
		this,
		SLOT( changeDir() ));

	marker_topic_property_->hide();
	queue_size_property_->hide();
}

VectorMapDisplay::~VectorMapDisplay()
{
}


void
VectorMapDisplay::onInitialize()
{

}


void
VectorMapDisplay::changeDir()
{
	const string vectorMapDirName = vMapDir_->getStdString();
	mapData = shared_ptr<VectorMapLoader> (new VectorMapLoader(vectorMapDirName));
	mapData->setPointOffset(defaultPointOffset[0], defaultPointOffset[1], defaultPointOffset[2]);
	auto visMsgs = mapData->getAsMessages();

	markers_.clear();

	for (auto &markerM: visMsgs->markers) {
		rviz::MarkerBasePtr marker(rviz::createMarker(markerM.type, this, context_, scene_node_));
		if (marker) {
			marker->setMessage(markerM);
		}
		markers_.insert(marker);
	}

	context_->queueRender();
}


void
VectorMapDisplay::onEnable()
{
}


void
VectorMapDisplay::onDisable()
{

}


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(VectorMapDisplay, rviz::Display)
