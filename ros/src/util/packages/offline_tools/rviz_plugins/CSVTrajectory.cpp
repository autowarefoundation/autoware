/*
 *  Copyright (c) 2019, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
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
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 *  THE POSSIBILITY OF SUCH DAMAGE.
 */


/*
 * CSVTrajectory.cpp
 *
 *  Created on: Oct 4, 2018
 *      Author: sujiwo
 */

#include "CSVTrajectory.h"
#include "csv.h"
#include <OgreMatrix4.h>
#include <cstdlib>
#include <nav_msgs/Path.h>

using namespace std;

CSVTrajectory::CSVTrajectory() {
  csvFilePath = new rviz::StringProperty(
      "CSVFilePath", QString(), "CSV File to load", this, SLOT(changeFile()));

  axesLengthSize =
      new rviz::FloatProperty("Length", 0.3, "Length of the axes.", this,
                              SLOT(updatePoseAxisGeometry()));
  axesRadius = new rviz::FloatProperty("Radius", 0.03, "Radius of the axes.",
                                       this, SLOT(updatePoseAxisGeometry()));
}

CSVTrajectory::~CSVTrajectory() {}

void CSVTrajectory::onInitialize() {}

void CSVTrajectory::changeFile() {
  string fn = csvFilePath->getString().toStdString();
  return updateDisplay(fn);
}

void CSVTrajectory::updatePoseAxisGeometry() {
  for (int i = 0; i < axesList.size(); i++) {
    axesList[i]->set(axesLengthSize->getFloat(), axesRadius->getFloat());
  }

  // last step
  queueRender();
}

void CSVTrajectory::updateDisplay(const std::string &filename) {
  try {
    StringTable csvTableInp = create_table(filename);
    if (csvTableInp.columns() != 8)
      throw runtime_error("Input file must contains 8 columns without header");
    axesList.resize(csvTableInp.rows());

    for (int r = 0; r < csvTableInp.rows(); ++r) {

      axesList[r] = new rviz::Axes(
          scene_manager_, scene_node_,
          axesLengthSize->getFloat(), // Axes length; must be set using property
          axesRadius->getFloat());    // Axes radius

      axesList[r]->setPosition(Ogre::Vector3(csvTableInp.getd(r, 1),
                                             csvTableInp.getd(r, 2),
                                             csvTableInp.getd(r, 3)));

      axesList[r]->setOrientation(
          Ogre::Quaternion(csvTableInp.getd(r, 7), csvTableInp.getd(r, 4),
                           csvTableInp.getd(r, 5), csvTableInp.getd(r, 6)));
    }
  } catch (std::runtime_error &e) {
    // Catch errors here
    // XXX: Put nice error message when unable to open input file
  }

  queueRender();
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(CSVTrajectory, rviz::Display)
