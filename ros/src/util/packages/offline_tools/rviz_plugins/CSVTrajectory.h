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
 * CSVTrajectory.h
 *
 *  Created on: Oct 4, 2018
 *      Author: sujiwo
 */

#ifndef _CSVTRAJECTORY_H_
#define _CSVTRAJECTORY_H_

#include "FileProperty.h"
#include <rviz/display.h>
#include <rviz/ogre_helpers/axes.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/string_property.h>
#include <string>
#include <vector>

class CSVTrajectory : public rviz::Display {
  Q_OBJECT
public:
  CSVTrajectory();
  virtual ~CSVTrajectory();

protected:
  //	void onInitialize();

private Q_SLOTS:
  void changeFile();
  void updatePoseAxisGeometry();

protected:
  virtual void onInitialize();

  //	FileProperty* csvFilePath;
  rviz::StringProperty *csvFilePath;
  std::vector<rviz::Axes *> axesList;

  rviz::FloatProperty *axesLengthSize;
  rviz::FloatProperty *axesRadius;

private:
  void updateDisplay(const std::string &filename);
  //	void updateDisplay ()
};

#endif /* _CSVTRAJECTORY_H_ */
