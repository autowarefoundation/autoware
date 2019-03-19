/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
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
