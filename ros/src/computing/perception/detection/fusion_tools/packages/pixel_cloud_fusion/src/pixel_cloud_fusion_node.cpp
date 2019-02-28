/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
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
 ********************
 *  v1.0: amc-nu (abrahammonrroy@yahoo.com)
 *
 * pixel_cloud_fusion_node.cpp
 *
 *  Created on: May, 19th, 2018
 */

#include "pixel_cloud_fusion/pixel_cloud_fusion.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, __APP_NAME__);

	ROSPixelCloudFusionApp app;

	app.Run();

	return 0;
}
