/*
 * Copyright 2017-2019 Autoware Foundation. All rights reserved.
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

#ifndef YMC_CAN_H
#define YMC_CAN_H

#include <iostream>
#include <cstring>
#include <vector>

#define RET_NO_PUBLISH (double)-1.0

namespace ymc
{

void setCanData(unsigned char* data, unsigned char d1, unsigned char d2, uint16_t d3, int16_t d4, unsigned char d5, unsigned char d6);
double translateCanData(const int id, const std::vector<std::string>& data, int* mode);

}

#endif
