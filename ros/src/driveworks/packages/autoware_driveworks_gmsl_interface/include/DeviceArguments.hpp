/*
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
<<<<<<< HEAD
 * 
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 * 
=======
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
>>>>>>> 78274e28ff4ac39185e5dfcad5d5ce2ba8d12b66
 *  All rights reserved.
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
  Modified from Nvidia SDK - Camera gmsl
<<<<<<< HEAD
  Author: Punnu Phairatt 
=======
  Author: Punnu Phairatt
>>>>>>> 78274e28ff4ac39185e5dfcad5d5ce2ba8d12b66
  Initial Date: 10/05/18
*/

#ifndef _DEVICE_ARGUMENTS_
#define _DEVICE_ARGUMENTS_

#include <map>
#include <string>
#include <iterator>
#include <vector>
#include <iostream>

namespace DriveWorks
{

typedef std::pair<std::string, std::string> option_t;

<<<<<<< HEAD
class DeviceArguments 
=======
class DeviceArguments
>>>>>>> 78274e28ff4ac39185e5dfcad5d5ce2ba8d12b66
{
public:
  /* Constructor */
  DeviceArguments(const std::vector<option_t>& options);
  /* Destructor */
  ~DeviceArguments();
<<<<<<< HEAD
	/* Print arguments on screen */   
  void printArguments();	 
  /* Get arguments */ 
  const std::string& get(const char *name) const;
  /* Set argument */
  bool set(const char* name, std::string new_value); 
	  
	  
private:
  std::map<std::string, std::string> arguments;
  const std::string empty_string = {}; 	
=======
	/* Print arguments on screen */
  void printArguments();
  /* Get arguments */
  const std::string& get(const char *name) const;
  /* Set argument */
  bool set(const char* name, std::string new_value);


private:
  std::map<std::string, std::string> arguments;
  const std::string empty_string = {};
>>>>>>> 78274e28ff4ac39185e5dfcad5d5ce2ba8d12b66
};

};//namespace

#endif


