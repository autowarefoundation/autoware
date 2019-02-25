#!/usr/bin/env python
#
# Copyright 2015-2019 Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import sys
import SocketServer
import struct

class Hdr(SocketServer.BaseRequestHandler):
	def handle(self):
		while True:
			data = self.request.recv(1024)
			if len(data) == 0:
				break;
			tp = struct.unpack('=5i', data)
			print(tp)
			(steer, accel, brake, gear, prog_manu) = tp

			#gear = 2 if gear is 1 else gear # for debug, R -> N , { 'b':0 , 'r':1 , 'n':2 , 'd':3 }
			#steer = 90 if steer > 90 else steer # for debug, limit 90
			data = struct.pack('=5i', steer, accel, brake, gear, prog_manu)
			self.request.send(data)
		self.request.close()

if __name__ == "__main__":
	port = sys.argv[1] if len(sys.argv) > 1 else 12345
	srv = SocketServer.TCPServer(('', port), Hdr)
	srv.serve_forever()

# EOF
