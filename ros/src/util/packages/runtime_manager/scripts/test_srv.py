#!/usr/bin/env python

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
			print tp
			(steer, accel, brake, gear, prog_manu) = tp
		self.request.close()		

if __name__ == "__main__":
	port = sys.argv[1] if len(sys.argv) > 1 else 12345
	srv = SocketServer.TCPServer(('', port), Hdr)
	srv.serve_forever()

# EOF
