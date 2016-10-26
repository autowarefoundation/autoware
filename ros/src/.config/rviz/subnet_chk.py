#!/usr/bin/env python

import sys
import os
import socket
import netifaces
import struct

def get_targ(path):
	with open(path, 'r') as file:
		lst = [ l.strip() for l in reversed(file.readlines()) ]
	lst = [ l for l in lst if l and l[0] != '#' ]
	vs = []
	for l in lst:
		if ':' not in l:
			return l
		p = [ a.strip() for a in l.split(':') ]
		if p[0] == 'host':
			vs.append(p[1])
	return next( (v for v in vs), None)

def addr_v(addr):
	return struct.unpack('!i', socket.inet_aton(addr))[0]

if len(sys.argv) < 2:
	print "Usage: {} <ipaddr|hostname|->".format(sys.argv[0])
	sys.exit(0)

targ = sys.argv[1]
if targ == '-': # read 'host' file
	path = os.path.join(os.path.dirname(__file__), 'host')
	if not os.path.exists(path):
		sys.exit(1)
	targ = get_targ(path)
	if not targ or targ == 'localhost':
		sys.exit(1)
try:
	targ = socket.gethostbyname(targ)
except socket.gaierror:
	print >> sys.stderr, "%s: cannot convert to ip address" % (targ)
	sys.exit(2)
targ_v = addr_v(targ)

lst = [ ifnm for ifnm in netifaces.interfaces() if not ifnm.startswith('lo') ]
lst = [ netifaces.ifaddresses(ifnm).get(netifaces.AF_INET) for ifnm in lst ]
lst = [ ( a.get('addr'), a.get('netmask') ) for l in lst if l for a in l ]
lst = [ p for p in lst if None not in p ]

for (addr, mask_s) in lst:
	v = addr_v(addr)
	mask = addr_v(mask_s)
	if (v & mask) == (targ_v & mask):
		print addr
		sys.exit(0)
sys.exit(1)
