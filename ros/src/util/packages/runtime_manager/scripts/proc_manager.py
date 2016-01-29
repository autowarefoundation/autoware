#!/usr/bin/env python

"""
  Copyright (c) 2015, Nagoya University
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this
    list of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

  * Neither the name of Autoware nor the names of its
    contributors may be used to endorse or promote products derived from
    this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

import os
import sys
import signal
import socket
import yaml
import ctypes
import psutil

libc = ctypes.CDLL("libc.so.6")

PR_CAPBSET_DROP=24
SOCK_PATH="/tmp/autoware_proc_manager"

class ProcManager:
	def __init__(self):
		self.sock = socket.socket(socket.AF_UNIX)
		try:
			os.unlink(SOCK_PATH)
		except:
			pass
		self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		self.sock.bind(SOCK_PATH)
		self.sock.listen(10)
		os.chmod(SOCK_PATH, 0777)

	def set_nice(self, pid, value):
		try:
			proc = psutil.Process(pid)
		except Exception as e:
			print("Error construct psutil.Process(pid={})".format(pid))
			return -1

		try:
			proc.set_nice(value)
		except Exception as e:
			print("Error set_nice: {}".format(e))
			return -1

		print("[set_nice] pid={}, value={} ".format(
			pid, value))
		return 0

	def set_cpu_affinity(self, pid, cpus):
		try:
			proc = psutil.Process(pid)
		except psutil.NoSuchProcess as e:
			return

		print("[CPU affinity] Set pid:{}, CPUS: {}: ".format(proc.pid, cpus))

		ret = 0
		try:
			proc.set_cpu_affinity(cpus)
		except Exception:
			ret = -1
		return ret

	def _policy_to_string(self, policy):
		if policy == 0:
			return "SCHED_OTHER"
		elif policy == 1:
			return "SCHED_FIFO"
		elif policy == 2:
			return "SCHED_RR"
		else:
			raise ValueError("Invalid schedule policy argument")

	def set_scheduling_policy(self, pid, policy, priority):
		print("[sched_setscheduler] pid={}, priority={} ".format(
			pid, self._policy_to_string(policy), priority))

		param = ctypes.c_int(priority)
		err = libc.sched_setscheduler(pid, ctypes.c_int(policy), ctypes.byref(param))
		return err

	def run(self):
		while True:
			conn, addr = self.sock.accept()
			data = conn.recv(4096)

			order = yaml.load(data)
			ret = 0

			if order['name'] == 'nice':
				ret = self.set_nice(order['pid'], order['nice'])
			elif order['name'] == 'cpu_affinity':
				ret = self.set_cpu_affinity(order['pid'], order['cpus'])
			elif order['name'] == 'scheduling_policy':
				ret = self.set_scheduling_policy(order['pid'],
								 order['policy'],
								 order['priority'])
			elif order['name'] == 'shutdown':
				conn.send(str.encode("0"))
				conn.close()
				print("[proc_manager.py] Shutdown process manager")
				break
			else:
				print("Error: unknown operation key: '{}'".format(order['name']))
				ret = -1
			conn.send(str.encode(str(ret)))
			conn.close()

def cap_last_cap():
	last_cap = 0
	with open("/proc/sys/kernel/cap_last_cap", "r") as f:
		last_cap = int(f.read())
	return last_cap

def drop_capabilities():
	KEEP_CAPS = [6, 7, 23] # CAP_SETUID, CAP_SETGID, CAP_SYS_NICE

	for cap in range(0, cap_last_cap()+1):
		if cap not in KEEP_CAPS:
			libc.prctl(PR_CAPBSET_DROP, cap)


if __name__ == "__main__":
	if os.getuid() != 0:
		print("You must run runtime manger as root user")
		sys.exit(-1)

	drop_capabilities()

	manager = ProcManager()
	manager.run()
