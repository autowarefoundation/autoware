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

#!/usr/bin/env python

import os
import sys
import signal
import socket
import yaml
import ctypes
import psutil
import select
import re
import pickle
import threading
import multiprocessing
import time # for *debug*
#import hashlib

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
		except AttributeError:
			proc.nice(value)
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
		except AttributeError:
			proc.cpu_affinity(cpus)
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

	def _set_sched_switch(self, t):
		f = open('/sys/kernel/debug/tracing/events/sched/sched_switch/enable', 'w')
		f.write('1' if t else '0')
		f.close()

	def _set_ftrace(self, t):
		f = open('/sys/kernel/debug/tracing/tracing_on', 'w')
		f.write('1' if t else '0')
		f.close()

	def _ftrace(self, sec, pids=[]):
		opid = [0] * multiprocessing.cpu_count()
		ret = {}
		for cpuno in range(0, multiprocessing.cpu_count()):
			ret[cpuno] = []
		wsec = sec
		f = open('/sys/kernel/debug/tracing/trace_pipe', 'r')
		time.sleep(wsec)
		while True:
			(r, _, _) = select.select([f], [], [], 0)
			if len(r) <= 0:
				break
			l = f.readline()
			m = re.match('^.* \[([0-9]*)\].* ([0-9]*\.[0-9]*): .*==> next_comm=.* next_pid=([0-9]*) next.*$', l)
			if m is None:
				continue
			cpuno = int(m.group(1))
			t = float(m.group(2))
			pid = int(m.group(3))
			if stime == 0:
				stime = t
			if pid not in pids and pid > 0:
				pid = 0 # idle...
			#if pid != opid[cpuno] or pid in pids or opid[cpuno] in pids:
			if pid != opid[cpuno]:
				dat = [pid, t]
				ret[cpuno].append(dat)
			opid[cpuno] = pid
		f.close()
		return ret

	def _filterNodePid(self, pids):
		f = open('/sys/kernel/debug/tracing/set_ftrace_pid','w')
		f.close()
		for pid in pids:
			f = open('/sys/kernel/debug/tracing/set_ftrace_pid','a')
			f.write(str(pid))
			f.close()

	def get_ftrace(self, sec, pids):
		st = time.time() # for *debug*
		self._ftrace(0)
		self._filterNodePid(pids)
		self._set_sched_switch(True)
		self._set_ftrace(True)
		ret = self._ftrace(1, pids)
		self._set_ftrace(False)
		self._set_sched_switch(False)
		self._ftrace(0)
		et = time.time() - st # for *debug*
		print "* ftrace", et, "sec" # for *debug*
		return ret

	def get_ftrace_cont(self, conn, interval, pids):
		self._filterNodePid(pids)
		self._set_sched_switch(True)
		self._set_ftrace(True)
		f = open('/sys/kernel/debug/tracing/trace_pipe', 'r')
		while True:
			opid = [0] * multiprocessing.cpu_count()
			ret = {}
			for cpuno in range(0, multiprocessing.cpu_count()):
				ret[cpuno] = []
			time.sleep(interval)
			while True:
				(r, _, _) = select.select([f], [], [], 0)
				if len(r) <= 0:
					break
				l = f.readline()
				m = re.match('^.* \[([0-9]*)\].* ([0-9]*\.[0-9]*): .*==> next_comm=.* next_pid=([0-9]*) next.*$', l)
				if m is None:
					continue
				cpuno = int(m.group(1))
				t = float(m.group(2))
				pid = int(m.group(3))
				if pid not in pids and pid > 0:
					pid = 0 # idle...
				#if pid != opid[cpuno] or pid in pids or opid[cpuno] in pids:
				if pid != opid[cpuno]:
					dat = [pid, t]
					ret[cpuno].append(dat)
				opid[cpuno] = pid
			dat = pickle.dumps(ret)
			slen = 0
			try:
				while slen < len(dat):
					slen += conn.send(dat[slen:])
			except socket.error:
				print "ftrace disconnected"
				break
		f.close()
		self._set_ftrace(False)
		self._set_sched_switch(False)
		conn.close()

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
			elif order['name'] == 'ftrace':
				ret = self.get_ftrace(order['sec'], order['pids'])
			elif order['name'] == 'ftrace_cont':
				th = threading.Thread(target=self.get_ftrace_cont,
						      name="ftrace_cont",
						      args=(conn, order['interval'], order['pids']))
				th.start()
				continue
			elif order['name'] == 'shutdown':
				conn.send(str.encode("0"))
				conn.close()
				print("[proc_manager.py] Shutdown process manager")
				break
			else:
				print("Error: unknown operation key: '{}'".format(order['name']))
				ret = -1
			if isinstance(ret, (int, long)):
				conn.send(str.encode(str(ret)))
			else:
				st = time.time() # for *debug*
				#dat = yaml.dump(ret) ## too slow!
				dat = pickle.dumps(ret)
				tt = time.time() - st # for *debug*
				print "** dump", tt, "sec"
				slen = 0
				try:
					while slen < len(dat):
						slen += conn.send(dat[slen:])
				except socket.error:
					print 'socket failed'
				tt = time.time() - st # for *debug*
				print "** sent", tt, "sec, size", len(dat)
				#print "** md5", hashlib.md5(dat).hexdigest()
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

def get_cpu_count():
	try:
		return psutil.NUM_CPUS
	except AttributeError:
		return psutil.cpu_count()
	

if __name__ == "__main__":
	if os.getuid() != 0:
		print("You must run runtime manger as root user")
		sys.exit(-1)

	drop_capabilities()

	manager = ProcManager()
	manager.run()

