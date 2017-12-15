#!/usr/bin/env python
import serial
import struct

import sys, getopt, time, glob, math, pdb, numpy

from mtdef import MID, MTException, Baudrates, XDIGroup, getName, getMIDName, XDIMessage, XDIProductMask

# Verbose flag for debugging
verbose = False

################################################################
# MTDevice class
################################################################
## Xsens MT device communication object.
class MTDevice(object):
	"""Xsens MT device communication object."""

	def __init__(self, port, baudrate=115200, timeout=0.1, autoconf=True,
			config_mode=False):
		"""Open device."""
#		self.device = serial.Serial(port, baudrate, timeout=timeout,
#				writeTimeout=timeout)
                self.device = serial.Serial(port, baudrate, timeout=timeout,
                                           writeTimeout=timeout, rtscts=True, dsrdtr=True)
		self.device.flushInput()	# flush to make sure the port is ready TODO
		self.device.flushOutput()	# flush to make sure the port is ready TODO
		## timeout for communication
		self.timeout = timeout
		if autoconf:
			self.auto_config()
		else:
			## mode parameter of the IMU
			self.mode = None
			## settings parameter of the IMU
			self.settings = None
			## length of the MTData message
			self.length = None
			## header of the MTData message
			self.header = None
		if config_mode:
			self.GoToConfig()

	############################################################
	# Low-level communication
	############################################################

	## Low-level message sending function.
	def write_msg(self, mid, data=[]):
		"""Low-level message sending function."""
		length = len(data)
		if length>254:
			lendat = [0xFF, 0xFF&length, 0xFF&(length>>8)]
		else:
			lendat = [length]
		packet = [0xFA, 0xFF, mid] + lendat + list(data)
		packet.append(0xFF&(-(sum(packet[1:]))))
		msg = struct.pack('%dB'%len(packet), *packet)
		start = time.time()
		while (time.time()-start)<self.timeout and self.device.read():
			#print ".",
			pass
		self.device.write(msg)
		if verbose:
			print "MT: Write message id 0x%02X (%s) with %d data bytes: [%s]"%(mid, getMIDName(mid), length,
							' '.join("%02X"% v for v in data))
	## Low-level message sending function for MK4
	def write_msg_mk4(self, mid, data=[]):
		"""Low-level message sending function."""
		length = len(data)
		if length>254:
			lendat = [0xFF, 0xFF&length, 0xFF&(length>>8)]
		else:
			lendat = [length]
		packet = [0xFA, 0xFF, mid] + lendat + list(data)
		packet.append(0xFF&(-(sum(packet[1:]))))
		print packet
		msg = struct.pack('%dB'%len(packet), *packet)
		start = time.time()
		while (time.time()-start)<self.timeout and self.device.read():
			print "configured!.",
			pass
		self.device.write(msg)
		if verbose:
			print "MT: Write message id 0x%02X (%s) with %d data bytes: [%s]"%(mid, getMIDName(mid), length,
							' '.join("%02X"% v for v in data))
		
	## Low-level MTData receiving function.
	# Take advantage of known message length.
	def read_data_msg(self, buf=bytearray()):
		"""Low-level MTData receiving function.
		Take advantage of known message length."""
		start = time.time()
		if self.length>254:
			totlength = 7 + self.length
		else:
			totlength = 5 + self.length
		while (time.time()-start)<self.timeout:
			while len(buf)<totlength:
				buf.extend(self.device.read(totlength-len(buf)))
			preamble_ind = buf.find(self.header)
			if preamble_ind==-1:	# not found
				# discard unexploitable data
				#sys.stderr.write("MT: discarding (no preamble).\n")
				del buf[:-3]
				continue
			elif preamble_ind:	# found but not at start
				# discard leading bytes
				#sys.stderr.write("MT: discarding (before preamble).\n")
				del buf[:preamble_ind]
				# complete message for checksum
				while len(buf)<totlength:
					buf.extend(self.device.read(totlength-len(buf)))
			if 0xFF&sum(buf[1:]):
				#sys.stderr.write("MT: invalid checksum; discarding data and "\
				#		"waiting for next message.\n")
				del buf[:buf.find(self.header)-2]
				continue
			data = str(buf[-self.length-1:-1])
			del buf[:]
			return data
		else:
			raise MTException("could not find MTData message.")

	## Low-level message receiving function.
	def read_msg(self):
		"""Low-level message receiving function."""
		start = time.time()
		while (time.time()-start)<self.timeout:
			new_start = time.time()

			# Makes sure the buffer has 'size' bytes.
			def waitfor(size=1):
				while self.device.inWaiting() < size:
					if time.time()-new_start >= self.timeout:
						raise MTException("timeout waiting for message.")

			c = self.device.read()
			while (not c) and ((time.time()-new_start)<self.timeout):
				c = self.device.read()
			if not c:
				raise MTException("timeout waiting for message.")
			if ord(c)<>0xFA:
				continue
			# second part of preamble
			waitfor(3)
			if ord(self.device.read())<>0xFF:	# we assume no timeout anymore
				continue
			# read message id and length of message
			#msg = self.device.read(2)
			mid, length = struct.unpack('!BB', self.device.read(2))
			if length==255:	# extended length
				waitfor(2)
				length, = struct.unpack('!H', self.device.read(2))
			# read contents and checksum

			waitfor(length+1)
			buf = self.device.read(length+1)
			while (len(buf)<length+1) and ((time.time()-start)<self.timeout):
				buf+= self.device.read(length+1-len(buf))
			if (len(buf)<length+1):
				continue
			checksum = ord(buf[-1])
			data = struct.unpack('!%dB'%length, buf[:-1])
			if mid == MID.Error:
				sys.stderr.write("MT error 0x%02X: %s."%(data[0],
						MID.ErrorCodes[data[0]]))
			if verbose:
				print "MT: Got message id 0x%02X (%s) with %d data bytes: [%s]"%(mid, getMIDName(mid), length,
								' '.join("%02X"% v for v in data))
			if 0xFF&sum(data, 0xFF+mid+length+checksum):
				sys.stderr.write("invalid checksum; discarding data and "\
						"waiting for next message.\n")
				continue			
			return (mid, buf[:-1])
		else:
			raise MTException("could not find message.")

	## Send a message and read confirmation
	def write_ack(self, mid, data=[]):
		"""Send a message a read confirmation."""
		self.write_msg(mid, data)
		for tries in range(100):
			mid_ack, data_ack = self.read_msg()
			if mid_ack==(mid+1):
				break
		else:
			raise MTException("Ack (0x%X) expected, MID 0x%X received instead"\
					" (after 100 tries)."%(mid+1, mid_ack))
		return data_ack



	############################################################
	# High-level functions
	############################################################
	## Reset MT device.
	def Reset(self):
		"""Reset MT device."""
		self.write_ack(MID.Reset)
	
	## Place MT device in configuration mode.
	def ReqDeviceId(self):
		"""Request Device ID."""
		self.write_ack(MID.ReqDID)


	## Place MT device in configuration mode.
	def GoToConfig(self):
		"""Place MT device in configuration mode."""
		self.write_ack(MID.GoToConfig)


	## Place MT device in measurement mode.
	def GoToMeasurement(self):
		"""Place MT device in measurement mode."""
		self.write_ack(MID.GoToMeasurement)


	## Restore MT device configuration to factory defaults (soft version).
	def RestoreFactoryDefaults(self):
		"""Restore MT device configuration to factory defaults (soft version).
		"""
		self.GoToConfig()
		self.write_ack(MID.RestoreFactoryDef)


	## Get current output mode.
	# Assume the device is in Config state.
	def GetOutputMode(self):
		"""Get current output mode.
		Assume the device is in Config state."""
		data = self.write_ack(MID.SetOutputMode)
		self.mode, = struct.unpack('!H', data)
		return self.mode


	## Select which information to output.
	# Assume the device is in Config state.
	def SetOutputMode(self, mode):
		"""Select which information to output.
		Assume the device is in Config state."""
		H, L = (mode&0xFF00)>>8, mode&0x00FF
		self.write_ack(MID.SetOutputMode, (H, L))
		self.mode = mode


	## Get current output mode.
	# Assume the device is in Config state.
	def GetOutputSettings(self):
		"""Get current output mode.
		Assume the device is in Config state."""
		data = self.write_ack(MID.SetOutputSettings)
		self.settings, = struct.unpack('!I', data)
		return self.settings


	## Select how to output the information.
	# Assume the device is in Config state.
	def SetOutputSettings(self, settings):
		"""Select how to output the information.
		Assume the device is in Config state."""
		HH, HL = (settings&0xFF000000)>>24, (settings&0x00FF0000)>>16
		LH, LL = (settings&0x0000FF00)>>8, settings&0x000000FF
		self.write_ack(MID.SetOutputSettings, (HH, HL, LH, LL))
		self.settings = settings


	## Set the period of sampling.
	# Assume the device is in Config state.
	def SetPeriod(self, period):
		"""Set the period of sampling.
		Assume the device is in Config state."""
		H, L = (period&0xFF00)>>8, period&0x00FF
		self.write_ack(MID.SetPeriod, (H, L))


	## Set the output skip factor.
	# Assume the device is in Config state.
	def SetOutputSkipFactor(self, skipfactor):
		"""Set the output skip factor.
		Assume the device is in Config state."""
		H, L = (skipfactor&0xFF00)>>8, skipfactor&0x00FF
		self.write_ack(MID.SetOutputSkipFactor, (H, L))


	## Get data length.
	# Assume the device is in Config state.
	def ReqDataLength(self):
		"""Get data length.
		Assume the device is in Config state."""
		data = self.write_ack(MID.ReqDataLength)
		self.length, = struct.unpack('!H', data)
		self.header = '\xFA\xFF\x32'+chr(self.length)
		return self.length


	## Ask for the current configuration of the MT device.
	# Assume the device is in Config state.
	def ReqConfiguration(self):
		"""Ask for the current configuration of the MT device.
		Assume the device is in Config state."""
		data_ack = self.write_ack(MID.SetOutputConfiguration,())
		config = []
		try:
			for i in range(len(data_ack)/4):
				config.append(struct.unpack('!HH', data_ack[i*4:i*4+4]))				
		except struct.error:
			raise MTException("could not parse configuration.")
		return config

	## Set the baudrate of the device using the baudrate id.
	# Assume the device is in Config state.
	def SetBaudrate(self, brid):
		"""Set the baudrate of the device using the baudrate id.
		Assume the device is in Config state."""
		baudRateAck = False
		self.write_msg(MID.SetBaudrate, (brid,))
		dataAck = self.write_ack(MID.SetBaudrate, ())
		bridAck = struct.unpack('!B',dataAck)
		if bridAck[0] == brid:
			baudRateAck = True
		return baudRateAck

	## Request the available XKF scenarios on the device.
	# Assume the device is in Config state.
	def ReqAvailableScenarios(self):
		"""Request the available XKF scenarios on the device.
		Assume the device is in Config state."""
		scenarios_dat = self.write_ack(MID.ReqAvailableScenarios)
		scenarios = []
		try:
			for i in range(len(scenarios_dat)/22):
				scenario_type, version, label =\
						struct.unpack('!BB20s', scenarios_dat[22*i:22*(i+1)])
				scenarios.append((scenario_type, version, label.strip()))
			## available XKF scenarios
			self.scenarios = scenarios
		except struct.error:
			raise MTException("could not parse the available XKF scenarios.")
		return scenarios


	## Request the ID of the currently used XKF scenario.
	# Assume the device is in Config state.
	def ReqCurrentScenario(self):
		"""Request the ID of the currently used XKF scenario.
		Assume the device is in Config state."""
		data = self.write_ack(MID.SetCurrentScenario)
		## current XKF id
		self.scenario_id, = struct.unpack('!H', data)
		try:
			scenarios = self.scenarios
		except AttributeError:
			scenarios = self.ReqAvailableScenarios()
		for t, _, label in scenarios:
			if t==self.scenario_id:
				## current XKF label
				self.scenario_label = label
				break
		else:
			self.scenario_label = ""
		return self.scenario_id, self.scenario_label


	## Sets the XKF scenario to use.
	# Assume the device is in Config state.
	def SetCurrentScenario(self, scenario_id):
		"""Sets the XKF scenario to use.
		Assume the device is in Config state."""
		data = self.ReqAvailableScenarios()
		availableSc = numpy.array(data)
		validateSc = availableSc == str(scenario_id)
		if validateSc.any():
			self.write_ack(MID.SetCurrentScenario, (0x00, scenario_id&0xFF))
			print "Set to scenario:%2d"%scenario_id
		else:
			raise MTException("not an available XKF scenario")


	############################################################
	# High-level utility functions
	############################################################
	## Configure the mode and settings of the MT MK4 device.
	# this configures the MK4 devices to publish Xsens sensorSample format
	def configureMti(self, mtiSampleRate, mtiMode):
		"""Configure the mode and settings of the MTMk4 device."""
		self.GoToConfig()
		self.timeout = math.pow(mtiSampleRate,-1)+MID.additionalTimeOutOffset  # additional 5ms leeway
		print "Timeout changed to %1.3fs based on current settings."%(self.timeout)
		mid = MID.SetOutputConfiguration
		midReqDID = MID.ReqDID
		dataReqDID = (0x00, 0x00)
		dataDID = self.write_ack(midReqDID, dataReqDID)
		try:
			masterID = struct.unpack('!L', dataDID)
		except struct.error:
			raise MTException("could not parse configuration.")
		# to have a clear distinction between MTi-G-700 and 100-series devices
		deviceIDProductMask = hex(masterID[0]&0x00f00000)
		deviceTypeMask = hex(masterID[0]&0x0f000000)
		# check for user input for the delta q and delta v quantities
		new_imu_period = XDIMessage.DeltaQFs 
		if mtiSampleRate < int(new_imu_period):
			new_imu_period = mtiSampleRate
		# check for user input for the rate IMU quantities
		rate_imu_period = XDIMessage.RateOfTurnFs 
		if mtiSampleRate < int(rate_imu_period):
			rate_imu_period = mtiSampleRate	
		# check for user input for the mag quantities
		new_mag_period = XDIMessage.MagneticFieldFs 
		
		if mtiSampleRate < int(new_mag_period):
			new_mag_period = mtiSampleRate
		# check for user input for the  baro samples
		new_pressure_period = XDIMessage.PressureFs 
		if mtiSampleRate < int(new_pressure_period):
			new_pressure_period = mtiSampleRate
		
		if (deviceIDProductMask[2] == XDIProductMask.MTi1Series) | (deviceIDProductMask[2] == XDIProductMask.FMT1000Series):
			new_imu_period = rate_imu_period = new_mag_period = XDIMessage.FsModule
			if mtiSampleRate < int(new_imu_period):
				new_imu_period = rate_imu_period = new_mag_period = mtiSampleRate
		# All messages with corresponding output data rates
		"Packet couter, SampleTimeFine"
		# mPc = self.getMtiConfigBytes(XDIMessage.PacketCounter, XDIMessage.PaddedFs)
		mStf = self.getMtiConfigBytes(XDIMessage.SampleTimeFine, XDIMessage.PaddedFs)
		"Sensor data"
		mImuDq = self.getMtiConfigBytes(XDIMessage.DeltaQ, new_imu_period)
		mImuDv = self.getMtiConfigBytes(XDIMessage.DeltaV, new_imu_period)
		"Sensor data (rate quantities)"
		mImuMag = self.getMtiConfigBytes(XDIMessage.MagneticField, new_mag_period)
		mImuGyr = self.getMtiConfigBytes(XDIMessage.RateOfTurn, rate_imu_period)
		mImuAcc = self.getMtiConfigBytes(XDIMessage.Acceleration, rate_imu_period)
		"Baro data"
		mImuP = self.getMtiConfigBytes(XDIMessage.Pressure, new_pressure_period)
		"GNSS data"
		mGnssPvt = self.getMtiConfigBytes(XDIMessage.GnssPvtData, XDIMessage.GnssFs)
		mGnssSat = self.getMtiConfigBytes(XDIMessage.GnssSatInfo, XDIMessage.GnssFs)
		"Status word"
		mSw = self.getMtiConfigBytes(XDIMessage.StatusWord, XDIMessage.PaddedFs)
		# Filter related messages
		"Filter estimate"
		mOrientationQuat = self.getMtiConfigBytes(XDIMessage.OrientationQuat, rate_imu_period)
		mOrientation = self.getMtiConfigBytes(XDIMessage.Orientation,rate_imu_period)
		mVelocity = self.getMtiConfigBytes(XDIMessage.Velocity,rate_imu_period)
		mPosition = self.getMtiConfigBytes(XDIMessage.PositionLatLon,rate_imu_period)
		mHeight = self.getMtiConfigBytes(XDIMessage.PositionHeight,rate_imu_period)	
		
		# Output configuration set based on the product ID and user specification	
		if (deviceIDProductMask[2] == XDIProductMask.MTi100Series) & (deviceTypeMask[2] == XDIProductMask.MTi700Device):
			print "MTi-G-700/710 (GNSS/INS) device detected."
			if mtiMode == 1:
				print "Enabled publishing all sensor data"
				data = mStf+mImuDq+mImuDv+mImuMag+mImuP+mGnssPvt+mGnssSat+mSw+mOrientationQuat
			elif mtiMode == 2:
				print "Enabled publishing all sensor data (rate quantities)"
				data = mStf+mImuGyr+mImuAcc+mImuMag+mImuP+mGnssPvt+mGnssSat+mSw+mOrientationQuat
			elif mtiMode == 3:
				print "Enabled publishing all filter estimates"
				data = mStf+mSw+mOrientation+mVelocity+mPosition+mHeight
			else:
				raise MTException("unknown mtiMode: (%d)."%	(mtiMode))			
		elif deviceIDProductMask[2] == XDIProductMask.MTi100Series:
			print "MTi-100/200/300 device detected."
			if mtiMode == 1:
				print "Enabled publishing all sensor data"
				data = mStf+mImuDq+mImuDv+mImuMag+mImuP+mSw+mOrientationQuat
			elif mtiMode == 2:
				print "Enabled publishing all sensor data (rate quantities)"
				data = mStf+mImuGyr+mImuAcc+mImuMag+mImuP+mSw+mOrientationQuat
			elif mtiMode == 3:
				print "Enabled publishing all filter estimates"
				data = mStf+mSw+mOrientation
			else:
				raise MTException("unknown mtiMode: (%d)."%	(mtiMode))
		elif deviceIDProductMask[2] == XDIProductMask.MTi10Series:
			print "MTi-10/20/30 device detected"
			if mtiMode == 1:
				print "Enabled publishing all sensor data"
				data = mStf+mImuDq+mImuDv+mImuMag+mSw+mOrientationQuat
			elif mtiMode == 2:
				print "Enabled publishing all sensor data (rate quantities)"
				data = mStf+mImuGyr+mImuAcc+mImuMag+mSw+mOrientationQuat
			elif mtiMode == 3:
				print "Enabled publishing all filter estimates"
				data = mStf+mSw+mOrientation
			else:
				raise MTException("unknown mtiMode: (%d)."%	(mtiMode))
		elif deviceIDProductMask[2] == XDIProductMask.MTi1Series:
			print "MTi-1/2/3 device detected"
			if mtiMode == 1:
				print "Enabled publishing all sensor data"
				data = mStf+mImuDq+mImuDv+mImuMag+mSw+mOrientationQuat
			elif mtiMode == 2:
				print "Enabled publishing all sensor data (rate quantities)"
				data = mStf+mImuGyr+mImuAcc+mImuMag+mSw+mOrientationQuat
			elif mtiMode == 3:
				print "Enabled publishing all filter estimates"
				data = mStf+mSw+mOrientation
			else:
				raise MTException("unknown mtiMode: (%d)."%	(mtiMode))
		elif deviceIDProductMask[2] == XDIProductMask.FMT1000Series:
			print "FMT-1010/1020/1030 device detected"
			if mtiMode == 1:
				print "Enabled publishing all sensor data"
				data = mStf+mImuDq+mImuDv+mImuMag+mSw+mOrientationQuat
			elif mtiMode == 2:
				print "Enabled publishing all sensor data (rate quantities)"
				data = mStf+mImuGyr+mImuAcc+mImuMag+mSw+mOrientationQuat
			elif mtiMode == 3:
				print "Enabled publishing all filter estimates"
				data = mStf+mSw+mOrientation
			else:
				raise MTException("unknown mtiMode: (%d)."%	(mtiMode))	
		else:
			raise MTException("Unknown device")
		
		self.write_msg(mid, data)
		# check for the set baudrate
		dataAck = self.write_ack(MID.SetBaudrate, ())
		bridAck = struct.unpack('!B',dataAck)
		brSettings = Baudrates.get_BR(bridAck[0])
		print "Device configured at %1.0f bps"%(brSettings)
		self.GoToMeasurement()
		
	def getMtiConfigBytes(self, dataMessage, dataFs):
		H, L = (dataMessage&0xFF00)>>8, dataMessage&0x00FF
		HFs, LFs = (dataFs&0xFF00)>>8, dataFs&0x00FF
		message = [H, L, HFs, LFs] 
		return message


	## Read configuration from device.
	def auto_config(self):
		"""Read configuration from device."""
		self.GoToConfig()
		config =self.ReqConfiguration()
		Config = numpy.array(config)
		if Config.any():
			configuredMtiFs = numpy.max(Config[Config[:,1]!=65535,1])
			self.timeout = math.pow(configuredMtiFs,-1)+MID.additionalTimeOutOffset  # additional 5ms leeway
			print "Timeout defaults to %1.3fs based on output settings."%(self.timeout)
		else:
			self.timeout = 1+MID.additionalTimeOutOffset 
			print "Timeout defaults to %1.3fs based on output settings."%(self.timeout)
		self.GoToMeasurement()
		return self.timeout
		

	## Read and parse a measurement packet
	def read_measurement(self, mode=None, settings=None):
		# getting data
		mid, data = self.read_msg()
		if mid==MID.MTData2:
			return self.parse_MTData2(data)
		else:
			raise MTException("Only MTData2 supported, use -f and -m to configure MTi.\n unknown data message: mid=0x%02X (%s)."%	(mid, getMIDName(mid)))

	## Parse a new MTData2 message
	def parse_MTData2(self, data):
		# Functions to parse each type of packet
		def parse_temperature(data_id, content, ffmt):
			o = {}
			if (data_id&0x00F0) == 0x10:	# Temperature
				o['Temp'], = struct.unpack('!'+ffmt, content)
			else:
				raise MTException("unknown packet: 0x%04X."%data_id)
			return o
		def parse_timestamp(data_id, content, ffmt):
			o = {}
			if (data_id&0x00F0) == 0x10:	# UTC Time
				o['ns'], o['Year'], o['Month'], o['Day'], o['Hour'],\
						o['Minute'], o['Second'], o['Flags'] =\
						struct.unpack('!LHBBBBBB', content)
			elif (data_id&0x00F0) == 0x20:	# Packet Counter
				o['PacketCounter'], = struct.unpack('!H', content)
			elif (data_id&0x00F0) == 0x30:	# Integer Time of Week
				o['TimeOfWeek'], = struct.unpack('!L', content)
			elif (data_id&0x00F0) == 0x40:	# GPS Age
				o['gpsAge'], = struct.unpack('!B', content)
			elif (data_id&0x00F0) == 0x50:	# Pressure Age
				o['pressureAge'], = struct.unpack('!B', content)
			elif (data_id&0x00F0) == 0x60:	# Sample Time Fine
				o['SampleTimeFine'], = struct.unpack('!L', content)
			elif (data_id&0x00F0) == 0x70:	# Sample Time Coarse
				o['SampleTimeCoarse'], = struct.unpack('!L', content)
			elif (data_id&0x00F0) == 0x80:	# Frame Range
				o['startFrame'], o['endFrame'] = struct.unpack('!HH', content)
			else:
				raise MTException("unknown packet: 0x%04X."%data_id)
			return o
		def parse_orientation_data(data_id, content, ffmt):
			o = {}
			if (data_id&0x00F0) == 0x10:	# Quaternion
				o['Q0'], o['Q1'], o['Q2'], o['Q3'] = struct.unpack('!'+4*ffmt,
						content)
			elif (data_id&0x00F0) == 0x20:	# Rotation Matrix
				o['a'], o['b'], o['c'], o['d'], o['e'], o['f'], o['g'], o['h'],\
						o['i'] = struct.unpack('!'+9*ffmt, content)
			elif (data_id&0x00F0) == 0x30:	# Euler Angles
				o['Roll'], o['Pitch'], o['Yaw'] = struct.unpack('!'+3*ffmt,
						content)
			else:
				raise MTException("unknown packet: 0x%04X."%data_id)
			return o
		def parse_pressure(data_id, content, ffmt):
			o = {}			
			if (data_id&0x00F0) == 0x10:	# Baro pressure
				# FIXME is it really U4 as in the doc and not a float/double?
				o['Pressure'], = struct.unpack('!L', content)
			else:
				raise MTException("unknown packet: 0x%04X."%data_id)
			return o
		def parse_acceleration(data_id, content, ffmt):
			o = {}
			if (data_id&0x00F0) == 0x10:	# Delta V
				o['Delta v.x'], o['Delta v.y'], o['Delta v.z'] = \
						struct.unpack('!'+3*ffmt, content)
			elif (data_id&0x00F0) == 0x20:	# Acceleration
				o['accX'], o['accY'], o['accZ'] = \
						struct.unpack('!'+3*ffmt, content)
			elif (data_id&0x00F0) == 0x30:	# Free Acceleration
				o['freeAccX'], o['freeAccY'], o['freeAccZ'] = \
						struct.unpack('!'+3*ffmt, content)
			else:
				raise MTException("unknown packet: 0x%04X."%data_id)
			return o
		def parse_position(data_id, content, ffmt):
			o = {}
			heightFlag = False
			if (data_id&0x00F0) == 0x40:	# LatLon
				o['lat'], o['lon'] = struct.unpack('!'+2*ffmt, content)				
			elif (data_id&0x00F0) == 0x20:	# Altitude Ellipsoid
				o['ellipsoid'] = struct.unpack('!'+1*ffmt, content)
				heightFlag = True
			else:
				raise MTException("unknown packet: 0x%04X."%data_id)
			return o, heightFlag 
		def parse_angular_velocity(data_id, content, ffmt):
			o = {}
			# FIXME is it really 802y and 803y as in the doc?
			if (data_id&0x00F0) == 0x20:	# Rate of Turn
				o['gyrX'], o['gyrY'], o['gyrZ'] = \
						struct.unpack('!'+3*ffmt, content)
			elif (data_id&0x00F0) == 0x30:	# Delta Q
				o['Delta q0'], o['Delta q1'], o['Delta q2'], o['Delta q3'] = \
						struct.unpack('!'+4*ffmt, content)
			else:
				raise MTException("unknown packet: 0x%04X."%data_id)
			return o
		def parse_GNSS(data_id, content, ffmt):
			o = {}
			pvtFlag = False
			if (data_id&0x00F0) == 0x10:	# GNSS PVT DATA
				o['iTOW'],x1,x2,x3,x4,x5,x6,x7,x8,x9,o['fix'],o['flag'],o['nSat'],x10,lon,lat,h,a,hAcc, \
				vAcc,vN,vE,vD,x11,x12,sAcc,headAcc,headVeh,gDop,pDop,tDop,vDop,hDop,nDop,eDop = \
						struct.unpack('!LHBBBBBBLiBBBBiiiiLLiiiiiLLIHHHHHHH', content)
				o['lat'], o['lon'], o['hEll'], o['hMsl'], o['velN'], o['velE'], o['velD'], \
				o['horzAcc'], o['vertAcc'], o['speedAcc'], o['GDOP'],  o['PDOP'],  o['TDOP'],\
				o['VDOP'], o['HDOP'], o['NDOP'], o['EDOP'], o['heading'], o['headingAcc'] = 1e-7*lat, 1e-7*lon, 1e-3*h, \
						1e-3*a, 1e-3*vN, 1e-3*vE, 1e-3*vD, 1e-3*hAcc, 1e-3*vAcc, 1e-3*sAcc, 1e-2*gDop, \
						1e-2*pDop, 1e-2*tDop, 1e-2*vDop, 1e-2*hDop, 1e-2*nDop, 1e-2*eDop, 1e-5*headVeh, 1e-5*headAcc
				pvtFlag = True
			elif (data_id&0x00F0) == 0x20:	# GNSS SAT Info
				o['iTOW'], o['numCh'] = struct.unpack('!LBxxx', content[:8])
				channels = []
				ch = {}
				for i in range(o['numCh']):
					ch['gnssId'], ch['svId'], ch['cno'], ch['flags'] = \
							struct.unpack('!BBBB', content[8+4*i:12+4*i])
					channels.append(ch)
					ch = {} # empty 
				o['channels'] = channels
			else:
				raise MTException("unknown packet: 0x%04X."%data_id)
			return o, pvtFlag
		def parse_SCR(data_id, content, ffmt):
			o = {}
			if (data_id&0x00F0) == 0x10:	# ACC+GYR+MAG+Temperature
				o['accX'], o['accY'], o['accZ'], o['gyrX'], o['gyrY'], \
						o['gyrZ'], o['magX'], o['magY'], o['magZ'], o['Temp']=\
						struct.unpack("!9Hh", content)
			elif (data_id&0x00F0) == 0x20:	# Gyro Temperature
				o['tempGyrX'], o['tempGyrY'], o['tempGyrZ'] = \
						struct.unpack("!hhh", content)
			else:
				raise MTException("unknown packet: 0x%04X."%data_id)
			return o
		def parse_analog_in(data_id, content, ffmt):
			o = {}
			if (data_id&0x00F0) == 0x10:	# Analog In 1
				o['analogIn1'], = struct.unpack("!H", content)
			elif (data_id&0x00F0) == 0x20:	# Analog In 2
				o['analogIn2'], = struct.unpack("!H", content)
			else:
				raise MTException("unknown packet: 0x%04X."%data_id)
			return o
		def parse_magnetic(data_id, content, ffmt):
			o = {}
			if (data_id&0x00F0) == 0x20:	# Magnetic Field
				o['magX'], o['magY'], o['magZ'] = \
						struct.unpack("!3"+ffmt, content)
			else:
				raise MTException("unknown packet: 0x%04X."%data_id)
			return o
		def parse_velocity(data_id, content, ffmt):
			o = {}
			if (data_id&0x00F0) == 0x10:	# Velocity XYZ
				o['velX'], o['velY'], o['velZ'] = \
						struct.unpack("!3"+ffmt, content)
			else:
				raise MTException("unknown packet: 0x%04X."%data_id)
			return o
		def parse_status(data_id, content, ffmt):
			o = {}
			if (data_id&0x00F0) == 0x10:	# Status Byte
				o['StatusByte'], = struct.unpack("!B", content)
			elif (data_id&0x00F0) == 0x20:	# Status Word
				o['StatusWord'], = struct.unpack("!L", content)
			elif (data_id&0x00F0) == 0x40:	# RSSI
				o['RSSI'], = struct.unpack("!b", content)
			else:
				raise MTException("unknown packet: 0x%04X."%data_id)
			return o

		# data object
		output = {}
		while data:
			try:
				data_id, size = struct.unpack('!HB', data[:3])
				if (data_id&0x0003) == 0x3:
					float_format = 'd'
				elif (data_id&0x0003) == 0x0:
					float_format = 'f'
				else:
					raise MTException("fixed point precision not supported.")
				content = data[3:3+size]
				data = data[3+size:]
				group = data_id&0xFF00
				ffmt = float_format
				if group == XDIGroup.Temperature:
					output['Temperature'] = parse_temperature(data_id, content, ffmt)
				elif group == XDIGroup.Timestamp:
					output['Timestamp'] = parse_timestamp(data_id, content, ffmt)
				elif group == XDIGroup.OrientationData:
					output['Orientation Data'] = parse_orientation_data(data_id, content, ffmt)
				elif group == XDIGroup.Pressure:
					output['Pressure'] = parse_pressure(data_id, content, ffmt)
				elif group == XDIGroup.Acceleration:
					output['Acceleration'] = parse_acceleration(data_id, content, ffmt)
				elif group == XDIGroup.Position:
					temp, dataFlagPos = parse_position(data_id, content, ffmt)
					if dataFlagPos:
						output['Altitude'] = temp
					else:
						output['Latlon'] = temp
				elif group == XDIGroup.AngularVelocity:
					output['Angular Velocity'] = parse_angular_velocity(data_id, content, ffmt)
				elif group == XDIGroup.GNSS:
					temp, dataFlagGnss = parse_GNSS(data_id, content, ffmt)
					if dataFlagGnss:
						output['Gnss PVT'] = temp
					else:
						output['Gnss SATINFO'] = temp
				elif group == XDIGroup.SensorComponentReadout:
					output['SCR'] = parse_SCR(data_id, content, ffmt)
				elif group == XDIGroup.AnalogIn:
					output['Analog In'] = parse_analog_in(data_id, content, ffmt)
				elif group == XDIGroup.Magnetic:
					output['Magnetic'] = parse_magnetic(data_id, content, ffmt)
				elif group == XDIGroup.Velocity:
					output['Velocity'] = parse_velocity(data_id, content, ffmt)
				elif group == XDIGroup.Status:
					output['Status'] = parse_status(data_id, content, ffmt)
				else:
					raise MTException("unknown XDI group: 0x%04X."%group)
			except struct.error, e:
				raise MTException("couldn't parse MTData2 message.")
		return output

	## Parse a legacy MTData message
	## Change the baudrate, reset the device and reopen communication.
	def ChangeBaudrate(self, baudrate):
		"""Change the baudrate, reset the device and reopen communication."""
		self.GoToConfig()
		brid = Baudrates.get_BRID(baudrate)
		bridAck = self.SetBaudrate(brid)
		if bridAck: # Testing if the BR was set correctly
			self.device.baudrate=baudrate
			print "Baudrate set to %d bps"%baudrate
			time.sleep(0.01)
		else:
			print "NOK:Baudrate not configured."



################################################################
# Auto detect port
################################################################
def find_devices():
	mtdev_list = []
	for port in glob.glob("/dev/tty*S*"):
		try:
			br = find_baudrate(port)
			if br:
				mtdev_list.append((port, br))
		except MTException:
			pass
	return mtdev_list


################################################################
# Auto detect baudrate
################################################################
def find_baudrate(port):
	baudrates = (115200, 460800, 921600, 230400, 57600, 38400, 19200, 9600)
	for br in baudrates:
		try:
			mt = MTDevice(port, br)
		except serial.SerialException:
			raise MTException("unable to open %s"%port)
		try:
			mt.GoToConfig()
			mt.GoToMeasurement()
			return br
		except MTException:
			pass



################################################################
# Documentation for stand alone usage
################################################################
def usage():
		print """MT device driver.
Usage:
	./mtdevice.py [commands] [opts]

Commands:
	-h, --help
		Print this help and quit.
	-r, --reset
		Reset device to factory defaults.
	-f, --mtiSampleRate=SAMPLERATE
		Configures the device to the specified Output Data Rate (ODR).Possible 
		ODR's are 1,2,4,5,10,20,40,50,80,100,200 & 400 (the maximum output rate 
		for mag, baro and GNSS sensor is 100Hz, 50Hz and 4Hz respectively)
	-m, --sensorMode=SENSORMODE
		Configures the device to a particular sensor mode. The values can be 1
		(for sensor data),2 (for sensor data with rate quantities) or 
		3(for filter estimates). Use it in conjunction with -f command 		 			
	-a, --change-baudrate=NEW_BAUD
		Change baudrate from BAUD (see below) to NEW_BAUD.	
	-e, --echo
		Print MTData. It is the default if no other command is supplied.
	-i, --inspect
		Print current MT device configuration.
	-x, --xkf-scenario=ID
		Change the current XKF scenario.
Options:
	-d, --device=DEV
		Serial interface of the device (default: /dev/ttyUSB0). If 'auto', then
		all serial ports are tested at all baudrates and the first
		suitable device is used.
	-b, --baudrate=BAUD
		Baudrate of serial interface (default: 115200). If 0, then all
		rates are tried until a suitable one is found.
	-s, --skip-factor=SKIPFACTOR
		Number of samples to skip before sending MTData2 message
		(default: 0).
		The frequency at which MTData message is send is:
			115200/(PERIOD * (SKIPFACTOR + 1))
		If the value is 0xffff, no data is send unless a ReqData request
		is made.		
"""
################################################################
# Main function
################################################################
def main():
	# parse command line
	shopts = 'hra:eid:b:s:x:f:m:'
	lopts = ['help', 'reset', 'change-baudrate=', 'echo',
			'inspect', 'device=', 'baudrate=', 'skip-factor=', 
			'xkf-scenario=', 'mti-odr=','mti-mode=']
	try:
		opts, args = getopt.gnu_getopt(sys.argv[1:], shopts, lopts)
		#pdb.set_trace()
	except getopt.GetoptError, e:
		print e
		usage()
		return 1
	# default values
	device = '/dev/ttyUSB0'
	baudrate = 115200
	mode = 1
	settings = None
	period = None
	skipfactor = None
	new_baudrate = None
	new_xkf = None
	sampleRate = 100
	actions = []
	# filling in arguments
	for o, a in opts:
		if o in ('-h', '--help'):
			usage()
			return
		if o in ('-r', '--reset'):
			actions.append('reset')
		if o in ('-a', '--change-baudrate'):
			try:
				new_baudrate = int(a)
			except ValueError:
				print "change-baudrate argument must be integer."
				return 1
			actions.append('change-baudrate')
		if o in ('-e', '--echo'):
			actions.append('echo')
		if o in ('-i', '--inspect'):
			actions.append('inspect')
		if o in ('-x', '--xkf-scenario'):
			try:
				new_xkf = int(a)
			except ValueError:
				print "xkf-scenario argument must be integer."
				return 1
			actions.append('xkf-scenario')
		if o in ('-d', '--device'):
			device = a
		if o in ('-b', '--baudrate'):
			try:
				baudrate = int(a)
			except ValueError:
				print "Baudrate argument must be integer."
				return 1		
		if o in ('-s', '--skip-factor'):
			try:
				skipfactor = int(a)
			except ValueError:
				print "skip-factor argument must be integer."
				return 1
		if o in ('-f', '--mti-odr'):
			try:
				sampleRate = int(a)
				actions.append('setMtiOutputConfiguration')
			except ValueError:
				print "MTi sample rate argument must be integer."
				return 1	
		if o in ('-m','--mti-mode'):
			try:
				mode = int(a)
				actions.append('setMtiOutputConfiguration')
			except ValueError:
				print "MTi mode argument must be integer."
				return 1	
										
				
	# if nothing else: echo
	if len(actions) == 0:
		actions.append('echo')
	try:
		if device=='auto':
			devs = find_devices()
			if devs:
				print "Detected devices:","".join('\n\t%s @ %d'%(d,p) for d,p in
						devs)
				print "Using %s @ %d"%devs[0]
				device, baudrate = devs[0]
			else:
				print "No suitable device found."
				return 1
		# find baudrate
		if not baudrate:
			baudrate = find_baudrate(device)
		if not baudrate:
			print "No suitable baudrate found."
			return 1
		# open device
		try:
			mt = MTDevice(device, baudrate)
		except serial.SerialException:
			raise MTException("unable to open %s"%device)
		# execute actions
		if 'inspect' in actions:
			mt.GoToConfig()
			print "Device: %s at %d bps:"%(device, baudrate)
			print "General configuration:", mt.ReqConfiguration()
			print "Available scenarios:", mt.ReqAvailableScenarios()
			print "Current scenario: %s (id: %d)"%mt.ReqCurrentScenario()[::-1]
			mt.GoToMeasurement()
		if 'change-baudrate' in actions:
			print "Changing baudrate from %d to %d bps\n"%(baudrate, new_baudrate),
			sys.stdout.flush()
			mt.ChangeBaudrate(new_baudrate)
		if 'reset' in actions:
			print "Restoring factory defaults",
			sys.stdout.flush()
			mt.RestoreFactoryDefaults()
			print " Ok"		# should we test it was actually ok?
		if 'xkf-scenario' in actions:
			print "Changing XKF scenario..."
			sys.stdout.flush()
			mt.GoToConfig()
			mt.SetCurrentScenario(new_xkf)
			mt.GoToMeasurement()
			print "Ok"
		if 'setMtiOutputConfiguration' in actions:
			sys.stdout.flush()
			mt.GoToConfig()
			print "Device intiated at %d Hz"%(sampleRate)
			mt.configureMti(sampleRate,mode)
		if 'echo' in actions:
			try:
				while True:
					print mt.read_measurement(mode, settings)
			except KeyboardInterrupt:
				pass
	except MTException as e:
		#traceback.print_tb(sys.exc_info()[2])
		print e



if __name__=='__main__':
	main()

