/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
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

package com.example.candatasender.CanGatherData;

import java.io.Serializable;

public class SensorData implements Serializable {
	private static final long serialVersionUID = -8226799362703816492L;
	public static final String name = "CanGatherSensorData";
	public long    timestamp;
	public Double  calOrientationX;
	public Double  calOrientationY;
	public Double  calOrientationZ;
	public Double  accX;
	public Double  accY;
	public Double  accZ;
	public Double  gravityX;
	public Double  gravityY;
	public Double  gravityZ;
	public Double  gyroscopeX;
	public Double  gyroscopeY;
	public Double  gyroscopeZ;
	public Double  light;
	public Double  linearAccX;
	public Double  linearAccY;
	public Double  linearAccZ;
	public Double  magneticFieldX;
	public Double  magneticFieldY;
	public Double  magneticFieldZ;
	public Double  orientationX;
	public Double  orientationY;
	public Double  orientationZ;
	public Double  pressure;
	public Double  proximity;
	public Double  rotationVectorX;
	public Double  rotationVectorY;
	public Double  rotationVectorZ;
	public Double  rotationVectorC;
	public Double  temperature;
	public Double  ambientTemperature;
	public Double  relativeHumidity;

	public static SensorData parse(String line) throws NumberFormatException {
		SensorData sensor = new SensorData();

		String[] data = (line + ",z").split(","); // ",,"で終わらないようにするため
		if (data.length != 33) throw new NumberFormatException(name + " CSV format error: column = " + data.length);

		sensor.timestamp          = Long.parseLong(data[0].trim());
		sensor.calOrientationX    = parseData(data[1]);
		sensor.calOrientationY    = parseData(data[2]);
		sensor.calOrientationZ    = parseData(data[3]);
		sensor.accX               = parseData(data[4]);
		sensor.accY               = parseData(data[5]);
		sensor.accZ               = parseData(data[6]);
		sensor.gravityX           = parseData(data[7]);
		sensor.gravityY           = parseData(data[8]);
		sensor.gravityZ           = parseData(data[9]);
		sensor.gyroscopeX         = parseData(data[10]);
		sensor.gyroscopeY         = parseData(data[11]);
		sensor.gyroscopeZ         = parseData(data[12]);
		sensor.light              = parseData(data[13]);
		sensor.linearAccX         = parseData(data[14]);
		sensor.linearAccY         = parseData(data[15]);
		sensor.linearAccZ         = parseData(data[16]);
		sensor.magneticFieldX     = parseData(data[17]);
		sensor.magneticFieldY     = parseData(data[18]);
		sensor.magneticFieldZ     = parseData(data[19]);
		sensor.orientationX       = parseData(data[20]);
		sensor.orientationY       = parseData(data[21]);
		sensor.orientationZ       = parseData(data[22]);
		sensor.pressure           = parseData(data[23]);
		sensor.proximity          = parseData(data[24]);
		sensor.rotationVectorX    = parseData(data[25]);
		sensor.rotationVectorY    = parseData(data[26]);
		sensor.rotationVectorZ    = parseData(data[27]);
		sensor.rotationVectorC    = parseData(data[28]);
		sensor.temperature        = parseData(data[29]);
		sensor.ambientTemperature = parseData(data[30]);
		sensor.relativeHumidity   = parseData(data[31]);

		return sensor;
	}

	private static Double parseData(String data) throws NumberFormatException {
		String str = data.trim();
		if (str.length() == 0) return null;
		return Double.valueOf(str);
	}
}
