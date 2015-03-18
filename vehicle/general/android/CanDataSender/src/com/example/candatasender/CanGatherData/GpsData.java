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

public class GpsData implements Serializable {
	private static final long serialVersionUID = -5705178908192954844L;
	public static final String name = "CanGatherGpsData";
	public long    timestamp;
	public Double  lat;
	public Double  lon;
	public Double  bearing;
	public Double  altitude;
	public Double  accuracy;
	public Double  speed;

	public static GpsData parse(String line) throws NumberFormatException {
		GpsData gps = new GpsData();

		String[] data = (line + ",z").split(","); // ",,"で終わらないようにするため
		if (data.length != 8) throw new NumberFormatException(name + " CSV format error: column = " + data.length);

		// timestamp[ms],lat,lon,bearing,altitude,accuracy,speed
		gps.timestamp = Long.parseLong(data[0].trim());
		gps.lat       = parseData(data[1]);
		gps.lon       = parseData(data[2]);
		gps.bearing   = parseData(data[3]);
		gps.altitude  = parseData(data[4]);
		gps.accuracy  = parseData(data[5]);
		gps.speed     = parseData(data[6]);

		return gps;
	}

	private static Double parseData(String data) throws NumberFormatException {
		String str = data.trim();
		if (str.length() == 0) return null;
		return Double.valueOf(str);
	}
}
