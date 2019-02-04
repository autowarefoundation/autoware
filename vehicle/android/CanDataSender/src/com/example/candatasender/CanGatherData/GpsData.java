/*
 * Copyright 2015 Autoware Foundation. All rights reserved.
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
