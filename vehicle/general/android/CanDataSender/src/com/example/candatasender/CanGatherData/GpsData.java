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
