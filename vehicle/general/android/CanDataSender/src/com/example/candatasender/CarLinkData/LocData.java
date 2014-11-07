package com.example.candatasender.CarLinkData;

import java.io.Serializable;

public class LocData implements Serializable {
	private static final long serialVersionUID = -2652898043418591821L;
	public static final String name = "CarLinkLocData";
	public long    timestamp;
	public double  lat;
	public double  lon;

	public static LocData parse(String line) throws NumberFormatException {
		LocData loc = new LocData();

		String[] data = line.split(",");
		if (data.length != 4) throw new NumberFormatException(name + " CSV format error: column = " + data.length);

		// TimeStamp[ms],Lat,Lon
		loc.timestamp = Long.parseLong(data[0].trim());
		loc.lat       = Double.parseDouble(data[2].trim());
		loc.lon       = Double.parseDouble(data[3].trim());

		return loc;
	}
}
