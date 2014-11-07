package com.example.candatasender.CarLinkData;

import java.io.Serializable;

public class AccData implements Serializable {
	private static final long serialVersionUID = -7114294697673343971L;
	public static final String name = "CarLinkAccData";
	public long    timestamp;
	public double  x;
	public double  y;
	public double  z;

	public static AccData parse(String line) throws NumberFormatException {
		AccData acc = new AccData();

		String[] data = line.split(",");
		if (data.length != 4) throw new NumberFormatException(name + " CSV format error: column = " + data.length);

		// TimeStamp[ms],X[G],Y[G],Z[G]
		acc.timestamp = Long.parseLong(data[0].trim());
		acc.x         = Double.parseDouble(data[1].trim());
		acc.y         = Double.parseDouble(data[2].trim());
		acc.z         = Double.parseDouble(data[3].trim());

		return acc;
	}
}
