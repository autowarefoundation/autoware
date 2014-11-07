package com.example.candatasender.CanGatherData;

import java.io.Serializable;

public class ItsecuData implements Serializable {
	private static final long serialVersionUID = -7780858658830630720L;
	public static final String name = "CanGatherItsecuData";
	public long    timestamp;
	public Double  str;
	public Double  vel;
	public Double  waterTemperature;
	public Double  gas;

	public static ItsecuData parse(String line) throws NumberFormatException {
		ItsecuData itsecu = new ItsecuData();

		String[] data = (line + ",z").split(","); // ",,"で終わらないようにするため
		if (data.length != 6) throw new NumberFormatException(name + " CSV format error: column = " + data.length);

		// timestamp[ms],str,vel,waterTemp,gas
		itsecu.timestamp        = Long.parseLong(data[0].trim());
		itsecu.str              = parseData(data[1]);
		itsecu.vel              = parseData(data[2]);
		itsecu.waterTemperature = parseData(data[3]);
		itsecu.gas              = parseData(data[4]);

		return itsecu;
	}

	private static Double parseData(String data) throws NumberFormatException {
		String str = data.trim();
		if (str.length() == 0) return null;
		return Double.valueOf(str);
	}
}
