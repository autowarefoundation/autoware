package com.example.candatasender.CanGatherData;

import java.io.Serializable;

public class EnvData implements Serializable {
	private static final long serialVersionUID = -447612400659939005L;
	public static final String name = "CanGatherEnvData";
	public long    timestamp;
	public Double  battery;
	public Double  temperature;

	public static EnvData parse(String line) throws NumberFormatException {
		EnvData env = new EnvData();

		String[] data = (line + ",z").split(","); // ",,"で終わらないようにするため
		if (data.length != 4) throw new NumberFormatException(name + " CSV format error: column = " + data.length);

		// timestamp[ms],battery,temperature
		env.timestamp   = Long.parseLong(data[0].trim());
		env.battery     = parseData(data[1]);
		env.temperature = parseData(data[2]);

		return env;
	}

	private static Double parseData(String data) throws NumberFormatException {
		String str = data.trim();
		if (str.length() == 0) return null;
		return Double.valueOf(str);
	}
}
