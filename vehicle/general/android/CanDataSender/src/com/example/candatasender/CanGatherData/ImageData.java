package com.example.candatasender.CanGatherData;

import java.io.File;
import java.io.Serializable;
import java.util.Date;
import java.text.SimpleDateFormat;
import java.text.ParsePosition;

public class ImageData implements Serializable {
	private static final long serialVersionUID = 1641161126966925369L;
	public static final String name = "CanGatherImageData";
	public long    timestamp;
	public String  filename;

	public static ImageData parse(String line) {
		ImageData img = new ImageData();

		File abstractFile = new File(line);
		String filename = abstractFile.getName();
		String ts = filename.substring(0, filename.length() - ".jpg".length());
		SimpleDateFormat sdf = new SimpleDateFormat("yyyy-MM-dd-HH-mm-ss-SSS");
		ParsePosition pos = new ParsePosition(0);
		Date date = sdf.parse(ts, pos);
		if (date == null) return null;

		img.timestamp  = date.getTime();
		img.filename   = line;
		return img;
	}
}
