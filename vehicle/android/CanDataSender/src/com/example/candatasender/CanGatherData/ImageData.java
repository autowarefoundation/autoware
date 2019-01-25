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
