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
