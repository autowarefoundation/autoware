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

public class CanData implements Serializable {
	private static final long serialVersionUID = 5868650586794237676L;
	private static final int USB_COLUMN_NR = 5;
	private static final int BT_COLUMN_NR = 2;
	public static final String name = "CarLinkCanData";
	public long       timestamp;
	public double     value;
	public CanDataType type;
	//private static int dummyNum = 1; // dummysend

	public enum UsbBt {
		USB,
		BT,
		UNKNOWN;
	}

	public enum CanDataType {
		str(0x025),
		vel(0x610),
		gas(0x245),
		brake(0x224),
		seatbelt(0x620),
		unknown(0x000);

		private final int canId;
		private CanDataType(final int canId) {
			this.canId = canId;
		}
		public int getCanId() {
			return canId;
		}
		public static CanDataType valueOf(int canId) {
			for (CanDataType type: values()) {
				if (type.getCanId() == canId) {
					return type;
				}
			}
			return unknown;
		}
	}

	public static CanData parseUsb(String line) throws NumberFormatException {
		CanData can = new CanData();

		String[] data = line.split(",");
		if (data.length != USB_COLUMN_NR) throw new NumberFormatException(name + "(USB) CSV format error: column = " + data.length);

		// SysTimeStamp[ms],TimeStamp[us],ID(hhh),DataLength(h),Data(hh...)
		can.timestamp = Long.parseLong(data[0]);
		can.type = CanDataType.valueOf(Integer.parseInt(data[2], 16));
		switch (can.type) {
		case str:
			short sstr = (short)Integer.parseInt(data[4].substring(1, 4) + "0", 16);
			can.value = (double)(sstr >> 4) * 1.5;
			break;
		case vel:
			int ivel = Integer.parseInt(data[4].substring(4, 6), 16);
			can.value = (double)ivel;
			break;
		case gas:
			int igas = Integer.parseInt(data[4].substring(4, 6), 16);
			can.value = (double)igas * 0.5;
			break;
		case brake:
			int ibrake = Integer.parseInt(data[4].substring(8, 12), 16);
			can.value = (double)ibrake * 0.00125;
			break;
		case seatbelt:
			int isb = Integer.parseInt(data[4].substring(14, 15), 16);
			can.value = (double)isb * 0.015625;
			break;
		case unknown:
			//can.type = CanDataType.str; // dummysend
			//can.value = dummyNum++; // dummysend
		default:
			break;
		}
		return can;
	}

	public static CanData parseBt(String line) throws NumberFormatException {
		CanData can = new CanData();

		String[] data = line.split(",");
		if (data.length != BT_COLUMN_NR) throw new NumberFormatException(name + "(BT) CSV format error: column = " + data.length);

		// TimeStamp[ms],RawMsg['t'+ID(hhh)+DataLength(h)+Data(hh...)]
		can.timestamp = Long.parseLong(data[0]);
		can.type = CanDataType.valueOf(Integer.parseInt(data[1].substring(1, 4), 16));
		switch (can.type) {
		case str:
			short sstr = (short)Integer.parseInt(data[1].substring(6, 9) + "0", 16);
			can.value = (double)(sstr >> 4) * 1.5;
			break;
		case vel:
			int ivel = Integer.parseInt(data[1].substring(9, 11), 16);
			can.value = (double)ivel;
			break;
		case gas:
			int igas = Integer.parseInt(data[1].substring(9, 11), 16);
			can.value = (double)igas * 0.5;
			break;
		case brake:
			int ibrake = Integer.parseInt(data[1].substring(13, 17), 16);
			can.value = (double)ibrake * 0.00125;
			break;
		case seatbelt:
			int isb = Integer.parseInt(data[1].substring(19, 20), 16);
			can.value = (double)isb * 0.015625;
			break;
		case unknown:
		default:
			break;
		}
		return can;
	}

	public static CanData parse(String line, UsbBt usbbt) throws NumberFormatException {
		CanData can = null;

		switch (usbbt) {
		case USB:
			can = parseUsb(line);
			break;
		case BT:
			can = parseBt(line);
			break;
		default:
			break;
		}
		return can;
	}
	
	public static UsbBt parseUsbBt(String line) {
		String[] data = line.split(",");
		switch (data.length) {
		case USB_COLUMN_NR:
			return UsbBt.USB;
		case BT_COLUMN_NR:
			return UsbBt.BT;
		default:
			return UsbBt.UNKNOWN;
		}
	}
}
