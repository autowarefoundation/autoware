# Some Guidelines for Tuning the Ring Ground Filter

Author	: Patiphon Narksri

*Problem:* Some parts of vertical objects are detected as ground points.

*FIX:* One workaround for this is to set the "clipping_threshold" parameter in the launch file.
	  By setting this threshold, any points higher than this threshold will be detected as vertical points.
	  However, because all points that are higher than the threshold will be detected as vertical points, slopes might be incorrectly detected as vertical points as well.

---

*Problem:* Some small objects like curbs are missing.
*FIX:* Try to lower the "gap_thres" parameter.
	  However, lowering this parameter too much might result in mis-detecting slopes as vertical points.
	  Usually, 0.15 - 0.2 is enough for detecting curbs.

---

*Problem:* Ring shaped noise (ground points being detected as vertical points) occurs nearby the vehicle.
*FIX:* Try to lower the "points_distance" parameter.
	  However, lowering this parameter too much might result in mis-detecting vertical objects which are far away from the vehicle as ground points.

---

*Problem:* Line shaped noise (in radial direction) occurs near edges of vertical objects.
*FIX:* Decrease the "min_points" parameter. However, by doing so, some parts of vertical objects will be mis-detected as ground points.
