#ifndef POINTS_DOWNSAMPLER_H
#define POINTS_DOWNSAMPLER_H

static pcl::PointCloud<pcl::PointXYZI> removePointsByRange(pcl::PointCloud<pcl::PointXYZI> scan, double min_range, double max_range)
{
  pcl::PointCloud<pcl::PointXYZI> narrowed_scan;
  narrowed_scan.header = scan.header;

  max_range = 100.0;    //  This is a only tempolary patch for Localization problem.
  double square_min_range = min_range * min_range;
  double square_max_range = max_range * max_range;

  for(pcl::PointCloud<pcl::PointXYZI>::const_iterator iter = scan.begin(); iter != scan.end(); ++iter)
  {
    const pcl::PointXYZI &p = *iter;
//    p.x = iter->x;
//    p.y = iter->y;
//    p.z = iter->z;
//    p.intensity = iter->intensity;
    double square_distance = p.x * p.x + p.y * p.y;

    if(square_min_range <= square_distance && square_distance <= square_max_range){
      narrowed_scan.points.push_back(p);
    }
  }
#if 1
  return narrowed_scan;
#else
  return scan;    //  This is a only tempolary patch for Localization problem.
#endif
}

#endif // POINTS_DOWNSAMPLER_H
