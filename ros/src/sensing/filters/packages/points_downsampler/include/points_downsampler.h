#ifndef POINTS_DOWNSAMPLER_H
#define POINTS_DOWNSAMPLER_H

static pcl::PointCloud<pcl::PointXYZI> removePointsByRange(pcl::PointCloud<pcl::PointXYZI> scan, double min_range, double max_range)
{
  pcl::PointCloud<pcl::PointXYZI> narrowed_scan;

  narrowed_scan.header = scan.header;

  for(pcl::PointCloud<pcl::PointXYZI>::const_iterator iter = scan.begin(); iter != scan.end(); ++iter)
  {
    pcl::PointXYZI p;
    p.x = iter->x;
    p.y = iter->y;
    p.z = iter->z;
    p.intensity = iter->intensity;
    double distance = sqrt(p.x * p.x + p.y * p.y);

    if(min_range <= distance && distance <= max_range){
      narrowed_scan.points.push_back(p);
    }
  }

  return narrowed_scan;
}

#endif // POINTS_DOWNSAMPLER_H
