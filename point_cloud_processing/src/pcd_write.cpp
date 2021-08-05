#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int
  main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ> max_cloud;

  // Fill in the cloud data
  max_cloud.width    = 5;
  max_cloud.height   = 1;
  max_cloud.is_dense = false;
  max_cloud.points.resize (max_cloud.width * max_cloud.height);

  for (auto& point: max_cloud)
  {
    point.x = 1024 * rand () / (RAND_MAX + 1.0f);
    point.y = 1024 * rand () / (RAND_MAX + 1.0f);
    point.z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  pcl::io::savePCDFileASCII ("test_pcd.pcd", max_cloud);
  
  std::cerr << "Saved " << max_cloud.size () << " data points to test_pcd.pcd." << std::endl;

  for (const auto& point: max_cloud)
    std::cerr << "    " << point.x << " " << point.y << " " << point.z << std::endl;

  return (0);
}