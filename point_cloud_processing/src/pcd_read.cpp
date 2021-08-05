#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/ksa/Desktop/Pasture_Monitoring/pcd_to_heights/test_pcd/max_heights_cloud.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }

  std::string path = "/home/ksa/Desktop/Pasture_Monitoring/pcd_to_heights/test_pcd/";
  std::string max_height_csv_file_path = path + "max_point_heights.csv";


  // // initiate height file for perimeter points
  // std::ofstream max_height_points_csv;
  // max_height_points_csv.open(max_height_csv_file_path);
  // max_height_points_csv << "X position" << "," << "Y position" << "," << "Z position" << "," << "Height" << std::endl;

  // // std::cout << "Loaded "
  // //           << cloud->width * cloud->height
  // //           << " data points from max_heights_cloud.pcd with the following fields: "
  // //           << std::endl;

  // // for (const auto& point: *cloud)
  // //   std::cout << "    " << point.x
  // //             << " "    << point.y
  // //             << " "    << point.z << std::endl;

  // for(size_t i = 0; i < max_heights_cloud->points.size(); i++)
  // {
  //     max_height_points_csv << plot_all_cloud->points[i].x << "," << plot_all_cloud->points[i].y << "," << plot_all_cloud->points[i].z << ","  << std::endl;
  // }

  // max_height_points_csv.close();


  return (0);
}