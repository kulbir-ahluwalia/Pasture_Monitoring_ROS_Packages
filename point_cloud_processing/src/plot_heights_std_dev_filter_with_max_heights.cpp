#include <iostream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <math.h>
#include <string>
#include <fstream>
#include <sstream>
#include <cstdlib>


struct Vector3
{
    double x;
    double y;
    double z;
};

int main(int argc, char** argv)
{
    if(argc != 3)
    {
        std::cerr << "plot_only_param_file plot_all_pcd_file" << std::endl;
        return -1;
    }
    
    float negative_x_filter = 0;
    float positive_x_filter = 0;
    float negative_y_filter = 0;
    float positive_y_filter = 0;
    float negative_z_filter = 0;
    float positive_z_filter = 0;
    float rotation_x = 0;
    float rotation_y = 0;
    float rotation_z = 0;

    std::ifstream infile(argv[1]);

    int count = 0;
    float param = 0;
    while(infile >> param) {
        switch(count) {
            case 0:
                negative_x_filter = param;
                break;
            case 1:
                positive_x_filter = param;
                break;
            case 2:
                negative_y_filter = param;
                break;
            case 3:
                positive_y_filter = param;
                break;
            case 4:
                negative_z_filter = param;
                break;
            case 5:
                positive_z_filter = param;
                break;
            case 6:
                rotation_x = param;
                break;
            case 7:
                rotation_y = param;
                break;
            case 8:
                rotation_z = param;
                break;    
            default:
                std::cout << "Shouldn't get to this. Param file setup incorrectly." << std::endl;
                break;           
        }
        count++;
    }  

    std::cout << "Importing plot_all pcd file" << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr plot_all_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if(pcl::io::loadPCDFile<pcl::PointXYZ> (argv[2], *plot_all_cloud) == -1)
    {
        PCL_ERROR("Couldn't read file ");
        PCL_ERROR(argv[2]);
        PCL_ERROR("\n");
        return (-1);
    }

    









    /*
    int perimeter_count = 0;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    for(size_t i = 0; i < plot_only_cloud->size(); i++)
    {
        
        for(size_t j = 0; j < plot_all_cloud->size(); j++)
        {
            //std::cout << plot_all_cloud->points[j].x << " " << plot_only_cloud->points[i].x << std::endl;
            //if(true)
            if(plot_all_cloud->points[j].x == plot_only_cloud->points[i].x &&
               plot_all_cloud->points[j].y == plot_only_cloud->points[i].y &&
               plot_all_cloud->points[j].z == plot_only_cloud->points[i].z)
            {
                  //pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
                  inliers->indices.push_back(j);
                  perimeter_count++;
                  std::cout << "Perimeter points count: " << perimeter_count << std::endl;
                  
                  extract.setInputCloud(plot_all_cloud);
                  extract.setIndices(inliers);
                  extract.setNegative(true);
                  extract.filter(*plot_all_cloud);
                  
                  break;
            }
            
        }
        
    }

    extract.setInputCloud(plot_all_cloud);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*plot_all_cloud);
    */

    std::cout << "Filtering plot out using parameters file" << std::endl;

    //heights of plants = PLOT ONLY
    pcl::PointCloud<pcl::PointXYZ>::Ptr plot_only_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::CropBox<pcl::PointXYZ> boxFilter;
    //boxFilter.setTranslation(Eigen::Vector3f(4, 1.5, 0));
    boxFilter.setMin(Eigen::Vector4f(negative_x_filter, negative_y_filter, negative_z_filter, 1.0));
    boxFilter.setMax(Eigen::Vector4f(positive_x_filter, positive_y_filter, positive_z_filter, 1.0));
    //boxFilter.setMin(Eigen::Vector4f(-1, -3, -100, 1.0));
    //boxFilter.setMax(Eigen::Vector4f(1, 3, 100, 1.0));
    boxFilter.setRotation(Eigen::Vector3f(rotation_x, rotation_y, rotation_z));
    boxFilter.setInputCloud(plot_all_cloud);
    //boxFilter.setNegative(true);
    boxFilter.filter(*plot_only_cloud);

    //heights of points in perimeter
    std::cout << "Filtering perimeter out using parameters file" << std::endl;
    //boxFilter.setTranslation(Eigen::Vector3f(4, 1.5, 0));
    boxFilter.setMin(Eigen::Vector4f(negative_x_filter, negative_y_filter, negative_z_filter, 1.0));
    boxFilter.setMax(Eigen::Vector4f(positive_x_filter, positive_y_filter, positive_z_filter, 1.0));
    //boxFilter.setMin(Eigen::Vector4f(-1, -3, -100, 1.0));
    //boxFilter.setMax(Eigen::Vector4f(1, 3, 100, 1.0));
    boxFilter.setRotation(Eigen::Vector3f(rotation_x, rotation_y, rotation_z));
    boxFilter.setInputCloud(plot_all_cloud);
    boxFilter.setNegative(true);
    boxFilter.filter(*plot_all_cloud);

    std::cout << "Saving perimeter and plot files in current directory" << std::endl;

    std::string path = "/home/ksa/Desktop/Pasture_Monitoring/pcd_to_heights/test_pcd/";
    std::string perimeter_pc_path = path + "perimeter.pcd";
    std::string plot_only_pc_path = path + "plot_only.pcd";
    std::string plot_plant_height_csv_file_path = path + "plot_plant_heights.csv";

    std::string plot_plant_one_std_height_csv_file_path = path + "plot_plant_one_std_heights.csv";
    std::string plot_plant_two_std_height_csv_file_path = path + "plot_plant_two_std_heights.csv";
    std::string plot_plant_four_std_height_csv_file_path = path + "plot_plant_four_std_heights.csv";

    std::string perimeter_height_csv_file_path = path + "perimeter_point_heights.csv";


    pcl::io::savePCDFileASCII(perimeter_pc_path, *plot_all_cloud);
    pcl::io::savePCDFileASCII(plot_only_pc_path, *plot_only_cloud);

    // /home/ksa/Desktop/Pasture_Monitoring/pcd_to_heights/test_pcd.pcd
    // initiate height file for plot points
    std::ofstream height_file_plot_plants;
    height_file_plot_plants.open(plot_plant_height_csv_file_path);
    height_file_plot_plants << "X position" << "," << "Y position" << "," << "Z position" << "," << "Height" << std::endl;

    // initiate height file for plot points greater than (mean +one std)
    std::ofstream height_file_plot_plants_one_std;
    height_file_plot_plants_one_std.open(plot_plant_one_std_height_csv_file_path);
    height_file_plot_plants_one_std << "X position" << "," << "Y position" << "," << "Z position" << "," << "Height" << std::endl;

    // initiate height file for plot points greater than (mean +two std)
    std::ofstream height_file_plot_plants_two_std;
    height_file_plot_plants_two_std.open(plot_plant_two_std_height_csv_file_path);
    height_file_plot_plants_two_std << "X position" << "," << "Y position" << "," << "Z position" << "," << "Height" << std::endl;

    // initiate height file for plot points greater than (mean +four std)
    std::ofstream height_file_plot_plants_four_std;
    height_file_plot_plants_four_std.open(plot_plant_four_std_height_csv_file_path);
    height_file_plot_plants_four_std << "X position" << "," << "Y position" << "," << "Z position" << "," << "Height" << std::endl;


    // initiate height file for perimeter points
    std::ofstream height_file_perimeter_plants;
    height_file_perimeter_plants.open(perimeter_height_csv_file_path);
    height_file_perimeter_plants << "X position" << "," << "Y position" << "," << "Z position" << "," << "Height" << std::endl;



    std::cout << "Computing heights of perimeter" << std::endl;
    Vector3 sum;
    sum.x = 0;
    sum.y = 0;
    sum.z = 0;
    for(size_t i = 0; i < plot_all_cloud->points.size(); i++)
    {
        sum.x += plot_all_cloud->points[i].x;
        sum.y += plot_all_cloud->points[i].y;
        sum.z += plot_all_cloud->points[i].z;
    }

    Vector3 centroid;
    centroid.x = sum.x / plot_all_cloud->size();
    centroid.y = sum.y / plot_all_cloud->size();
    centroid.z = sum.z / plot_all_cloud->size();

    // plot_all_cloud = perimeter point cloud
    // centroid.z = average height of z points
    float average_z_height = centroid.z;
    std::cout << "average_z_height is: " << average_z_height << std::endl;
    std::cout << "number of points is: " << plot_all_cloud->size() << std::endl;

    float std_dev_numerator = 0.0;
    float std_dev_perimeter = 0.0;

    //find standard deviation of z heights of perimeter
    for(size_t i = 0; i < plot_all_cloud->points.size(); i++)
    {
        std_dev_numerator += pow(plot_all_cloud->points[i].z - average_z_height, 2);
    }

    std_dev_perimeter = sqrt(std_dev_numerator / plot_all_cloud->size());
    std::cout << "standard deviation of perimeter is: " << std_dev_perimeter << std::endl;





    //linearly sqaures estimate


    double xx = 0.0;
    double xy = 0.0;
    double xz = 0.0;
    double yy = 0.0;
    double yz = 0.0;
    double zz = 0.0;

    for(size_t i = 0; i < plot_all_cloud->points.size(); i++)
    {
       Vector3 r;
       r.x = plot_all_cloud->points[i].x - centroid.x;
       r.y = plot_all_cloud->points[i].y - centroid.y;
       r.z = plot_all_cloud->points[i].z - centroid.z;
       xx += r.x * r.x;
       xy += r.x * r.y;
       xz += r.x * r.z;
       yy += r.y * r.y;
       yz += r.y * r.z;
       zz += r.z * r.z;
    }

    double det_x = yy*zz - yz*yz;
    double det_y = xx*zz - xz*xz;
    double det_z = xx*yy - xy*xy;

    double max = det_x;
    int max_letter = 1;
    if(det_y > max)
    {
        max = det_y;
        max_letter = 2;
    }
    if(det_z > max)
    {
        max = det_z;
        max_letter = 3;
    }

    Vector3 dir;
    if(max_letter == 1)
    {
        dir.x = det_x;
        dir.y = xz*yz - xy*zz;
        dir.z = xy*yz - xz*yy;
    }
    else if(max_letter == 2)
    {
        dir.x = xz*yz - xy*zz;
        dir.y = det_y;
        dir.z = xy*xz - yz*xx;
    }
    else if(max_letter == 3)
    {
        dir.x = xy*yz - xz*yy;
        dir.y = xy*xz - yz*xx;
        dir.z = det_z;
    }

    double length = (dir.x * dir.x) + (dir.y * dir.y) + (dir.z * dir.z);
    length = sqrt(length);

    dir.x = dir.x / length;
    dir.y = dir.y / length;
    dir.z = dir.z / length;


    //double d = (dir.x * centroid.x * -1) + (dir.y * centroid.y * -1) + (dir.z * centroid.z * -1);

    //save plot estimated heights
    double distance = 0;
    Vector3 subtract;
    int index = -1;

    //create a new point cloud
    // pcl::PointCloud<pcl::PointXYZ>::Ptr max_heights_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> max_heights_cloud;


    max_heights_cloud.width    = 5000000;
    max_heights_cloud.height   = 1;
    max_heights_cloud.is_dense = false;
    max_heights_cloud.points.resize (max_heights_cloud.width * max_heights_cloud.height);

    int j =0;

    for(size_t i = 0; i < plot_only_cloud->points.size(); i++)
    {
        double temp_distance = 0;
        subtract.x = plot_only_cloud->points[i].x - centroid.x;
        subtract.y = plot_only_cloud->points[i].y - centroid.y;
        subtract.z = plot_only_cloud->points[i].z - centroid.z;
        //temp_distance = (plot_only_cloud->points[i].x * dir.x) + (plot_only_cloud->points[i].y * dir.y) + (plot_only_cloud->points[i].z * dir.z) + d;
        //temp_distance = std::abs(temp_distance);
        temp_distance = (dir.x * subtract.x) + (dir.y * subtract.y) + (dir.z * subtract.z);

        double mag_dist = std::abs(temp_distance);

        // height_file_plot_plants << plot_only_cloud->points[i].x << "," << plot_only_cloud->points[i].y << "," << plot_only_cloud->points[i].z << "," << mag_dist << std::endl;

        // if (plot_only_cloud->points[i].z >= (average_z_height + std_dev_perimeter))
        // {
        //     height_file_plot_plants_one_std << plot_only_cloud->points[i].x << "," << plot_only_cloud->points[i].y << "," << plot_only_cloud->points[i].z << "," << mag_dist << std::endl;
        // }

        // if (plot_only_cloud->points[i].z >= (average_z_height + (2*std_dev_perimeter)))
        // {
        //     height_file_plot_plants_two_std << plot_only_cloud->points[i].x << "," << plot_only_cloud->points[i].y << "," << plot_only_cloud->points[i].z << "," << mag_dist << std::endl;
        // }


        //
        if (plot_only_cloud->points[i].z >= (average_z_height + (4*std_dev_perimeter)))
        {
            height_file_plot_plants_four_std << plot_only_cloud->points[i].x << "," << plot_only_cloud->points[i].y << "," << plot_only_cloud->points[i].z << "," << mag_dist << std::endl;

            
            
            max_heights_cloud[j].x = plot_only_cloud->points[i].x;
            max_heights_cloud[j].y = plot_only_cloud->points[i].y;
            max_heights_cloud[j].z = plot_only_cloud->points[i].z;


            j = j+1;

            // for (const auto& point: max_heights_cloud)
            // {
            //     std::cerr << "    " << point.x << " " << point.y << " " << point.z << std::endl;
            // }

        }
       

    }

    for(size_t i = 0; i < plot_all_cloud->points.size(); i++)
    {
        height_file_perimeter_plants << plot_all_cloud->points[i].x << "," << plot_all_cloud->points[i].y << "," << plot_all_cloud->points[i].z << ","  << std::endl;
    }

    //save perimeter points as well in the csv file

    // pcl::io::savePCDFileASCII(plot_only_pc_path, *plot_only_cloud);

    std::string max_heights_path = "/home/ksa/Desktop/Pasture_Monitoring/pcd_to_heights/test_pcd/";
    std::string max_heights_pc_path = path + "max_heights_pc.pcd";

    pcl::io::savePCDFileASCII (max_heights_pc_path, max_heights_cloud);

    std::cerr << "Saved " << max_heights_cloud.size () << " data points to max_heights_cloud.pcd." << std::endl;
    
    std::cout << "Saving plot_heights.csv file in test_pcd directory" << std::endl;
    height_file_plot_plants.close();
    height_file_plot_plants_one_std.close();
    height_file_plot_plants_two_std.close();
    // height_file_plot_plants_three_std.close();
    height_file_plot_plants_four_std.close();
    height_file_perimeter_plants.close();
 
    return 0;
}
