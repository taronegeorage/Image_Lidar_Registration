#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

int main() 
{
    pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob(new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>),
        cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    
    // read data
    pcl::PCDReader reader;
    // /home/cj/Downloads/outdoor_data/pc/velodyne/000799.pcd /home/cj/ros_ws2/src/RGB_TO_LIDAR/data/finalCloud.pcd
    reader.read("/home/cj/ros_ws2/src/RGB_TO_LIDAR/data/finalCloud.pcd", *cloud_blob);
    std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;

    // create filtering object
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud_blob);
    sor.setLeafSize(0.1f, 0.1f, 0.1f);
    sor.filter(*cloud_filtered_blob);

    // Convert to the templated PC
    pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);
    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

    // write
    pcl::PCDWriter writer;
    //writer.write<pcl::PointXYZ> (".pcd", *cloud_filtered);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients() );
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices() );
    // create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold (0.5);

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    int i = 0, nr_points = (int) cloud_filtered->points.size();
    while(cloud_filtered->points.size() > 0.2*nr_points) {
        // segment the largest planar component from the remainning cloud
        seg.setInputCloud (cloud_filtered);
        seg.segment(*inliers, *coefficients);
        if(inliers->indices.size() == 0) {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // if(fabs(coefficients->values[0]+coefficients->values[1])<0.5)
        //     continue;
        // Extract the inliers
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_p);
        std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;
        if(fabs(coefficients->values[1])<0.9) {
            std::stringstream ss;
            ss << "000799_plane_" << i << ".pcd";
            writer.write<pcl::PointXYZ> (ss.str(), *cloud_p);
        }
        std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;

        // create the filtering object
        extract.setNegative (true);
        extract.filter(*cloud_f);
        cloud_filtered.swap(cloud_f);
        i++;
    }
    writer.write<pcl::PointXYZ> ("left.pcd", *cloud_f);
    return 0;
}