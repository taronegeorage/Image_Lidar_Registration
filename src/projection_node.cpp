// laser /home/cj/Downloads/outdoor_data/pc/velodyne/000441.pcd  //799
//img /home/cj/Downloads/outdoor_data/image/4_distort/000449.jpg  //810
// /home/cj/ros_ws2/src/RGB_TO_LIDAR/data/surfaceMap.pcd
#include<iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "fisheyeimg.h"

using namespace std;

int main(int argc, char **argv)
{
    Fisheye *f = new Fisheye();
    float fx, fy, cx, cy;
    f->getIntrinsics(fx, fy, cx, cy);
    int w, h;
    f->getImagesize(w, h);
    Eigen::Matrix4d Tcl = f->getTcl();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cld(new pcl::PointCloud<pcl::PointXYZ>);
	if(pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/cj/Downloads/outdoor_data/pc/velodyne/000799.pcd", *cld) == -1) {
		PCL_ERROR("CANNOT READ FILE");
		return -1;
	}
	cout << "Loaded " << cld->width << "*" << cld->height << " data points." << endl;
    pcl::transformPointCloud(*cld, *cld, Tcl);
    std::cout << Tcl.matrix() << std::endl;
    pcl::io::savePCDFileBinary("newpc.pcd", *cld );
    cv::Mat img = cv::imread("/home/cj/Downloads/outdoor_data/image/4_undistort/000810.jpg");

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients() );
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices() );
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold (0.01);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PCDWriter writer;
    double threshold = 0.8 * cld->points.size();
    int num_planes = 0;
    vector<pcl::PointCloud<pcl::PointXYZ> > planes;
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>),
        cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cld, *tmp);

    while(tmp->points.size() > threshold) {
        seg.setInputCloud(tmp);
        seg.segment(*inliers, *coefficients);
        if(inliers->indices.size() == 0) {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            exit(-1);
        }
        std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;
        extract.setInputCloud(tmp);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_p);
        if(fabs(coefficients->values[0])>0.9 || fabs(coefficients->values[2])>0.9) {
            // remove outliers
            pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
            outrem.setInputCloud(cloud_p);
            outrem.setRadiusSearch(0.7);
            outrem.setMinNeighborsInRadius(5);
            outrem.filter(*cloud_p);
            std::cerr << "plane " << num_planes << std::endl;
            num_planes++;
            planes.push_back(*cloud_p);
            std::stringstream ss;
            ss << "plane_" << num_planes << ".pcd";
            writer.write<pcl::PointXYZ> (ss.str(), *cloud_p);
        }
        extract.setNegative (true);
        extract.filter(*cloud_f);
        tmp.swap(cloud_f);
    }

    for(int idx = 0; idx < planes.size(); idx++) {
        pcl::PointCloud<pcl::PointXYZ> pc = planes[idx];
        int cam_cnt = 0;
        cv::Mat tmp_img = img.clone();
        // project planes onto cams
        // for(int i = 0; i < poses.size(); i++) {
            int point_cnt = 0;
            for (auto& point : pc.points) {
                if(point.z <= 0)
                    continue;
                int u = (int)(point.x*fx/point.z+cx);
                int v = (int)(point.y*fy/point.z+cy);
                if(u < 0 || v < 0 || u >= w || v >= h)
                    continue;
                cv::Point laserpoint;
                laserpoint.x = u;
                laserpoint.y = v;
                cv::circle(tmp_img, laserpoint, 4, cv::Scalar(0,255,0));
                point_cnt++;
            }
            if(point_cnt > 0.5*pc.points.size()) {
                stringstream name;
                name << idx+1 << "th plane_";
                cv::imshow(name.str(), tmp_img);
                cv::waitKey(0);
                cam_cnt++;
            }
            cout << "finish" << endl;
        // }
    }
    // combine
    // typedef pcl::PointXYZRGB PointT;
    // pcl::PointCloud<PointT> newpc;
    // newpc.resize(cld->width*cld->height);
    // int count = 0;
    // float maxX = 25.0, maxY = 6.0, minZ = -1.4;
    // for(auto& point : cld->points) {
    //     if(point.z <= 0)
    //         continue;
    //     // if(point.x > maxX || point.x < 0.0 || abs(point.y) > maxY || point.z < minZ)
    //     //     continue;
    //     int u = (int)(point.x*fx/point.z+cx);
    //     int v = (int)(point.y*fy/point.z+cy);
    //     if(u < 0 || v < 0 || u > w || v > h)
    //         continue;
    //     newpc.points[count].x = point.x;
    //     newpc.points[count].y = point.y;
    //     newpc.points[count].z = point.z;
    //     newpc.points[count].b = img.data[v*img.step+u*img.channels()];
    //     newpc.points[count].g = img.data[v*img.step+u*img.channels()+1];
    //     newpc.points[count].r = img.data[v*img.step+u*img.channels()+2];
    //     count++;
    // }
    // newpc.height = 1;
    // newpc.width = count;
    // newpc.resize(count);
    // newpc.is_dense = false;

    // pcl::io::savePCDFileBinary("newpc.pcd", newpc );

    return 0;
}
