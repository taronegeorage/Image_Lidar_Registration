#include<iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <vector>
#include "frame.h"



void show_traj(pcl::PointCloud<pcl::PointXYZ>& cld, std::vector<Frame>& poses) {
    typedef pcl::PointXYZRGB PointT;
    pcl::PointCloud<PointT> traj;
    int traj_count = 0;
    traj.height = 1;
    traj.width = poses.size();
    traj.resize(traj.width+cld.points.size());
    traj.is_dense = false;

    for(int i = 0; i < poses.size(); i++) {
        Eigen::Vector3d p = poses[i].trans;
        traj.points[i].x = p(0);
        traj.points[i].y = p(1);
        traj.points[i].z = p(2);
        traj.points[i].r = traj.points[i].g = traj.points[i].b = 255;
    }
    traj_count = poses.size();
    for(auto& point : cld.points){
        traj.points[traj_count].x = point.x;
        traj.points[traj_count].y = point.y;
        traj.points[traj_count].z = point.z;
        traj.points[traj_count].r = traj.points[traj_count].g = traj.points[traj_count].b = 100;
        traj_count++;
    }
    pcl::io::savePCDFileBinary("traj.pcd", traj );
}