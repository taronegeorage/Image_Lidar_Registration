#include<iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

using namespace std;

int main(int argc, char* argv[])
{

    Eigen::Matrix4f Tcl;
    // Tcl << -1.44694624e-02,-9.99894963e-01,8.34671705e-04,-3.00805142e-03,
    //         		5.13704445e-03,-9.09086106e-04,-9.99986392e-01,-1.06576990e-01,
    //         		9.99882116e-01,-1.44649777e-02,5.14965886e-03,-2.40858246e-01,
    //         		0.00000000e+00,0.00000000e+00,0.00000000e+00,1.00000000e+00;
    Tcl << 9.99967175e-01,6.05118632e-03,-5.38809830e-03,-6.76300340e-04,
            -5.43088201e-03,7.07626705e-03,-9.99960215e-01,-1.50224504e-01,
            -6.01281795e-03,9.99956654e-01,7.10889805e-03,-2.43711597e-01,
            0.00000000e+00,0.00000000e+00,0.00000000e+00,1.00000000e+00;
    Eigen::Matrix4f T_lidar_loam;
    T_lidar_loam << 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1;
    Eigen::Matrix4f Transform = Tcl * T_lidar_loam;

	typedef pcl::PointXYZ PointT;

	pcl::PointCloud<PointT>::Ptr map(new pcl::PointCloud<PointT>);
	if(pcl::io::loadPCDFile<PointT> (argv[1], *map) == -1) {
		PCL_ERROR("CANNOT READ FILE");
		return -1;
	}
	cout << "read maps with " << map->width * map->height 
		<< " data points" << endl;
    pcl::transformPointCloud(*map, *map, Transform);

    pcl::PointCloud<PointT>::Ptr plane(new pcl::PointCloud<PointT>);
	if(pcl::io::loadPCDFile<PointT> (argv[2], *plane) == -1) {
		PCL_ERROR("CANNOT READ FILE");
		return -1;
	}
	cout << "read planes with " << plane->width * plane->height 
		<< " data points" << endl;

    pcl::PointCloud<pcl::PointXYZRGB> all;
    int count = 0;
    all.height = 1;
    all.width = map->width + plane->width;
    all.resize(all.width * all.height);
    all.is_dense = false;

	for(auto& point : map->points){
        all.points[count].x = point.x;
        all.points[count].y = point.y;
        all.points[count].z = point.z;
        all.points[count].r = 255;
        all.points[count].g = 255;
        all.points[count].b = 255;
        count++;
    }
    for(auto& point : plane->points){
        all.points[count].x = point.x;
        all.points[count].y = point.y;
        all.points[count].z = point.z;
        all.points[count].r = 255;
        all.points[count].g = 0;
        all.points[count].b = 0;
        count++;
    }
    pcl::io::savePCDFileBinary("show.pcd", all );

}