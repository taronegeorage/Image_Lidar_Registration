
#include<iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <Eigen/Core>

using namespace std;

int main (int argc, char** argv) {
    typedef pcl::PointXYZRGB PointT;

    Eigen::Matrix4d Tcl;
    Tcl << -1.44694624e-02,-9.99894963e-01,8.34671705e-04,-3.00805142e-03, 
            5.13704445e-03,-9.09086106e-04,-9.99986392e-01,-1.06576990e-01,
            9.99882116e-01,-1.44649777e-02,5.14965886e-03,-2.40858246e-01,
            0.00000000e+00,0.00000000e+00,0.00000000e+00,1.00000000e+00;
    Eigen::Matrix4f T_lidar_loam;
    T_lidar_loam << 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cld(new pcl::PointCloud<pcl::PointXYZ>);
	if(pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cld) == -1) {
		PCL_ERROR("CANNOT READ FILE");
		return -1;
	}
	cout << "Loaded " << cld->width * cld->height << " data points from " << argv[1] << endl;
    // pcl::transformPointCloud(*cld, *cld, T_lidar_loam);
    
    pcl::PointCloud<PointT> data_with_init_cam;
    int count = 0;
    data_with_init_cam.height = 1;
    data_with_init_cam.width = cld->width + 60;
    data_with_init_cam.resize(data_with_init_cam.width * data_with_init_cam.height);
    data_with_init_cam.is_dense = false;
    for(auto& point : cld->points) {
        data_with_init_cam.points[count].x = point.x;
        data_with_init_cam.points[count].y = point.y;
        data_with_init_cam.points[count].z = point.z;
        data_with_init_cam.points[count].r = 255;
        data_with_init_cam.points[count].g = 255;
        data_with_init_cam.points[count].b = 255;
        count++;
    }
    cout << "draw the init cam frame" << endl;
    for(int i = 0; i < 20; i++) {
        PointT p1;
        p1.x = i - 10; p1.y = 5; p1.z = 0;
        p1.r = 255; p1.b = 0; p1.g = 0;
        data_with_init_cam.points[count+2*i] = p1;
        p1.y = -5;
        data_with_init_cam.points[count+2*i+1] = p1;
    }
    for(int i = 0; i < 10; i++) {
        PointT p1;
        p1.x = -10; p1.y = i - 5; p1.z = 0;
        p1.r = 255; p1.b = 0; p1.g = 0;
        data_with_init_cam.points[count+40+2*i] = p1;
        p1.x = 10;
        data_with_init_cam.points[count+40+2*i+1] = p1;
    }
    pcl::io::savePCDFileBinary("full.pcd", data_with_init_cam );

    pcl::PointCloud<PointT> select_x_pos;
    count = 0;
    select_x_pos.height = 1;
    select_x_pos.width = data_with_init_cam.width;
    select_x_pos.resize(data_with_init_cam.width * data_with_init_cam.height);
    select_x_pos.is_dense = false;
	for(auto& point : data_with_init_cam.points){
		if(point.x <= 0)
			continue;
        select_x_pos.points[count] = point;
        count++;
    }
    select_x_pos.resize(count);
    cout << "get point clouds x>0 with " << count << " points" << endl;
    pcl::io::savePCDFileBinary("select_x_pos.pcd", select_x_pos );

    pcl::PointCloud<PointT> select_y_pos;
    count = 0;
    select_y_pos.height = 1;
    select_y_pos.width = data_with_init_cam.width;
    select_y_pos.resize(data_with_init_cam.width * data_with_init_cam.height);
    select_y_pos.is_dense = false;
	for(auto& point : data_with_init_cam.points){
		if(point.y <= 0)
			continue;
        select_y_pos.points[count] = point;
        count++;
    }
    cout << "get point clouds y>0 with " << count << " points" << endl;
    pcl::io::savePCDFileBinary("select_y_pos.pcd", select_y_pos );

    pcl::PointCloud<PointT> select_z_pos;
    count = 0;
    select_z_pos.height = 1;
    select_z_pos.width = data_with_init_cam.width;
    select_z_pos.resize(data_with_init_cam.width * data_with_init_cam.height);
    select_z_pos.is_dense = false;
	for(auto& point : data_with_init_cam.points){
		if(point.z <= 0)
			continue;
        select_z_pos.points[count] = point;
        count++;
    }
    cout << "get point clouds z>0 with " << count << " points" << endl;
    pcl::io::savePCDFileBinary("select_z_pos.pcd", select_z_pos );
}