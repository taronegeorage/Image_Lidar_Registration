#ifndef __POINTS_H__
#define __POINTS_H__

#include <opencv2/core/core.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

struct LidarPointStruct
{
    bool visible;
    double x, y, z;
    LidarPointStruct(bool flag = false) {
        
        visible = flag;
    }
};


struct PixelPointStruct
{
    bool visible;
    int u, v;
    cv::Point lasesrpoint;
    PixelPointStruct(int x, int y, bool flag = false) {
        u = x; v = y;
        visible = flag;
    }
    PixelPointStruct(){
        visible = false;
    }
};

typedef struct LidarPointStruct LidarPoint;
typedef struct PixelPointStruct PixelPoint;

#endif