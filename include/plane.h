#ifndef __PLANE_H__
#define __PLANE_H__

#include <opencv2/core/core.hpp>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
struct PLANE
{
    double param[4];
    pcl::PointCloud<pcl::PointXYZ> cld;
    
    PLANE (pcl::ModelCoefficients coeff, pcl::PointCloud<pcl::PointXYZ>& points) {
        for(int i = 0; i < 4; i++)
            param[i] = coeff.values[i];
        cld = points;
    }
};


typedef struct PLANE Plane;

#endif