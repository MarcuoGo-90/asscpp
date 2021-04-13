//
// Created by seg on 4/12/21.
//

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "cscpp/calc_pointing_angle.h"
//#include <pcl/io/pcd_io.h>
//#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/point_cloud.h>
//#include <pcl/filters/voxel_grid.h>



double getviewpoint(pcl::PointCloud<pcl::PointXYZ> point_cloud)
{

    /// Steps:
    // import point cloud
    // import viewpoint
    // import max dist
    // filter out points further than max dist away
    // calculate closest point to viewpoint
    // specify an area around that point to consider for inspection
    // find a way to get the unit normal to the surface
    // reverse that vector to get the pointing direction


    return 2.0;
}