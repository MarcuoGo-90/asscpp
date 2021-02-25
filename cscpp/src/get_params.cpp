//
// Created by seg on 2/10/21.
//

#include "cscpp/get_params.h"
#include "ros/ros.h"


////Get the simulation parameters from the ROS params in the launch file
paramReturn getParameters()
{
    ros::NodeHandle nh_private("~"); // necessary to get the parameters from the launch file
    paramReturn param;

//    nh_private.param("PARAM_NAME", param.PARAM_NAME, DEFAULT VALUE);
//      1st value: name of param in .launch file
//      2nd value: param.<corresponding name in paramReturn structure>
//      3rd value: default value (currently set for a F35)

// *** TO ADD A NEW PARAMETER, ADD IT TO THE paramReturn STRUCT IN ROSpublisher.h, THEN ADD IT HERE ***

//  *** General Parameters ****
    nh_private.param("Model_PCD", param.model_pcd,std::string{"F35"});
    nh_private.param("Model_OBJ", param.model_obj, std::string{"F35"});
    nh_private.param("Target_Coverage", param.tgt_coverage, 60.0);
    nh_private.param("CovTolerance", param.CovTolerance, 1.0);
    nh_private.param("wait_time", param.wait_time, 1.0);
    nh_private.param("HeuristicType", param.HeuristicType, 5);
    nh_private.param("Debug", param.Debug, false);
    nh_private.param("continuous", param.continuous, false);
    nh_private.param("voxelresolution", param.voxelresolution, 0.5);

//    *** UAV Parameters ****
    nh_private.param("UAV_start_X", param.UAV_start_X, -2.0);
    nh_private.param("UAV_start_Y", param.UAV_start_Y, 0.0);
    nh_private.param("UAV_start_Z", param.UAV_start_Z,3.0);
    nh_private.param("UAV_end_X", param.UAV_end_X, -2.0);
    nh_private.param("UAV_end_Y", param.UAV_end_Y, 0.0);
    nh_private.param("UAV_end_Z", param.UAV_end_Z, 3.0);

//  *** Search Space Parameters ****
    nh_private.param("min_dist", param.min_dist, 1.0);
    nh_private.param("max_dist", param.max_dist, 3.0);
    nh_private.param("Orientation_Resolution", param.Orientation_Resolution,45.0);
    nh_private.param("res_start", param.res_start, 1.5);
    nh_private.param("res_decrement", param.res_decrement, 0.5);
    nh_private.param("connection_radius", param.connection_radius, 2.43);
    nh_private.param("gridstartX", param.gridstartX, -20.0);
    nh_private.param("gridstartY", param.gridstartY, -10.0);
    nh_private.param("gridstartZ", param.gridstartZ, 10.0);
    nh_private.param("gridsizeX", param.gridsizeX, 40.0);
    nh_private.param("gridsizeY", param.gridsizeY, 20.0);
    nh_private.param("gridsizeZ", param.gridsizeZ, 20.0);

//  **** Sensor Parameters ****
    nh_private.param("HorizFOV", param.HorizFOV, 57);
    nh_private.param("VertFOV", param.VertFOV,44);
    nh_private.param("FocalLength", param.FocalLength,0.08);
    nh_private.param("NearPlaneDist", param.NearPlaneDist, 0.7);
    nh_private.param("FarPlaneDist", param.FarPlaneDist, 6.0);
    nh_private.param("PixWidth", param.PixWidth,1600);
    nh_private.param("PixHeight", param.PixHeight,1200);
    nh_private.param("SensorPoseX", param.SensorPoseX,0.122555);
    nh_private.param("SensorPoseY", param.SensorPoseY,0.0);
    nh_private.param("SensorPoseZ", param.SensorPoseZ,-0.0762);
    nh_private.param("SensorRoll", param.SensorRoll,0.0);
    nh_private.param("SensorPitch", param.SensorPitch,0.523599);
    nh_private.param("SensorYaw", param.SensorYaw,0.0);
    return param;
}