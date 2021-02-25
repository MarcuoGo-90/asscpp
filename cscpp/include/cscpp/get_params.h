//
// Created by seg on 2/10/21.
//

#ifndef CSCPP_GET_PARAMS_H
#define CSCPP_GET_PARAMS_H

#include <string>

// defines the return of getParameters(). Allows you to return integers/doubles/strings with a single function
struct paramReturn
{
//  *** General Parameters ****
    std::string model_pcd;
    std::string model_obj;
    double tgt_coverage;
    double CovTolerance;
    double wait_time;
    int HeuristicType;
    bool Debug;
    bool continuous;
    double voxelresolution;

//  **** UAV Parameters ****
    double UAV_start_X;
    double UAV_start_Y;
    double UAV_start_Z;
    double UAV_end_X;
    double UAV_end_Y;
    double UAV_end_Z;

//  *** Search Space Paramters ****
    double min_dist;
    double max_dist;
    double Orientation_Resolution;
    double res_start;
    double res_decrement;
    double connection_radius;
    double gridstartX;
    double gridstartY;
    double gridstartZ;
    double gridsizeX;
    double gridsizeY;
    double gridsizeZ;

//  ***** Sensor Params ****
    int HorizFOV;
    int VertFOV;
    double FocalLength;
    double NearPlaneDist;
    double FarPlaneDist;
    int PixWidth;
    int PixHeight;
    double SensorPoseX;
    double SensorPoseY;
    double SensorPoseZ;
    double SensorRoll;
    double SensorPitch;
    double SensorYaw;
};

paramReturn getParameters();

#endif //CSCPP_GET_PARAMS_H
