/***************************************************************************
 *   Copyright (C) 2006 - 2016 by                                          *
 *      Tarek Taha, KURI  <tataha@tarektaha.com>                           *
 *      Randa Almadhoun   <randa.almadhoun@kustar.ac.ae>                   *
 *                                                                         *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Steet, Fifth Floor, Boston, MA  02111-1307, USA.          *
 ***************************************************************************/

#include "cscpp/get_params.h"
#include "sspp/pathplanner.h"
#include "ros/ros.h"
#include <ros/package.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/Pose.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseArray.h>
#include <cscpp/occlusion_culling.h>
#include "cscpp/coverage_path_planning_heuristic.h"
#include "sspp/rviz_drawing_tools.h"

using namespace SSPP;
 
int main( int argc, char **  argv)
{
    ros::init(argc, argv, "path_planning");
    ros::NodeHandle nh;
    ros::Time timer_start = ros::Time::now();

    // Pull in parameters from launch file
    paramReturn param{getParameters()}; //add/modify parameters in get_params.cpp/h


    // Define the PCD path, load PCD file, and put it into a PointCloud
    std::string modelName = std::string(ros::package::getPath("cscpp")+"/pcd/"+param.model_pcd+".pcd"); // get path
    pcl::PointCloud<pcl::PointXYZ>::Ptr originalCloudPtr(new pcl::PointCloud<pcl::PointXYZ>); // Creates a base point cloud called originalCloudPtr
    pcl::io::loadPCDFile<pcl::PointXYZ> (modelName, *originalCloudPtr); //Loads the pcd model into originalCloudPtr


    // Define Search Space grid size and its origin
    geometry_msgs::Vector3 gridSize;
    gridSize.x = param.gridsizeX;
    gridSize.y = param.gridsizeY;
    gridSize.z = param.gridsizeZ;
    geometry_msgs::Pose gridStartPose;
    gridStartPose.position.x = param.gridstartX;
    gridStartPose.position.y = param.gridstartY;
    gridStartPose.position.z = param.gridstartZ;


    // Define UAV size
    double robotH{0.6}; // TODO: maybe make these paramters?
    double robotW{0.75};
    double narrowestPath{0.987}; //was robotH=.9, robotW=0.5
    geometry_msgs::Point robotCenter;
    robotCenter.x = -0.3f;
    robotCenter.y = 0.0f;
    // TODO: check out this function. it has variables like mass, and moment of inertia hard coded
    auto *robot= new Robot("Robot",robotH,robotW,narrowestPath,robotCenter); //auto was Robot class


    // Define Sensors. Add multiple by uncommenting out sensor1
    Sensors sensor2(param.HorizFOV,param.VertFOV,param.FocalLength,param.NearPlaneDist,param.FarPlaneDist,param.PixWidth,param.PixHeight,Eigen::Vector3f(param.SensorPoseX,param.SensorPoseY,param.SensorPoseZ), Eigen::Vector3f(param.SensorRoll,param.SensorPitch,param.SensorYaw));  // Downward Looking Sensor
    std::vector<Sensors> sensors;
    sensors.push_back(sensor2); // Downward Looking Sensor (sensorPitch is positive)
    //Sensors sensor1(param.HorizFOV,param.VertFOV,param.FocalLength,param.NearPlaneDist,param.FarPlaneDist,param.PixWidth,param.PixHeight,Eigen::Vector3f(param.SensorPoseX,param.SensorPoseY,param.SensorPoseZ), Eigen::Vector3f(Sparam.ensorRoll,-param.SensorPitch,param.SensorYaw)); //Upward looking Sensor
    //sensors.push_back(sensor1); //Upward looking Sensor (negative sensorPitch)

    // Initialize path planner
    PathPlanner * pathPlanner;
    int progressDisplayFrequency{1}; // Every how many iterations to display the tree
    double regGridConRad{param.connection_radius}; // regular grid connection radius
    pathPlanner = new PathPlanner(nh,robot,regGridConRad,progressDisplayFrequency,sensors);
    pathPlanner->setDebugDelay(0.0); // causes the planner to pause for the desired amount of time and display the search tree, useful for debugging

    // Initiate and call the coveragePathPlanningHeuristic function
    bool gradualVisualization = true;
    std::string collisionCheckModelPath = ros::package::getPath("cscpp") + "/mesh/"+param.model_obj+".obj";
    CoveragePathPlanningHeuristic coveragePathPlanningHeuristic(nh,collisionCheckModelPath,modelName,param.Debug, gradualVisualization, param.HeuristicType, param.HorizFOV, param.VertFOV, param.NearPlaneDist, param.FarPlaneDist,param.voxelresolution);
    coveragePathPlanningHeuristic.setCoverageTarget(param.tgt_coverage);
    coveragePathPlanningHeuristic.setCoverageTolerance(param.CovTolerance);
    pathPlanner->setHeuristicFucntion(&coveragePathPlanningHeuristic);

    // TODO: clean up this section
    //generate regular grid, filter and connect
    // ( to perform uniform sampling uncomment 1 or 2, to perform dynamic sampling uncomment part 3)
    //****************************************
    // 1- Generate Grid Samples (loading them from files, the generation is done using filtering.cpp node) (filtering.cpp is under catkin_ws/src/aircraft_inspection/component_test/src)
    //std::string str1 = ros::package::getPath("cscpp")+"/txt/SearchSpaceUAV_1.5m_1to4_etihad_nowheels_nointernal_scaled_newdensed.txt";//robot
    //std::string str2 = ros::package::getPath("cscpp")+"/txt/SearchSpaceCam_1.5m_1to4_etihad_nowheels_nointernal_scaled_newdensed_0.txt";//sensor1
    //std::string str3 = ros::package::getPath("cscpp")+"/txt/SearchSpaceCam_1.5m_1to4_etihad_nowheels_nointernal_scaled_newdensed_1.txt";//sensor2
    //const char * filename1 = str1.c_str();
    //const char * filename2 = str2.c_str();
    //const char * filename3 = str3.c_str();
    //pathPlanner->loadRegularGrid(filename1,filename2,filename3);
    //pathPlanner->connectNodes();


    // 2- Generate Grid Samples (generate grid and filter it if you want using the cscpp itself)
//    bool sampleOrientations{true}; // TODO: Check out what these two bools actual do
//    bool samplesFiltering{true};
//    pathPlanner->generateRegularGrid(gridStartPose, gridSize,param.res_start,sampleOrientations,param.Orientation_Resolution,samplesFiltering);
//    pathPlanner->connectNodes();


    // 3- Generate Grid Samples (dynamically with different resolution generate grid connect clusters and re-do this until the disc. resolution is 0)
    pathPlanner->dynamicNodesGenerationAndConnection(gridStartPose,gridSize,param.res_start,param.res_decrement, param.min_dist, param.max_dist, param.Orientation_Resolution,regGridConRad);

    //std::string searchSpaceFileName = ros::package::getPath("cscpp")+"/txt/"+"generated_search_space_"+param.model_pcd+"_dsscpp.txt";
    //pathPlanner->saveSearchSpace(searchSpaceFileName.c_str());
    //pathPlanner->loadSearchSpace(searchSpaceFileName.c_str());

    std::cout<<"\nSpace Generation took: "<<double(ros::Time::now().toSec() - timer_start.toSec())<<" secs \n";

    // Visualize and check the number of connections and search space
    std::cout<<"\nGetting Connections"; fflush(stdout);
    std::vector<geometry_msgs::Point> searchSpaceConnections = pathPlanner->getConnections();
    std::cout<<"\nSearch Space Connections: "<<searchSpaceConnections.size() << '\n';
    std::vector<geometry_msgs::Point> searchSpaceNodes = pathPlanner->getSearchSpace();
    std::cout<<"\n---->>> Total Nodes in search Space = "<<searchSpaceNodes.size()<<'\n';
    
    std::vector<geometry_msgs::PoseArray> sensorsPoseSS;
    geometry_msgs::PoseArray robotPoseSS;
    std::cout<<"\nGetting Robot Poses";fflush(stdout);
    pathPlanner->getRobotSensorPoses(robotPoseSS,sensorsPoseSS);
    std::cout<<"\nSetting Min and Max Sensor Accuracy";fflush(stdout);
    coveragePathPlanningHeuristic.setMaxMinSensorAccuracy(sensorsPoseSS); //TODO: fix magic number 0.0000285 in path planner line 193, occlusion_culling, etc


    // Save Search space to a file search space
    ofstream ssRobotFile,ssRobotFile1;
    std::string SSfile_loc1 = ros::package::getPath("cscpp")+"/txt/ss_robot/"+param.model_pcd+"_search_space.txt";
    ssRobotFile.open (SSfile_loc1.c_str());
    // std::string SSfile_loc2 = ros::package::getPath("cscpp")+"/txt/ss_lkh/"+param.model_pcd+"_search_space_numbered_points.txt"; // uncomment if you want the nodes numbered for some reason
    // ssRobotFile1.open (SSfile_loc2.c_str());

    // for(int j = 0; j<robotPoseSS.poses.size(); j++) //Clang-tidy replaced this and robotPoseSS.poses[j].orientation.x
    for(auto & pose : robotPoseSS.poses) //Loops through all search space nodes and prints to the file path above
    {
        tf::Quaternion qt(pose.orientation.x, pose.orientation.y,pose.orientation.z,pose.orientation.w);
        double yaw_deg{DEG2RAD(tf::getYaw(qt))}; //transforms quartenion to Euler yaw angle. Assumes no pitch/roll
        ssRobotFile << pose.position.x<<" "<<pose.position.y<<" "<<pose.position.z<<" "<<yaw_deg<<"\n";
        // ssRobotFile1 << j+1 <<" " <<robotPoseSS.poses[j].position.x<<" "<<robotPoseSS.poses[j].position.y<<" "<<robotPoseSS.poses[j].position.z<<"\n";
    }


    //find the path and print it
    std::cout<<"\nStarting Search";fflush(stdout);
    ros::Time timer_restart = ros::Time::now();
    Pose start(param.UAV_start_X, param.UAV_start_Y, param.UAV_start_Z,DEG2RAD(0.0)); //Desired UAV starting location
    Pose end(param.UAV_end_X,param.UAV_end_Y,param.UAV_end_Z,DEG2RAD(0.0)); // Desired UAV ending location. TODO: Don't think this is used. Delete this and the parameters?
    Node * path = pathPlanner->startSearch(start, param.continuous, param.Debug); // Calls Astar path planner. Dig into this to fix how nodes are added
    ros::Time timer_end = ros::Time::now();
    std::cout<<"\nPath Finding took:"<<double(timer_end.toSec() - timer_restart.toSec())<<" secs";


    if(path) // Check if path exists
    {
       //pathPlanner->printNodeList();  // Uncomment if you want to print all the nodes of the generated path (not recommended if path is long)
    }
    else
    {
        std::cout<<"\nNo Path Found";
    }


// **** Save path to a text file *****
    std::stringstream ss,cc,aa,dd,mm; // Put parameters into string form
    ss << param.tgt_coverage;
    cc << regGridConRad;
    aa << DEG2RAD(param.SensorPitch);
    dd << param.max_dist;
    mm << param.min_dist;
    std::string path_file_loc = ros::package::getPath("cscpp")+"/txt/"+param.model_pcd+"_"+ss.str()+"%_"+mm.str()+"to"+dd.str()+"m_"+cc.str()+"GridConRad_"+aa.str()+"Deg.txt"; // Path location/name

    // Define variables needed for saving path to file
    geometry_msgs::Point linePoint; // used to build the pathsegments
    std::vector<geometry_msgs::Point> pathSegments; // how path is visualized in RViz
    geometry_msgs::PoseArray robotPose; // used to visualize UAV position/orientation
    geometry_msgs::PoseArray sensorPose; // used to visualize sensor position/orientation
    pcl::PointCloud<pcl::PointXYZ> temp_cloud; // visible point cloud for individual sensor poses
    pcl::PointCloud<pcl::PointXYZ> combined; // sums the visible points for all sensor poses. Used to visualize Covered Point Cloud
    octomap::OcTree* oct;
    std::vector<double> accuracyPerViewpointAvg;
    OcclusionCulling occlusionCulling(nh,modelName, param.HorizFOV, param.VertFOV, param.NearPlaneDist, param.FarPlaneDist,param.voxelresolution); // Initializes OcclusionCulling and defines what part of PCD can be seen by camera
    double accuracySum{0};
    double dist{0};

    // Iterate through the path and save the data to the file
    ofstream pathFile;
    pathFile.open (path_file_loc.c_str());
    while(path != nullptr)
    {
        // Write current position/orientation
        tf::Quaternion qt(path->pose.p.orientation.x,path->pose.p.orientation.y,path->pose.p.orientation.z,path->pose.p.orientation.w);
        double yaw = DEG2RAD(tf::getYaw(qt)); // degrees
        pathFile << param.wait_time <<" " << path->pose.p.position.x<<" "<<path->pose.p.position.y<<" "<<path->pose.p.position.z<<" "<<yaw<<"\n"; // Added wait_time so Gazebo can read the waypoints (using dronekit)

        pcl::PointCloud<pcl::PointXYZ> temp;
        if (path->next != nullptr) // Gets visualizaton data
        {
            linePoint.x = path->pose.p.position.x;
            linePoint.y = path->pose.p.position.y;
            linePoint.z = path->pose.p.position.z;
            pathSegments.push_back(linePoint);
            robotPose.poses.push_back(path->pose.p);

            for(int i =0; i<path->senPoses.size();i++)
            {
                sensorPose.poses.push_back(path->senPoses[i].p);
                temp_cloud=occlusionCulling.extractVisibleSurface(path->senPoses[i].p); // calc what sensor sees at this pose
                combined += temp_cloud; // add to total visible points
            }


            linePoint.x = path->next->pose.p.position.x;
            linePoint.y = path->next->pose.p.position.y;
            linePoint.z = path->next->pose.p.position.z;
            pathSegments.push_back(linePoint);
            robotPose.poses.push_back(path->next->pose.p);

            for(int i =0; i<path->next->senPoses.size();i++)
            {
                sensorPose.poses.push_back(path->next->senPoses[i].p);
                temp_cloud=occlusionCulling.extractVisibleSurface(path->next->senPoses[i].p);
                combined += temp_cloud;
                temp += temp_cloud;

                if(temp.points.size()!=0) // TODO: Don't really understand this bit
                {
                    double avgAcc = occlusionCulling.calcAvgAccuracy(temp_cloud,path->next->senPoses[i].p);
                    double a = (occlusionCulling.maxAccuracyError - avgAcc)/occlusionCulling.maxAccuracyError;
                    accuracyPerViewpointAvg.push_back(a);
                    accuracySum += avgAcc;
                    //std::cout<<"accuracy per viewpoints: "<<a<<" "<<avgAcc<<std::endl;
                }
            }
            dist= dist+ Dist(path->next->pose.p,path->pose.p); // Path Length
        }
        else
        {
            if(coveragePathPlanningHeuristic.getHeuristicType() == InfoGainVolumetricH)
                oct = new octomap::OcTree(*path->octree);
        }
        path = path->next;
    }
    pathFile.close();


    // Define point cloud covered by path
    pcl::PointCloud<pcl::PointXYZ>::Ptr coveredCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    coveredCloudPtr->points=combined.points;


    std::cout<<"\nDistance calculated from the path: "<<dist<<"m\n";
    float coveredCloudPercentage{occlusionCulling.calcCoveragePercent(coveredCloudPtr)};
    std::cout<<"Covered Cloud % : "<< coveredCloudPercentage <<"%\n";
    std::cout<<"Average Accuracy per viewpoint is "<<accuracySum/accuracyPerViewpointAvg.size()<<"m\n";


    ///// **************** Plot Text File In RVIZ ***********
    // double wait, X, Y, Z, YAW;
    // geometry_msgs::Point linePoint2;
    // std::vector<geometry_msgs::Point> pathSegments2;
    // geometry_msgs::PoseArray sensorPose2;
    // geometry_msgs::Pose sensPoses2; 
    // ifstream file; 
    // file.open("/home/antuser/catkin_ws/src/asscpp/cscpp/F15WaypointFile.txt");
    /// ***File needs to be modified to repeat all but the last waypoint twice ***

    // if(file.is_open())
    // {
    //     while(file >> wait >> X >> Y >> Z >> YAW)
    //     {
    //         std::cout<< wait <<" " << X <<" " << Y <<" " << Z << " "<< YAW << '\n'; 
        
    //         linePoint2.x = X;
    //         linePoint2.y = Y;
    //         linePoint2.z = Z;
    //         pathSegments2.push_back(linePoint2);
    //         double yaw2, pitch2, roll2; 
    //         pitch2 = 0.523599; // 30 deg
    //         roll2 = 0; 
    //         yaw2 = YAW; // Not sure why you don't convert to radians? 

    //         double cy = cos(yaw2 * 0.5);
    //         double sy = sin(yaw2 * 0.5);
    //         double cp = cos(pitch2 * 0.5);
    //         double sp = sin(pitch2 * 0.5);
    //         double cr = cos(roll2 * 0.5);
    //         double sr = sin(roll2 * 0.5);

    //         double w, x, y, z;
    //         w = cr * cp * cy + sr * sp * sy;
    //         x = sr * cp * cy - cr * sp * sy;
    //         y = cr * sp * cy + sr * cp * sy;
    //         z = cr * cp * sy - sr * sp * cy;

    //         sensPoses2.position.x = X; 
    //         sensPoses2.position.y = Y; 
    //         sensPoses2.position.z = Z; 
    //         sensPoses2.orientation.x = x; 
    //         sensPoses2.orientation.y = y;
    //         sensPoses2.orientation.z = z;
    //         sensPoses2.orientation.w = w;
    //         sensorPose2.poses.push_back(sensPoses2);
    //     }
    //     file.close(); 
    // }    
    // else
    //     std::cout<<"file is not open"<<'\n';
    // visualization_msgs::Marker pathMarker2 = drawLines(pathSegments2,2000,1,10000000,0.1); //20000
    // ***End Plot Text File ***


    //  Define the types topics that will be visible in RViz.
    ros::Publisher originalCloudPub  = nh.advertise<sensor_msgs::PointCloud2>("original_point_cloud", 100);
    ros::Publisher visiblePub        = nh.advertise<sensor_msgs::PointCloud2>("occlusion_free_cloud", 100);
    ros::Publisher pathPub           = nh.advertise<visualization_msgs::Marker>("generated_path", 10);
    ros::Publisher searchSpacePub    = nh.advertise<visualization_msgs::Marker>("search_space", 10);
    ros::Publisher connectionsPub    = nh.advertise<visualization_msgs::Marker>("connections", 10);
    ros::Publisher robotPosePub      = nh.advertise<geometry_msgs::PoseArray>("robot_pose", 10);
    ros::Publisher sensorPosePub     = nh.advertise<geometry_msgs::PoseArray>("sensor_pose", 10);
    ros::Publisher robotPoseSSPub    = nh.advertise<geometry_msgs::PoseArray>("SS_robot_pose", 10);
    ros::Publisher octomapPub        = nh.advertise<octomap_msgs::Octomap>("octomap", 1);

    std::vector<ros::Publisher> sensorsPoseSSPub;
    for(int i = 0; i<sensorsPoseSS.size(); i++) // Creates publishers for how ever many sensors are defined
    {
        ros::Publisher  sensorPoseSSPub   = nh.advertise<geometry_msgs::PoseArray>("SS_sensor_pose_"+std::to_string(i), 10);
        sensorsPoseSSPub.push_back(sensorPoseSSPub);
    }
    // *** For Plotting Txt files in RVIZ
    // ros::Publisher pathPub2          = nh.advertise<visualization_msgs::Marker>("generated_path2", 10);
    // ros::Publisher sensorPosePub2    = nh.advertise<geometry_msgs::PoseArray>("sensor_pose2", 10);

    // *** Create markers for visualization messages TODO: fix floating numbers
    visualization_msgs::Marker connectionsMarker = drawLines(searchSpaceConnections,10000,3,100000000,0.03);
    visualization_msgs::Marker pathMarker = drawLines(pathSegments,2000,1,10000000,0.1); //20000
    visualization_msgs::Marker searchSpaceMarker = drawPoints(searchSpaceNodes,2,1000000);

//  *** Publish ROS Messages for visualization ***
    ros::Rate loopRate(10);
    while (ros::ok()) //Publish messages as long as roscore is running
    {
        if( coveragePathPlanningHeuristic.getHeuristicType() == InfoGainVolumetricH)
        {
            octomap_msgs::Octomap octomap;
            octomap.binary = 1;
            octomap.id = 1;
            octomap.resolution =0.25;
            octomap.header.frame_id = "map";
            octomap.header.stamp = ros::Time::now();
            bool res = octomap_msgs::fullMapToMsg(*oct, octomap);
            if(res)
            {
                octomapPub.publish(octomap);
            }
            else
            {
                ROS_WARN("OCT Map serialization failed!");
            }
        }

        sensor_msgs::PointCloud2 cloud1;
        pcl::toROSMsg(*originalCloudPtr, cloud1); //cloud of original (white) using original cloud
        cloud1.header.stamp = ros::Time::now();
        cloud1.header.frame_id = "map"; //change according to the global frame please!!
        originalCloudPub.publish(cloud1);

        sensor_msgs::PointCloud2 cloud2;
        pcl::toROSMsg(*coveredCloudPtr, cloud2); //cloud of original (white) using original cloud
        cloud2.header.stamp = ros::Time::now();
        cloud2.header.frame_id = "map"; //change according to the global frame please!!
        visiblePub.publish(cloud2);

        searchSpacePub.publish(searchSpaceMarker);
        connectionsPub.publish(connectionsMarker);
        pathPub.publish(pathMarker);

        robotPose.header.frame_id= "map";
        robotPose.header.stamp = ros::Time::now();
        robotPosePub.publish(robotPose);

        sensorPose.header.frame_id= "map";
        sensorPose.header.stamp = ros::Time::now();
        sensorPosePub.publish(sensorPose);

        robotPoseSS.header.frame_id= "map";
        robotPoseSS.header.stamp = ros::Time::now();
        robotPoseSSPub.publish(robotPoseSS);

        for(int i = 0 ; i<sensorsPoseSS.size(); i++) // publishes however many sensors are defined
        {
            sensorsPoseSS[i].header.frame_id= "map";
            sensorsPoseSS[i].header.stamp = ros::Time::now();
            sensorsPoseSSPub[i].publish(sensorsPoseSS[i]);
        }

        // ****Plot Txt File in RVIZ ****
        // pathMarker2.header.frame_id = "map";
        // pathMarker2.header.stamp = ros::Time::now();
        // pathPub2.publish(pathMarker2);
        // sensorPose2.header.frame_id= "map";
        // sensorPose2.header.stamp = ros::Time::now();
        // sensorPosePub2.publish(sensorPose2);  //*******

        ros::spinOnce();
        loopRate.sleep();
    }

    delete robot;
    delete pathPlanner;
    return 0;
}
