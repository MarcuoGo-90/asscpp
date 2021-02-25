// #include "sspp/pathplanner.h"
// #include "ros/ros.h"
// #include <ros/package.h>
// #include <tf/tf.h>
// #include <tf_conversions/tf_eigen.h>
// #include <geometry_msgs/Pose.h>
// #include <eigen_conversions/eigen_msg.h>

// #include <geometry_msgs/Pose.h>
// #include <geometry_msgs/PoseArray.h>

// #include <cscpp/occlusion_culling.h>

// #include "cscpp/coverage_path_planning_heuristic.h"
// #include "sspp/rviz_drawing_tools.h"
// #include "rviz_visual_tools/rviz_visual_tools.h"
// #include <iostream>
// #include <fstream> 
// #include <string>
// using namespace std;


// int main()
// {
//     // ros::init(argc, argv, "path_planning2");
//     // ros::NodeHandle nh;
//     // ros::Publisher pathPub           = nh.advertise<visualization_msgs::Marker>("generated_path2", 10);

    
//     double wait, X, Y, Z, YAW;

//     geometry_msgs::Point linePoint;
//     std::vector<geometry_msgs::Point> pathSegments;

//     ifstream file; 
//     file.open("/home/antuser/catkin_ws/src/asscpp/cscpp/F15WaypointFile.txt");
//     if(file.is_open())
//     {
//         while(file >> wait >> X >> Y >> Z >> YAW)
//         {
//             std::cout<< wait <<" " << X <<" " << Y <<" " << Z << " "<< YAW << '\n'; 
        
//             linePoint.x = X;
//             linePoint.y = Y;
//             linePoint.z = Z;
//             pathSegments.push_back(linePoint);
//         }
//         // std::cout<<"TOOOTALY OPEN"<<'\n';
//         file.close(); 
//     }    
//     else
//         std::cout<<"file is not open"<<'\n';
//     return 0;


//     // rviz_visual_tools::RvizVisualToolsPtr visualTools;

//     // visualTools->publishPath(pathSegments, rviz_visual_tools::RED, rviz_visual_tools::LARGE,"generated_path2");
//     // visualization_msgs::Marker pathMarker = drawLines(pathSegments,2000,1,10000000,0.1); //20000
//     // pathPub.publish(pathMarker);

// }
