//
// Created by antuser on 9/17/20.
//
//
// Created by antuser on 9/17/20.
//
#include <iostream>
#include <fstream>
#include <string>
#include "ros/ros.h"
#include <ros/package.h>

using namespace std;

int main() {

    ros::NodeHandle nh_private("~"); // necessary to get the parameters from the launch file

    //Get the simulation parameters positions from the ROS params in the launch file
    string model_pcd;
    nh_private.param("Model_PCD", model_pcd, model_pcd);
    double tgt_coverage;
    nh_private.param("Target_Coverage", tgt_coverage, tgt_coverage);
    double connection_radius;
    nh_private.param("connection_radius", connection_radius, connection_radius);

    std::stringstream ss,cc;
    ss << targetCov;
    cc <<connection_radius;
    std::string old_file = ros::package::getPath("cscpp")+"/txt/"+model_pcd+"_"+ss.str()+"%_"+cc.str()+"GridConRad.txt";
    std::string new_file = ros::package::getPath("cscpp")+"/txt/Formatted"+model_pcd+"_"+ss.str()+"%_"+cc.str()+"GridConRad.txt";
    ofstream outfile;
    outfile.open(new_file);

    ifstream inFile;
    inFile.open(old_file); // add the directory
    // Check for Error
    if (inFile.fail()) {
        cerr << "Error Opening File" << endl;
        exit(1);
    }
    double pi;
    pi = 3.14159265359;
    double X;
    double Y;
    double Z;
    double Yaw;
    double waitTime;
    waitTime = 3.0;
    while (inFile >> X >> Y >> Z >> Yaw) {
        Yaw = Yaw *180/pi;
        cout << X<< " " << Y << " " << Z << " " << Yaw << endl;
        outfile << waitTime << " " << X <<" "<< Y <<" " << Z <<" " << Yaw << endl;

    }

    inFile.close();
    outfile.close();
    return 0;
}
