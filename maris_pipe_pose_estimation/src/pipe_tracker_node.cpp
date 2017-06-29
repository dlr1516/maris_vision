/**
 * MARIS Vision - ROS Packages of Vision System used in MARIS project. 
 * Copyright (C) 2016 Fabjan Kallasi and Dario Lodi Rizzini.
 *
 * This file is part of MARIS Vision. 
 * Visit http://rimlab.ce.unipr.it/Maris.php for more information about 
 * the project. 
 *
 * MARIS Vision is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * MARIS Vision is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with MARIS Vision.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <ros/ros.h>
#include <ros/time.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>

#include <iostream>
#include <string>

#include <maris_ros_msgs/TransfMatrix.h>
#include <maris_ros_msgs/VehiclePosition.h>

#include <maris_pipe_pose_estimation/PipePoseTracker.h>


ros::Subscriber cToInSub;
ros::Publisher cToOutPub;
ros::Subscriber altitudeSub;
PipePoseTracker tracker;

// Callback to read depth from system vehicle
void altitudeCallback(const std_msgs::Float32& altitude);

// Callback to read input transformation
void transformationCallback(const maris_ros_msgs::TransfMatrix& transf);

// Conversion functions
void eigenToMarisPTN(const Eigen::Matrix4d& eigenTransf,maris_ros_msgs::TransfMatrix& marisTransf);

void marisToEigenPTN(const maris_ros_msgs::TransfMatrix& marisTransf,Eigen::Matrix4d& eigenTransf);

// MAIN
int main(int argc,char** argv)
{
  ros::init(argc, argv, "PipeTrackerNode");
  ros::NodeHandle nh("~");

  std::string cToInTopic, cToOutTopic, altitudeTopic;
  double distToll;
  bool enableAltitude;
  double altitudeToll;
  bool enableAngle;
  double angleTollDeg;
  nh.param<std::string>("cToInTopic", cToInTopic, "/vision/cTo");
  nh.param<std::string>("cToOutTopic", cToOutTopic, "/vision/cToCorrected");
  nh.param<std::string>("altitudeTopic", altitudeTopic, "/vehicle/altitude");
  nh.param<double>("distToll", distToll, 1.0);
  nh.param<bool>("enableAltitude", enableAltitude, false);
  nh.param<double>("altitudeToll", altitudeToll, 1.0);
  nh.param<bool>("enableAngle", enableAngle, false);
  nh.param<double>("angleTollDeg", angleTollDeg, 30.0);

  tracker.setDistanceTollerance(distToll);
  tracker.setEnableAltitude(enableAltitude);
  tracker.setAltitudeTollerance(altitudeToll);
  tracker.setEnableOrientation(enableAngle);
  tracker.setOrientationTollerance(M_PI/180.0*angleTollDeg);

  cToInSub = nh.subscribe(cToInTopic, 1, transformationCallback);
//  if (enableAltitude) {
    altitudeSub = nh.subscribe(altitudeTopic, 1, altitudeCallback);
//  }
  cToOutPub = nh.advertise<maris_ros_msgs::TransfMatrix>(cToOutTopic, 1);

  ros::Rate r(15);
  while (ros::ok()) {
    ros::spinOnce();
  }
  return 0;
}

void altitudeCallback(const std_msgs::Float32& altitude)
{
//  std::cout << "Received altitude " << altitude.data << std::endl;
  tracker.setAltitude(altitude.data);
}

void transformationCallback(const maris_ros_msgs::TransfMatrix& inTransfMaris)
{
 std::cout << "---------------" << std::endl;
  std::cout << "Received frame" << std::endl;
  // Received frame is transformed to Eigen matrix 
  Eigen::Matrix4d inTransfEigen;
  Eigen::Matrix4d outTransfEigen;
  maris_ros_msgs::TransfMatrix outTransfMaris;
  marisToEigenPTN(inTransfMaris,inTransfEigen);
  // Test frame: if valid, it publishes the corrected frame
  bool valid = tracker.acceptTransform(inTransfEigen,outTransfEigen);
  if (valid) {
    eigenToMarisPTN(outTransfEigen,outTransfMaris);
    cToOutPub.publish(outTransfMaris);
  }
}

void eigenToMarisPTN(const Eigen::Matrix4d& eigenTransf,maris_ros_msgs::TransfMatrix& marisTransf) 
{
  for (int i = 0; i < 16; ++i) {
    int r = i % 4;
    int c = i / 4;
    marisTransf.cwmatrix[i] = eigenTransf(r,c);
  }
}

void marisToEigenPTN(const maris_ros_msgs::TransfMatrix& marisTransf,Eigen::Matrix4d& eigenTransf) 
{
  for (int i = 0; i < 16; ++i) {
    int r = i % 4;
    int c = i / 4;
    eigenTransf(r,c) = marisTransf.cwmatrix[i];
  }
}


