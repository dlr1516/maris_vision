/**
 * MARIS Vision - ROS Packages of Vision System used in MARIS project. 
 * Copyright (C) 2016 Dario Lodi Rizzini.
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
#include <iostream>
#include <string>
#include <ros/ros.h>
#include <ros/time.h>

// Messages: standard ROS and custom MARIS
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>
#include <maris_ros_msgs/TransfMatrix.h>
#include <maris_ros_msgs/VehiclePosition.h>

// Local data
#include <maris_pipe_pose_estimation/FrameTracker.h>
#include <maris_pipe_pose_estimation/frame_tracker_paramsConfig.h>
#include <maris_pipe_pose_estimation/Line3D.h>
#include <dynamic_reconfigure/server.h>

ros::Subscriber worldToObserverSub;
ros::Subscriber vTcSub;
ros::Subscriber observerToTargetInSub;
ros::Subscriber observerToTargetLineInSub;
ros::Publisher observerToTargetOutPub;
ros::Publisher lineMarkerPub;
ros::Publisher framePosePub;
ros::Subscriber altitudeSub;
FrameTracker tracker;
Eigen::Vector3d observerZ;
bool observerZOn;


// Global information
Eigen::Matrix4d vTcEigen;

// Callback to read depth from system vehicle
void altitudeCallback(const std_msgs::Float32& altitude);

// Callback to read input target pose transformation
void observerToTargetCallback(const maris_ros_msgs::TransfMatrix& transf);

// Callback to read input target line: the y-axis of target frame lies on such line
// and is oriented according to z-axis of worldToObserver frame
void observerToTargetLineCallback(const maris_pipe_pose_estimation::Line3D& line);

// Callback to read wTv
void worldToObserverCallback(const maris_ros_msgs::VehiclePosition& vp);

// Dynamic configuration
void configCallback(maris_pipe_pose_estimation::frame_tracker_paramsConfig& cfg, uint32_t level);

// Conversion functions
void eigenToMarisPTN(const Eigen::Matrix4d& eigenTransf,maris_ros_msgs::TransfMatrix& marisTransf);

void marisToEigenPTN(const maris_ros_msgs::TransfMatrix& marisTransf,Eigen::Matrix4d& eigenTransf);

// Sets the line marker to visualize marker
void setLineMarker(visualization_msgs::Marker& m,Eigen::Matrix4d& eigenTransf, double len, double radius,const std_msgs::Header& header);

// Sets the pose message
void setPose(geometry_msgs::PoseStamped& pose,const Eigen::Matrix4d& transf,const std_msgs::Header& header);

// MAIN
int main(int argc,char** argv)
{
  ros::init(argc, argv, "frame_tracker_node");
  ros::NodeHandle nh("~");

  std::string observerToTargetInTopic, observerToTargetOutTopic, worldToObserverTopic;
  std::string observerToTargetLineInTopic, altitudeTopic, markerOutTopic, poseOutTopic;
  double loopHz;
  double updateRate;
  bool rangeOn;
  double rangeMin, rangeMax;
  bool positionOn;
  double positionToll;
  bool orientationOn;
  double orientationTollDeg;
  int hitMin, hitMax, missMax;
  double radius, length;
  visualization_msgs::Marker markerMsg;
  geometry_msgs::PoseStamped poseMsg;
  std_msgs::Header headerMsg;
  std::string ns, frameId;

  // Data to publish the estimated frame
  Eigen::Matrix4d outTransfEigen;
  maris_ros_msgs::TransfMatrix outTransfMaris;

  // Initial value of observerZ (initialization is required when wTv is NOT PUBLISHED,
  // e.g. during offline tests)
  observerZ << 0.0, 0.0, -1.0;

  nh.param<double>("loopHz", loopHz, 15.0);
  nh.param<std::string>("worldToObserverTopic", worldToObserverTopic, "/vision/wTv");
  nh.param<std::string>("observerToTargetInTopic", observerToTargetInTopic, "/vision/vTo");
  nh.param<std::string>("observerToTargetLineInTopic",observerToTargetLineInTopic, "/vision/line");
  nh.param<std::string>("observerToTargetOutTopic", observerToTargetOutTopic, "/vision/cToCorrected");
  nh.param<std::string>("altitudeTopic", altitudeTopic, "/vehicle/altitude");
  nh.param<std::string>("markerOutTopic", markerOutTopic, "/vision/lineCorrectedMarker");
  nh.param<std::string>("poseOutTopic", poseOutTopic, "/vision/framePose");
  nh.param<double>("updateRate",updateRate,0.5);
  nh.param<bool>("rangeOn",rangeOn,false);
  nh.param<double>("rangeMin",rangeMin,1.0);
  nh.param<double>("rangeMax",rangeMax,5.0);
  nh.param<bool>("positionOn",positionOn,false);
  nh.param<double>("positionToll",rangeMax,1.0);
  nh.param<bool>("orientationOn",orientationOn,false);
  nh.param<double>("orientationTollDeg",orientationTollDeg,20.0);
  nh.param<bool>("observerZOn",observerZOn,true);
  nh.param<int>("hitMin",hitMin,2);
  nh.param<int>("hitMax",hitMax,10);
  nh.param<int>("missMax",missMax,5);
  nh.param<double>("markerRadius",radius,0.05);
  nh.param<double>("markerLength",length,1.2);
  nh.param<std::string>("ns",ns,"/vision");
  nh.param<std::string>("frameId",frameId,"/vision");

  std::cout << "Parameters:"
    << "\n  observerToTargetInTopic " << observerToTargetInTopic 
    << "\n  observerToTargetOutTopic " << observerToTargetOutTopic 
    << "\n  observerToTargetLineInTopic " << observerToTargetLineInTopic
    << "\n  worldToObserverTopic " << worldToObserverTopic 
    << "\n  updateRate " << updateRate
    << std::endl;

  // Enable tests on input data to filter potential outliers
  tracker.setHitMinMax(hitMin,hitMax);
  tracker.setMissMax(missMax);
  tracker.setUpdateRate(updateRate);
  tracker.enableRange(rangeOn);
  tracker.setRangeMinMax(rangeMin,rangeMax);
  tracker.enablePosition(positionOn);
  tracker.enableOrientation(orientationOn);
  tracker.setOrientationToll(M_PI/180.0*orientationTollDeg);
  tracker.enableOutput(false);

  worldToObserverSub = nh.subscribe(worldToObserverTopic, 1, worldToObserverCallback);
  observerToTargetInSub = nh.subscribe(observerToTargetInTopic, 1, observerToTargetCallback);
  observerToTargetLineInSub = nh.subscribe(observerToTargetLineInTopic, 1, observerToTargetLineCallback);
  observerToTargetOutPub = nh.advertise<maris_ros_msgs::TransfMatrix>(observerToTargetOutTopic, 1);
  lineMarkerPub = nh.advertise<visualization_msgs::Marker>(markerOutTopic, 1);
  framePosePub = nh.advertise<geometry_msgs::PoseStamped>(poseOutTopic, 1);
  // Altitudes is not used, but could be added in order to 
  //altitudeSub = nh.subscribe(altitudeTopic, 1, altitudeCallback);

  dynamic_reconfigure::Server<maris_pipe_pose_estimation::frame_tracker_paramsConfig> paramServer;
  dynamic_reconfigure::Server<maris_pipe_pose_estimation::frame_tracker_paramsConfig>::CallbackType pc = boost::bind(configCallback, _1, _2);
  paramServer.setCallback(pc);

  ros::Rate loopRate(loopHz);
  while (ros::ok()) {
    ros::spinOnce();

    // Publishes the estimates transformation
    outTransfEigen = tracker.getTransformation();
    eigenToMarisPTN(outTransfEigen,outTransfMaris);

    // Publishes cylinder...
    if (tracker.getTrackingState() != FrameTracker::EMPTY) {
      observerToTargetOutPub.publish(outTransfMaris);
      std::cout << "Frame trasker: frame value\n" << outTransfEigen << std::endl;

      headerMsg.frame_id = frameId;
      headerMsg.stamp = ros::Time::now();
      setLineMarker(markerMsg,outTransfEigen,length,radius,headerMsg);
      setPose(poseMsg,outTransfEigen,headerMsg);

      lineMarkerPub.publish(markerMsg);
      framePosePub.publish(poseMsg);
    }

//    tracker.printState();

    loopRate.sleep();
  }
  return 0;
}


void observerToTargetCallback(const maris_ros_msgs::TransfMatrix& transf)
{
  std::cout << "---------------" << std::endl;
  std::cout << "Received FRAME" << std::endl;

  // Received frame is transformed to Eigen matrix 
  Eigen::Matrix4d transfEigen;
  marisToEigenPTN(transf,transfEigen);
  tracker.update(transfEigen);

//  std::cout << "AFTER UPDATE" << std::endl;
//  tracker.printState();
//  std::cout << "\n\n";
}

void observerToTargetLineCallback(const maris_pipe_pose_estimation::Line3D& line)
{
  Eigen::Matrix4d transfEigen = Eigen::Matrix4d::Zero();
  Eigen::Vector3d vx;
  Eigen::Vector3d vy;
  Eigen::Vector3d vz;
  Eigen::Vector3d pos;

  std::cout << "---------------" << std::endl;
  std::cout << "Received LINE " << std::endl;

  if (observerZOn) {
    // z-axis of target frame = z-axis of observer in world frame coordinates
    // (actually the "plumb bob" direction)
    vz = observerZ;
    if (std::abs(vz.norm()) < 1e-6) {
      std::cerr << __FUNCTION__ << ": invalid z-axis " << vz.transpose() << std::endl;
      return;
    }
    vz = vz / vz.norm();

    // Line gives an approximate position (not used by tracker.updateLine())
    // and the y-axis (the line direction)
    pos << line.p0.x, line.p0.y, line.p0.z;
    vy << line.d.x, line.d.y, line.d.z;
    if (std::abs(vy.norm()) < 1e-6) {
      std::cerr << __FUNCTION__ << ": invalid y-axis" << vy.transpose() << std::endl;
      return;
    }
    vy = vy / vy.norm();

    // x-axis defined by previous axes
    vx = vy.cross(vz);

    // Transformation axis
    transfEigen.block<3, 1>(0, 0) = vx;
    transfEigen.block<3, 1>(0, 1) = vy;
    transfEigen.block<3, 1>(0, 2) = vz;
    transfEigen.block<3, 1>(0, 3) = pos;
    transfEigen(3, 3) = 1.0;

    // Updates tracker
    tracker.updateLine(transfEigen);
  }
  else {
    std::cout << "\n****** LINE ONLY ******\n" << std::endl;

    pos << line.p0.x, line.p0.y, line.p0.z;
    vy << line.d.x, line.d.y, line.d.z;
    tracker.updateLineOnly(pos,vy);
  }
  std::cout << "  p0 " << pos.transpose() << ", dir " << vy.transpose() << std::endl;
}

// Callback to read wTv
void worldToObserverCallback(const maris_ros_msgs::VehiclePosition& vp)
{
  Eigen::Matrix4d wTvEigen;

  // Sets the pose of camera w.r.t. vehicle frame
  marisToEigenPTN(vp.wTv,wTvEigen);

  // Saves the z-axis of observer frame w.r.t. world frame
  observerZ = wTvEigen.block<3,1>(0,2);
  std::cout << "OBSERVER_Z: " << observerZ.transpose() << std::endl;
 
  // Uses wTcEigen for object pose prediction
  tracker.predict(wTvEigen,vp.validTranslation);
}


void altitudeCallback(const std_msgs::Float32& altitude)
{
  // Altitude is not used yet. It could be used in validateAbs() 
  // instead of or combined to range test.
  // If we know the altidude, i.e. distance from swimmingpool ground/seabed, 
  // and the approximate distance of object from the ground, then
  // the unlikely frame could be filtered. 
}

void configCallback(maris_pipe_pose_estimation::frame_tracker_paramsConfig& cfg, uint32_t level) 
{
  tracker.setUpdateRate(cfg.updateRate);
  tracker.setHitMinMax(cfg.hitMin,cfg.hitMax);
  tracker.setMissMax(cfg.missMax);
  tracker.enableRange(cfg.rangeOn);
  tracker.setRangeMinMax(cfg.rangeMin,cfg.rangeMax);
  tracker.enablePosition(cfg.positionOn);
  tracker.setPositionToll(cfg.positionToll);
  tracker.enableOrientation(cfg.orientationOn);
  tracker.setOrientationToll(M_PI/180.0*cfg.orientationToll);
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

void setLineMarker(visualization_msgs::Marker& m,Eigen::Matrix4d& eigenTransf, double len, double radius,const std_msgs::Header& header) 
{
  m.header = header;

  m.color.a = 0.8;
  m.color.b = 0.0;
  m.color.g = 1.0;
  m.color.r = 1.0;

  Eigen::Vector3d pos = eigenTransf.block<3,1>(0,3);
  Eigen::Vector3d dir = eigenTransf.block<3,1>(0,1);  // axis y rotation matrix
//  std::cout << "pos: " << pos.transpose() << ", dir " << dir.transpose() << std::endl;

  m.points.resize(2);
  m.points[0].x = pos[0] + 0.5*len * dir[0];
  m.points[0].y = pos[1] + 0.5*len * dir[1];
  m.points[0].z = pos[2] + 0.5*len * dir[2];

  m.points[1].x = pos[0] - 0.5*len * dir[0];
  m.points[1].y = pos[1] - 0.5*len * dir[1];
  m.points[1].z = pos[2] - 0.5*len * dir[2];

  m.scale.x = radius;
  m.scale.y = radius;
  m.scale.z = 0.01;
}

void setPose(geometry_msgs::PoseStamped& pose,const Eigen::Matrix4d& transf,const std_msgs::Header& header)
{
  pose.header = header;

  Eigen::Quaterniond qr(transf.block<3, 3>(0, 0));
  pose.pose.orientation.w = qr.w();
  pose.pose.orientation.x = qr.x();
  pose.pose.orientation.y = qr.y();
  pose.pose.orientation.z = qr.z();
  pose.pose.position.x = transf(0, 3);
  pose.pose.position.y = transf(1, 3);
  pose.pose.position.z = transf(2, 3);
}

