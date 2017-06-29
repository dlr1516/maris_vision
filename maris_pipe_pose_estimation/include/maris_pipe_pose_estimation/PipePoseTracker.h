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
#ifndef PIPE_POSE_TRACKER_H
#define PIPE_POSE_TRACKER_H

#include <eigen3/Eigen/Dense>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <cmath>

class PipePoseTracker
{
public: 
  /** Default constructor.
   */
  PipePoseTracker() 
   : hitCounter_(0), missCounter_(0), hitMin_(2), missMax_(3),
     rangeMax_(4.0), rangeMin_(1.0), distToll_(1.0), 
     enableAltitude_(false), altitude_(0.0), altitudeToll_(1.0),
     enableAngle_(false), angleToll_(M_PI/180.0*30)
  { } 

  void setDistanceTollerance(double toll) { distToll_ = toll; }

  /** Enables altitude test.
   */
  void setEnableAltitude(bool enable) { enableAltitude_ = enable; }

  /** Sets the altitude value in meters.
   */
  void setAltitude(double d) { altitude_ = d; }

  /** Sets the altitude value in meters.
   */
  void setAltitudeTollerance(double toll) { altitudeToll_ = toll; }

  void setEnableOrientation(bool en) { enableAngle_ = en; }

  void setOrientationTollerance(double toll) { angleToll_ = toll; }

  void setRangeMin(double rmin) { rangeMin_ = rmin; }

  void setRangeMax(double rmax) { rangeMax_ = rmax; }

  /** Adds a new measurement. 
   */
//  void addMeasurement(const Eigen::Matrix4d& transfNew);

  /** Says if a transformation matrix is consistent with previous measurements. 
   * It writes the corrected frame if a correction is implemented, otherwise it is equal 
   * to the input frame.
   */
  bool acceptTransform(const Eigen::Matrix4d& inTransf,Eigen::Matrix4d& outTransf);

  static void computeRotationAxis(const Eigen::Matrix3d& rot,Eigen::Vector3d& axis,double& angle);

protected:
  Eigen::Matrix4d lastTransf_;  // last consistent measurement (adjusted...?)
  int hitCounter_;    // count the successive consistent measurements
  int missCounter_;   // count the successive inconsistent measurements
  int hitMin_;
  int missMax_;
  double distToll_;
  double rangeMin_;
  double rangeMax_;
  bool enableAltitude_;     // use altitude information (from vehicle) to check transformation consistency
  double altitude_;      
  double altitudeToll_;
  bool enableAngle_;
  double angleToll_;
};

#endif

