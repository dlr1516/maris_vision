/**
 * MARIS Vision - ROS Packages of Vision System used in MARIS project. 
 * Copyright (C) 2016 Fabjan Kallasi.
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
#ifndef FRAME_TRACKER_H
#define FRAME_TRACKER_H

#include <iostream>
// It should be #include <Eigen/Dense> 
#include <eigen3/Eigen/Dense>  


/** This class tracks a reference frame over the time. 
 * This frame may be the rigid frame attached to an object to track and 
 * its pose is expressed w.r.t. another frame, e.g. the observer reference frame.
 */
class FrameTracker
{
public:
  enum TrackingState { EMPTY, PENDING, VALID };

  /** Constructor. 
   */
  FrameTracker();

  /** Destructor. 
   */
  ~FrameTracker() { }

  /**
   */ 
  TrackingState getTrackingState() const 
  { 
    if (hitCounter_ == 0 && missCounter_ == 0) return EMPTY;
    else if (hitCounter_ < hitMin_) return PENDING;
    else return VALID; 
  }

  /** Prints internal tracker state.
   */
  void printState(std::ostream& out = std::cout) const 
  {
    out << "hitCounter_ " << hitCounter_ << " (min accept " << hitMin_ << ", saturated " << hitMax_
     << ")  missCounter_ " << missCounter_ << " (reset at " << missMax_ << ")" << std::endl;
  }

  /** Returns the current value of frame.
   */
  const Eigen::Vector3d& getPosition() const { return position_; }

  /** Returns the current value of frame.
   */
  const Eigen::Quaterniond& getOrientation() const { return orientation_; }

  /** Returns the current value of frame.
   */
  Eigen::Matrix4d getTransformation() const;

  // --------------------------------------------------------
  // PREDICT FUNCTIONS
  // --------------------------------------------------------
  
  /** Predicts the frame using a different source, e.g. the motion of the vehicle
   * carrying the sensor. 
   * It may use position and orientation or only orientation. 
   * The latter case occurs for example when an IMU is used.
   * The input arguments of the functions are:
   * @param position the absolute position of sensor frame w.r.t. odometry frame
   * @param orientation the absolute orientation of sensor frame w.r.t. odometry frame
   * @param positionValid it is true if the position contains valid data, 
   *   otherwise it uses only orientation
   */
  void predict(const Eigen::Vector3d& position,const Eigen::Quaterniond& orientation,bool positionValid); 

  /** Different interface of predict() function accepting transformation as input. 
   */
  void predict(const Eigen::Matrix4d& transf,bool positionValid)
  {
     Eigen::Vector3d position;
     Eigen::Quaterniond orientation;
     fromTranformationToPosQuat(transf,position,orientation);
     predict(position,orientation,positionValid);
  }

  // --------------------------------------------------------
  // UPDATE FUNCTIONS
  // --------------------------------------------------------

  /** Updates internal state with the given frame measurement.
   * It returns true if the new measurement has been validated.
   */
  bool update(const Eigen::Vector3d& position,const Eigen::Quaterniond& orientation);

  /** Updates internal state with the given frame measurement.
   * It returns true if the new measurement has been validated.
   */
  bool update(const Eigen::Matrix4d& transf);

  /** Updates the state frame using an observation frame constrained to a line. 
   * The line is represented by a point pos and a direction vector:
   *    p(t) = linePos + lineDir * t     (t is the parameter of line).
   * The z-axis is kept from last frame and is NOT UPDATED!
   */
  bool updateLineOnly(const Eigen::Vector3d& linePos,const Eigen::Vector3d& lineDir);

  /** Updates the state frame using an observation frame constrained to a line. 
   * The line corresponds to the y-axis of the frame. 
   */
  bool updateLine(const Eigen::Vector3d& position,const Eigen::Quaterniond& orientation);

  /** Updates the state frame using an observation frame constrained to a line. 
   * The line corresponds to the y-axis of the frame. 
   */
  bool updateLine(const Eigen::Matrix4d& transf);

  // --------------------------------------------------------
  // INPUT VALIDATION FUNCTIONS
  // --------------------------------------------------------

  /** Validates the input frame value according to absolute tests.
   * "Absolute" means independent from previous estimation. 
   * The parameters of absolute tests are labeled with ABS in the following.
   */  
  bool validateAbs(const Eigen::Vector3d& position,const Eigen::Quaterniond& orientation) const;

  /** Validates the input frame value according to absolute tests.
   * "Absolute" means independent from previous estimation. 
   * The parameters of absolute tests are labeled with ABS in the following.
   */  
  bool validateAbs(const Eigen::Matrix4d& transf) const;

  /** Validates the input frame value by comparing it with the previous estimation.
   * The parameters of relative tests are labeled with REL in the following.
   */  
  bool validateRel(const Eigen::Vector3d& position,const Eigen::Quaterniond& orientation) const;

  /** Validates the input frame value by comparing it with the previous estimation.
   * The parameters of relative tests are labeled with REL in the following.
   */  
  bool validateRel(const Eigen::Matrix4d& transf) const;

  /** Sets the minimum number of "hits" (i.e. valid frames) to validate the state 
   * and saturation value for hit counter (the latter parameter does not really 
   * influence the result: missMax_ is the one used to reset state!). 
   */
  void setHitMinMax(int hmin,int hmax) { hitMin_ = hmin; hitMax_ = hmax; }

  /** Sets the maximum number of "misses" (i.e. invalid frames) to reset the state.
   */
  void setMissMax(int mmax) { missMax_ = mmax; }

  /** ABS. Enables range test.
   */
  void enableRange(bool on) { rangeOn_ = on; }

  /** ABS. Sets the minimum and maximum range bounds of the frame.
   * The range is the distance between the reference origin and the origin 
   * of the tracked frame.
   * If the bounds are respectively 0.60 m and 4.0 m, a measurement out of the bounds
   * is not validated. 
   */
  void setRangeMinMax(double rmin,double rmax) { rangeMin_ = rmin; rangeMax_ = rmax; }

  /** REL. Enables position test.
   */
  void enablePosition(bool on) { positionOn_ = on; }

  /** REL. Sets maximum distance w.r.t. the currente state estimation. 
   */
  void setPositionToll(double ptoll) { positionToll_ = ptoll; }

  /** REL. Enables orientation test.
   */
  void enableOrientation(bool on) { orientationOn_ = on; }

  /** REL. Sets the tollerance on orientation difference w.r.t. the current state estimation. 
   */
  void setOrientationToll(double otoll) { orientationToll_ = otoll; }

  /** Sets the update rate coefficient: it must be between 0.0 (no update) and 1.0 (no memory).
   */
  void setUpdateRate(double ur) 
  {
    if (ur < 0.0 || ur > 1.0) {
      std::cerr << __FILE__ << "," << __LINE__ << ": invalid update rate " << ur 
        << ": must be in interval [0.0,1.0]: rate " << updateRate_ << " unchanged" << std::endl;
      return;
    }
    updateRate_ = ur;
  }
  
  /** Enables messages to standard output. 
   */
  void enableOutput(bool on) { outputOn_ = on; }

  /** Converts position vector and quaternion to homogenous transformation matrix.
   */
  static void fromPosQuatToTranformation(const Eigen::Vector3d& pos,const Eigen::Quaterniond& quat,Eigen::Matrix4d& transf);

  /** Converts homogenous transformation matrix to position vector and quaternion.
   */
  static void fromTranformationToPosQuat(const Eigen::Matrix4d& transf,Eigen::Vector3d& pos,Eigen::Quaterniond& quat);

  /** Returns the angle difference between two rotations represented as quaternion. 
   */
  static double angleDifference(const Eigen::Quaterniond& q1,const Eigen::Quaterniond& q2);

  /** Converts quaternion to axis angle. 
   */
  static void computeRotationAxis(const Eigen::Quaterniond& q,Eigen::Vector3d& axis,double& angle);

  /** Computes the line parameter s that minimizes the distance:
   *   smin = argmin | point - (linePos + lineDir * s) |^2
   * The projection point is  pointProj = linePos + lineDir * smin. 
   */
  static double lineProjection(const Eigen::Vector3d& linePos,const Eigen::Vector3d& lineDir,const Eigen::Vector3d& point,
    Eigen::Vector3d& pointProj);

private:
  // State vector and covariance
  Eigen::Vector3d position_;
  Eigen::Quaterniond orientation_;
  //Eigen::Matrix3d covariance_;
  double updateRate_; 
  // Last "odometry" position and orientation
  Eigen::Vector3d positionOdom_;
  Eigen::Quaterniond orientationOdom_;
  bool validPositionOdom_;
  bool validOrientationOdom_;
  // Detection counters to handle state flags
  int hitCounter_;    // count the successive consistent measurements
  int missCounter_;   // count the successive inconsistent measurements
  int hitMin_;
  int hitMax_;
  int missMax_;
  // Parameters for testing it
  bool rangeOn_;
  double rangeMin_;
  double rangeMax_;
  bool positionOn_;
  double positionToll_;
  bool orientationOn_;
  double orientationToll_;
  bool altitudeOn_;     
  double altitude_;      
  double altitudeToll_;
  // Output
  bool outputOn_;

  /** Increments the hit counter and decreases the miss counter.
   */
  void addValidMeasure()
  {
    if (hitCounter_ < hitMax_) {
      hitCounter_++;
    }
    if (missCounter_ > 0) {
      missCounter_--;
    }
  }

  /** Increments the hit counter and decreases the miss counter.
   */
  void addWrongMeasure()
  {
    if (hitCounter_ > 0) {
      missCounter_++;
      if (missCounter_ > missMax_) {
        hitCounter_ = 0;
        missCounter_ = 0;
      }
    }
  }
};

#endif

