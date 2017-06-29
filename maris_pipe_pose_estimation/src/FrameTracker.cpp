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
#include <maris_pipe_pose_estimation/FrameTracker.h>

// --------------------------------------------------------
// CONSTRUCTOR AND DATA EXPLICIT
// --------------------------------------------------------

FrameTracker::FrameTracker()
 : position_(Eigen::Vector3d::Zero()), orientation_(Eigen::Quaterniond::Identity()), 
   //covariance_(Eigen::Matrix3d::Identity()), 
   updateRate_(0.5),
   positionOdom_(Eigen::Vector3d::Zero()), orientationOdom_(Eigen::Quaterniond::Identity()),
   validPositionOdom_(false), validOrientationOdom_(false),
   hitCounter_(0), missCounter_(0), hitMin_(2), hitMax_(10), missMax_(3),
   rangeOn_(true), rangeMin_(0.6), rangeMax_(3.0), 
   positionOn_(true), positionToll_(1.0), 
   orientationOn_(false), orientationToll_(M_PI/180.0*45.0), 
   altitudeOn_(false), altitude_(0.0)
{
}

Eigen::Matrix4d FrameTracker::getTransformation() const 
{
  Eigen::Matrix4d transf;
  fromPosQuatToTranformation(position_,orientation_,transf);
  return transf;
}

// --------------------------------------------------------
// PREDICT FUNCTIONS
// --------------------------------------------------------

void FrameTracker::predict(const Eigen::Vector3d& position,const Eigen::Quaterniond& orientation,bool positionValid)
{
  Eigen::Matrix4d wTprev;
  Eigen::Matrix4d wTcurr;
  Eigen::Matrix4d currTprev;
  Eigen::Matrix4d state;

  // validOrientationOdom_ is false at first iteration, when no previous orientation is available
  if (validOrientationOdom_) {
    // If current position AND previous position are both valid, it predicts position and orientation.
    // Otherwise it predicts only orientation
    if (positionValid && validPositionOdom_) {
      fromPosQuatToTranformation(position,orientation,wTcurr);
      fromPosQuatToTranformation(positionOdom_,orientationOdom_,wTprev);
    }
    else {
      fromPosQuatToTranformation(Eigen::Vector3d::Zero(),orientation,wTcurr);
      fromPosQuatToTranformation(Eigen::Vector3d::Zero(),orientationOdom_,wTprev);
    }
    currTprev = wTcurr.inverse() * wTprev;
  }

  // Updates the current state 
  fromPosQuatToTranformation(position_,orientation_,state);
  state = currTprev * state;
  fromTranformationToPosQuat(state,position_,orientation_);

  // Updates next position or orientation
  positionOdom_ = position;
  orientationOdom_ = orientation;
  validPositionOdom_ = positionValid;
  validOrientationOdom_ = true;
}

// --------------------------------------------------------
// UPDATE FUNCTIONS
// --------------------------------------------------------

bool FrameTracker::update(const Eigen::Vector3d& position,const Eigen::Quaterniond& orientation)
{
  // Checks if the input state is validated
  if (!validateAbs(position,orientation)) {
//    missCounter_++;
//    if (missCounter_ > missMax_) {
//      outputOn_ && std::cout << __FUNCTION__ << ": after ABS validation: RESET state" << std::endl;
//      hitCounter_ = 0;
//      missCounter_ = 0;
//    }
    addWrongMeasure();
    return false;
  }
  // If the state is uninitialized, the current state is initialized.
  // If a single valid frame value is enough to start estimation, the state is approved.
  if (hitCounter_ == 0 && missCounter_ == 0) {
    outputOn_ && std::cout << __FUNCTION__ << ": INITIALIZE state" << std::endl;
    position_ = position;
    orientation_ = orientation;
//    hitCounter_++;
//    if (hitCounter_ >= hitMin_) {
//      return true;
//    }
//    else {
//      return false;
//    }
    addValidMeasure();
    return true;
  }
  // Checks if the input state is validated w.r.t. previous frame estimation
  if (!validateRel(position,orientation)) {
//    missCounter_++;
//    if (missCounter_ > missMax_) {
//      outputOn_ && std::cout << __FUNCTION__ << ": after REL validation : RESET state" << std::endl;
//      hitCounter_ = 0;
//      missCounter_ = 0;
//    }
    addWrongMeasure();
    return false;
  }
  // The frame value is valid and can be updated.
//  Eigen::Matrix3d infoPrev = covariance_.inverse();
//  Eigen::Matrix3d infoCurr = covariance.inverse();
//  double coeffPrev = 1.0/covariance_.trace();
//  double coeffCurr = 1.0/covariance.trace();
//  covariance_ = (infoPrev + infoCurr).inverse();
//  position_ = covariance_ * (infoPrev * position_ + infoCurr * position);
//  orientation_.slerp(coeffPrev / (coeffPrev + coeffCurr), orientation);
  position_ = (1.0-updateRate_) * position_ + updateRate_ * position;
  //orientation_.slerp((1.0 - updateRate_), orientation);
  orientation_ = orientation_.slerp(updateRate_, orientation);
  addValidMeasure();
  return true;
}

bool FrameTracker::update(const Eigen::Matrix4d& transf)
{
  Eigen::Vector3d pos;
  Eigen::Quaterniond quat;
  fromTranformationToPosQuat(transf,pos,quat);
  return update(pos,quat);
}

bool FrameTracker::updateLineOnly(const Eigen::Vector3d& linePos,const Eigen::Vector3d& lineDir)
{
  Eigen::Vector3d positionProj;
  Eigen::Matrix3d rot;
  Eigen::Vector3d vx;
  Eigen::Vector3d vy;
  Eigen::Vector3d vz;
  Eigen::Quaterniond orientation;
  Eigen::Quaterniond rotation;
  Eigen::Vector3d nx;
  Eigen::Vector3d ny;
  Eigen::Vector3d nz;
  Eigen::Quaterniond orientationStatePrev;

  // The line information can be used in tracking only if there is at least 
  // an observation about the frame. 
  // In this case, the previous frame origin is projected on the current line.
  if (getTrackingState() != EMPTY) {
    // The previous frame origin position (position_) is projected on the line
    //   p(s) = position + lineDir * s
    // and positionProj = argmin | p(s) - position_ |^2.
    // The vector positionProj is used as origin of the observed frame. 
    double s = lineProjection(linePos,lineDir,position_,positionProj);

    // Gets axes of current orientation_ 
    rot = orientation_.toRotationMatrix();
    vx = rot.block<3,1>(0,0);
    vy = rot.block<3,1>(0,1);
    vz = rot.block<3,1>(0,2);

    // Computes the new rotation matrix using lineDir as axis y and the previous axis z
    nz = vz;
    ny = lineDir;
    if (std::abs(ny.norm()) < 1e-6) {
      std::cerr << __FUNCTION__ << ": invalid y-axis" << ny.transpose() << std::endl;
      return false;
    }
    ny = ny / ny.norm();
    nx = ny.cross(nz);

    // Converts the rotation matrix to quaternion 
    rot.block<3,1>(0,0) = nx;
    rot.block<3,1>(0,1) = ny;
    rot.block<3,1>(0,2) = nz;
    orientation = rot;

    // Updates the state using:
    // a) linear interpolation between previous frame origin and its projection on the line
    // b) SLERP between previous orientation and the orientation estimated as above
    orientationStatePrev = orientation_;
    update(positionProj,orientation);

//    position_ = (1.0-updateRate_) * position_ + updateRate_ * positionProj;
//    orientation.setFromTwoVectors(vy,lineDir);
//    rotation = Eigen::Quaterniond::Identity .slerp(updateRate_,orientation;
//    orientation_ = rotation * orientation;


    std::cout << __FILE__ << "," << __LINE__ << ": ORIENTATION DIFFERENCE "
      << (180.0/M_PI*angleDifference(orientation,orientationStatePrev)) << " deg"
      << "\n  ORIENTATION INNOVATION "
      << (180.0/M_PI*angleDifference(orientation_,orientationStatePrev)) << " deg" << std::endl;
    return true;
  }
  return false;
}

bool FrameTracker::updateLine(const Eigen::Vector3d& position,const Eigen::Quaterniond& orientation)
{
  Eigen::Vector3d positionProj;
  Eigen::Quaterniond orientationStatePrev;

  // The line information can be used in tracking only if there is at least 
  // an observation about the frame. 
  // In this case, the previous frame origin is projected on the current line.
  if (getTrackingState() != EMPTY) {
    Eigen::Vector3d lineDir = orientation.toRotationMatrix().block<3,1>(0,1);
    // The previous frame origin position (position_) is projected on the line
    //   p(s) = position + lineDir * s
    // and positionProj = argmin | p(s) - position_ |^2.
    // The vector positionProj is used as origin of the observed frame. 
    double s = lineProjection(position,lineDir,position_,positionProj);
    orientationStatePrev = orientation_;
    update(positionProj,orientation);

    std::cout << __FILE__ << "," << __LINE__ << ": ORIENTATION DIFFERENCE "
      << (180.0/M_PI*angleDifference(orientation,orientationStatePrev)) << " deg"
      << "\n  ORIENTATION INNOVATION "
      << (180.0/M_PI*angleDifference(orientation_,orientationStatePrev)) << " deg" << std::endl;
    return true;
  }
  return false;
}

bool FrameTracker::updateLine(const Eigen::Matrix4d& transf)
{
  Eigen::Vector3d pos;
  Eigen::Quaterniond quat;
  fromTranformationToPosQuat(transf,pos,quat);
  return updateLine(pos,quat);
}

// --------------------------------------------------------
// INPUT VALIDATION FUNCTIONS
// --------------------------------------------------------

bool FrameTracker::validateAbs(const Eigen::Vector3d& position,const Eigen::Quaterniond& orientation) const
{
  // Test range, i.e. distance of frame from the origin (the sensor origin)
  double range = position.norm();
  if (rangeOn_ && (range < rangeMin_ || range > rangeMax_)) {
    outputOn_ && std::cerr << __FUNCTION__ << ": INVALID range " << range 
      << ": min " << rangeMin_ << ", max " << rangeMax_ << std::endl;
    return false;
  }
  // Test the altitude
  
  return true;
}

bool FrameTracker::validateAbs(const Eigen::Matrix4d& transf) const
{
  Eigen::Vector3d pos;
  Eigen::Quaterniond quat;
  fromTranformationToPosQuat(transf,pos,quat);
  return validateAbs(pos,quat);
}

bool FrameTracker::validateRel(const Eigen::Vector3d& position,const Eigen::Quaterniond& orientation) const
{
  // Test position distance
  double dist = (position - position_).norm();
  if (positionOn_ && dist > positionToll_) {
    outputOn_ && std::cerr << __FUNCTION__ << ": INVALID position " << position.transpose() 
      << ":\n  previous position " << position_.transpose() << ", distance " << dist 
      << ", tollerance " << positionToll_ << std::endl;
    return false;
  }
  // Test orientation difference
  double angleDiff = angleDifference(orientation,orientation_);
  if (orientationOn_ && std::abs(angleDiff) < orientationToll_) {
    outputOn_ && std::cerr << __FUNCTION__ << ": orientation difference " 
     << (180.0/M_PI*angleDiff) << " deg" << std::endl;
    return false;
  }
  return true;
}

bool FrameTracker::validateRel(const Eigen::Matrix4d& transf) const
{
  Eigen::Vector3d pos;
  Eigen::Quaterniond quat;
  fromTranformationToPosQuat(transf,pos,quat);
  return validateRel(pos,quat);
}

// --------------------------------------------------------
// STATIC FUNCTIONS (CONVERSION)
// --------------------------------------------------------

void FrameTracker::fromPosQuatToTranformation(const Eigen::Vector3d& pos,const Eigen::Quaterniond& quat,Eigen::Matrix4d& transf)
{
  transf = Eigen::Matrix4d::Identity();
  transf.block<3,1>(0,3) = pos;
  transf.block<3,3>(0,0) = quat.toRotationMatrix();
}

void FrameTracker::fromTranformationToPosQuat(const Eigen::Matrix4d& transf,Eigen::Vector3d& pos,Eigen::Quaterniond& quat)
{
  pos = transf.block<3,1>(0,3);
  quat = transf.block<3,3>(0,0);
}

double FrameTracker::angleDifference(const Eigen::Quaterniond& q1,const Eigen::Quaterniond& q2)
{
  Eigen::Vector3d axis;
  double angle;
  Eigen::Quaterniond dq = q1 * q2.inverse();
  computeRotationAxis(dq,axis,angle);
  return angle;
}

void FrameTracker::computeRotationAxis(const Eigen::Quaterniond& q,Eigen::Vector3d& axis,double& angle)
{
  double d = sqrt(q.x()*q.x() + q.y()*q.y() + q.z()*q.z());
  angle = 2.0 * atan2(d,q.w());
  if (std::abs(sin(angle)) > 1e-3) {
    axis.x() = q.x() / sin(angle);
    axis.y() = q.y() / sin(angle);
    axis.z() = q.z() / sin(angle);
    axis.normalize();
  }
  else {
    std::cerr << __FILE__ << "," << __LINE__ << ": matrix closer to identity" << std::endl;
  }
}

double FrameTracker::lineProjection(const Eigen::Vector3d& linePos,const Eigen::Vector3d& lineDir,
  const Eigen::Vector3d& point,Eigen::Vector3d& pointProj)
{
  // Computes the translation parameters that minimize the distance:
  //   smin = argmin | point - (linePos + lineDir * s) |^2
  double a, smin;
  a = (lineDir.dot(lineDir));
  if (std::abs(a) < 1e-6) {
    std::cerr << __FILE__ << "," << __LINE__ << ": null line direction!" << std::endl;
    return 0.0;
  }
  smin = (lineDir.dot(point - linePos)) / (lineDir.dot(lineDir));
  pointProj = linePos + lineDir * smin;
  return smin;
}

