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
#include <maris_pipe_pose_estimation/PipePoseTracker.h>


bool PipePoseTracker::acceptTransform(const Eigen::Matrix4d& inTransf,Eigen::Matrix4d& outTransf)
{
  Eigen::Matrix4d transfAdjusted;
  
  // Initializes transform at the beginning
  if (hitCounter_ == 0 && missCounter_ == 0) {
    hitCounter_++;
    lastTransf_ = inTransf;
    outTransf = inTransf;
    return false;
  }

  // Tests the distance w.r.t. last accepted reference frame
  Eigen::Vector3d posCurr, posLast;
  posCurr << inTransf(0,3), inTransf(1,3), inTransf(2,3);
  posLast << lastTransf_(0,3), lastTransf_(1,3), lastTransf_(2,3);
  double range = posCurr.norm();
  double dist = (posCurr - posLast).norm();
  std::cout << "Distance from last frame " << dist << ": tollerance " << distToll_ << ", range " << range 
    << ", min-max " << rangeMin_ << "," << rangeMax_ << std::endl;
  if (dist > distToll_ || range < rangeMin_ || range > rangeMax_) {
    std::cerr << "INVALID FRAME: distance " << dist << " >= frame distance origin " << distToll_ 
      << "\n  or range: " << rangeMin_ << " < " << range << " or\ " << range << " > " << rangeMax_ << std::endl;
    missCounter_++;
    if (missCounter_ > missMax_) {   // Too many missed frames...
      hitCounter_ = 0;        // ...reset the hitCounter_
      missCounter_ = 0;
    }
    return false;
  }
  // Test on altitude (if enabled)
  double tz = inTransf(2,3);
  std::cout << "Altitude " << altitude_ << ", estimated " << tz << std::endl;
  if (enableAltitude_) {
    if (std::abs(tz-altitude_) > altitudeToll_) {
      std::cerr << "INVALID FRAME: altitude (tz) " << tz << ": measured altitude " << altitude_ << ", tollerance " << altitudeToll_ << std::endl;
      missCounter_++;
      if (missCounter_ > missMax_) {   // Too many missed frames...
        hitCounter_ = 0;               // ...reset the hitCounter_
        missCounter_ = 0;
      }
      return false;    
    }
  }
  // Test orientation: the orientation of axis y must be consistent
  Eigen::Matrix3d rotCurr = inTransf.block<3,3>(0,0);  
  Eigen::Matrix3d rotLast = lastTransf_.block<3,3>(0,0);
  Eigen::Matrix3d rotDiff = rotCurr * rotLast.transpose();
  Eigen::Vector3d axis;
  double angle;
  computeRotationAxis(rotDiff,axis,angle);
  std::cout << "Orientation diff w.r.t. last: " << (180.0/M_PI*angle) << " deg: axis " << axis.transpose() << std::endl;
  if (enableAngle_) {
    if (std::abs(angle) > angleToll_) {
      std::cerr << "INVALID FRAME: distance " << dist << " >= frame distance origin " << distToll_ << std::endl;
      missCounter_++;
      if (missCounter_ > missMax_) {   // Too many missed frames...
        hitCounter_ = 0;               // ...reset the hitCounter_
        missCounter_ = 0;
      }
      return false;
    }
  }
  // Increments hit numbers and reset miss (we count only consecutive miss!)
  std::cout << "Accepted transformation:\n" << inTransf << "\nhit " << hitCounter_ << ", miss " << missCounter_ << std::endl;
  hitCounter_++;
  missCounter_ = 0;
  outTransf = inTransf;
  lastTransf_ = inTransf;
  // It does not publish the promising frame, but it acknoledges the correctness
  if (hitCounter_ < hitMin_) {
    return false;
  }
  return true;
}

void PipePoseTracker::computeRotationAxis(const Eigen::Matrix3d& rot,Eigen::Vector3d& axis,double& angle)
{
  Eigen::Quaterniond q(rot);
  double d = sqrt(q.x()*q.x() + q.y()*q.y() + q.z()*q.z());
  angle = 2.0f * atan2(d,q.w());
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
