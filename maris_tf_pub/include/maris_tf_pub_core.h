/**
 * MARIS Vision - ROS Packages of Vision System used in MARIS project. 
 * Copyright (C) 2016 Fabjan Kallasi, Dario Lodi Rizzini and Fabio Oleari. 
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
#ifndef SR_MARISTFPUB_CORE_H
#define SR_MARISTFPUB_CORE_H

#include <vector>
#include <fstream>

// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"

#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <maris_ros_msgs/TransfMatrix.h>


using std::string;

class MarisTFPub
{
public:
  string tf_parent_frame;
  string tf_child_frame;
  
  //! Constructor.
  MarisTFPub();

  //! Destructor.
  ~MarisTFPub();


  //! Synchronized callback function for subscriber.
  void messageCallback(const maris_ros_msgs::TransfMatrix& transfMat);

private:

};

#endif // SR_MARISTFPUB_CORE_H
