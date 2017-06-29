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
#include "std_msgs/String.h"

#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <string>
#include <sys/time.h>
#include <math.h> 



#include <sstream>

#include "maris_tf_pub_core.h"
#include <tf/transform_broadcaster.h>


#define _USE_MATH_DEFINES

/*--------------------------------------------------------------------
 * MarisTFPub()
 * Constructor.
 *------------------------------------------------------------------*/
MarisTFPub::MarisTFPub()
{
} // end MarisTFPub()


/*--------------------------------------------------------------------
 * ~E80DataBridge()
 * Destructor.
 *------------------------------------------------------------------*/
MarisTFPub::~MarisTFPub()
{
} // end ~MarisTFPub()


/*--------------------------------------------------------------------
 * messageCallback()
 *------------------------------------------------------------------*/
void MarisTFPub::messageCallback(const maris_ros_msgs::TransfMatrix& transfMat){

 
  ros::Time currentTime = ros::Time::now();

  static tf::TransformBroadcaster br;
  tf::Transform transform_Child2Parent;


  tf::Vector3 origin;
  origin.setValue(static_cast<double>(transfMat.cwmatrix[12]),static_cast<double>(transfMat.cwmatrix[13]),static_cast<double>(transfMat.cwmatrix[14]));

  tf::Matrix3x3 tf3d;
  tf3d.setValue(static_cast<double>(transfMat.cwmatrix[0]), static_cast<double>(transfMat.cwmatrix[4]), static_cast<double>(transfMat.cwmatrix[8]), 
                static_cast<double>(transfMat.cwmatrix[1]), static_cast<double>(transfMat.cwmatrix[5]), static_cast<double>(transfMat.cwmatrix[9]), 
                static_cast<double>(transfMat.cwmatrix[2]), static_cast<double>(transfMat.cwmatrix[6]), static_cast<double>(transfMat.cwmatrix[10]));

  tf::Quaternion tfqt;
  tf3d.getRotation(tfqt);

  transform_Child2Parent.setOrigin( origin );
  transform_Child2Parent.setRotation( tfqt );


  br.sendTransform(tf::StampedTransform(transform_Child2Parent, currentTime, tf_parent_frame, tf_child_frame));
}

