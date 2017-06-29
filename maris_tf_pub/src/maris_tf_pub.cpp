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
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>

#include "ros/ros.h"
#include "ros/time.h"


#include <ros/ros.h>
#include "std_msgs/String.h"

#include "maris_tf_pub_core.h"

// Sincronizzazione dei messaggi
#include <message_filters/subscriber.h>
#include <boost/lambda/bind.hpp>


using namespace message_filters;

int main(int argc, char** argv){
  ros::init(argc, argv, "maris_tf_pub");

  ros::NodeHandle node;

  string trans_mat_topic;
  string tf_parent_frame;
  string tf_child_frame;


  int rate;				// ROS node cycle frequency

  MarisTFPub *mytfpub = new MarisTFPub();

  std::cout << "--- STARTING MARIS TF Publisher ---" << std::endl;


  // Initialize node parameters from launch file or command line.
  // Use a private node handle so that multiple instances of the node can
  // be run simultaneously while using different parameters.
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("trans_mat_topic",           trans_mat_topic,          string("trans_mat_topic"));
  private_node_handle_.param("rate",                      rate,                     int(50));

  private_node_handle_.param("tf_parent_frame",            tf_parent_frame,       string("tf_parent_frame"));
  private_node_handle_.param("tf_child_frame",          tf_child_frame,     string("tf_child_frame"));
  
  
 
  //Passo al mytfpub i parametri
  mytfpub->tf_parent_frame = tf_parent_frame;
  mytfpub->tf_child_frame  = tf_child_frame;

  
  // Create the subscriber.
  ros::Subscriber trans_topic_sub;
  trans_topic_sub = node.subscribe(trans_mat_topic.c_str(), 1, &MarisTFPub::messageCallback, mytfpub);	  

  


  

  

  ros::Rate r(rate);

  while (ros::ok()){

    ros::spinOnce();
    r.sleep();
  }
  
 
  return 0;
};
