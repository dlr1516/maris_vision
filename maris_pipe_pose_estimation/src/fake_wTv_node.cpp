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

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include <boost/shared_ptr.hpp>

#include <maris_ros_msgs/VehiclePosition.h>

#include <eigen3/Eigen/Dense>

#define DEB(X)\
std::cout << #X << "=\t" << X << std::endl

class FakewTvNode {
public:

	FakewTvNode(ros::NodeHandle& nh) : _nh(nh) {
		
		_fakewTvPub = _nh.advertise<maris_ros_msgs::VehiclePosition>("/vehicle/wTv", 1);
	}

	void run() {
		maris_ros_msgs::VehiclePosition vp;
		maris_ros_msgs::TransfMatrix m;
		Eigen::Affine3d em = Eigen::Affine3d::Identity();
		Eigen::AngleAxisd aa(M_PI/2.0 , Eigen::Vector3d(1, 0, 0));
		em.prerotate(aa);
		for (int i = 0; i < 16; ++i){
			m.cwmatrix.elems[i] = em.data()[i];
			
		}
		
		DEB(em.data()[8]);
		DEB(em.data()[9]);
		DEB(em.data()[10]);
		DEB(m.cwmatrix.elems[8]);
		DEB(m.cwmatrix.elems[9]);
		DEB(m.cwmatrix.elems[10]);
		DEB("------------------");
		
		vp.validTranslation = true;
		vp.wTv = m;
//		DEB("Publishing");
		_fakewTvPub.publish(vp);
	}
private:
	ros::NodeHandle _nh;
	ros::Publisher _fakewTvPub;
};

int kbhit(void) {
	struct termios oldt, newt;
	int ch;
	int oldf;

	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

	ch = getchar();

	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	fcntl(STDIN_FILENO, F_SETFL, oldf);

	if (ch != EOF) {
		ungetc(ch, stdin);
		return 1;
	}

	return 0;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "fake_wTv_node");
	ros::NodeHandle nh_("~");
	FakewTvNode fakeNode(nh_);
	while (ros::ok()) {
		fakeNode.run();
	}
}
