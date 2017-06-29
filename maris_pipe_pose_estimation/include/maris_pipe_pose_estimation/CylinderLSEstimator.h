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
#ifndef CYLINDER_LS_ESTIMATOR_H
#define CYLINDER_LS_ESTIMATOR_H

#include <eigen3/Eigen/Dense>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

class CylinderLSEstimator {
public:
	typedef Eigen::Matrix<double, 3, 4> Matrix4x3d;

	/** Default constructor. 
	 */
	CylinderLSEstimator();

	/** Sets the radius of the cylinder to estimate.
	 */
	void setRadius(double r) {
		radius_ = r;
	}

	/** Sets the cylinder length to position the reference frame.
	 */
	void setLength(double l) {
		length_ = l;
	}

	/** Sets the nadir direction ("plumb bob" toward gravity force). 
	 */
	void setNadir(const Eigen::Vector3d& nadir) {
		nadir_ = nadir;
	}

	/** Sets the camera parameters of left camera using OpenCV types.
	 */
	void setCameraMatrixLeft(const cv::Mat& P) {
		//std::cout << "P: " << P.rows << " x " << P.cols << std::endl; 
		assert(P.rows == 3 && P.cols == 4);
		for (int r = 0; r < 3; ++r) {
			for (int c = 0; c < 4; ++c) {
				PLeft_(r, c) = P.at<double>(r, c);
			}
		}
		//	std::cout << "CameraMatrixLeft:\n" << PLeft_.matrix()<< std::endl;
	}

	/** Sets the camera parameters of left camera using OpenCV types.
	 */
	void setCameraMatrixRight(const cv::Mat& P) {
		//std::cout << "P: " << P.rows << " x " << P.cols << std::endl; 
		assert(P.rows == 3 && P.cols == 4);
		for (int r = 0; r < 3; ++r) {
			for (int c = 0; c < 4; ++c) {
				PRight_(r, c) = P.at<double>(r, c);
			}
		}
		//	std::cout << "CameraMatrixRight:\n" << PRight_.matrix()<< std::endl;
	}

	/** Sets the homogenous coordinates of line in left image:
	 *     l^T p = x * l(0) + y * l(1) + l(2) = 0
	 * The choice of line sign is not arbitrary: the cylinder must lies 
	 * in the positive half-plane:  
	 *     l^T p > 0.0
	 */
	void setLineLeft(const Eigen::Vector3d& l) {
		Eigen::Vector4d plane = PLeft_.transpose() * l;
		translatePlaneToCylinder(plane);
		planes_.push_back(plane);
	}

	/** Sets the homogenous coordinates of line in left image:
	 *     l^T p = x * cos(theta) + y * sin(theta) - rho = 0
	 * The choice of line sign is not arbitrary: the cylinder must lies 
	 * in the positive half-plane:  
	 *     l^T p > 0.0
	 */
	void setLineLeft(double theta, double rho) {
		Eigen::Vector3d l;
		l << cos(theta), sin(theta), -rho;
		setLineLeft(l);
	}

	/** Sets the homogenous coordinates of line in right image.
	 *     l^T p = x * l(0) + y * l(1) + l(2) = 0
	 * The choice of line sign is not arbitrary: the cylinder must lies 
	 * in the positive half-plane:  
	 *     l^T p > 0.0
	 */
	void setLineRight(const Eigen::Vector3d& l) {
		Eigen::Vector4d plane = PRight_.transpose() * l;
		translatePlaneToCylinder(plane);
		planes_.push_back(plane);
	}

	/** Sets the homogenous coordinates of line in right image:
	 *     l^T p = x * cos(theta) + y * sin(theta) - rho = 0
	 * The choice of line sign is not arbitrary: the cylinder must lies 
	 * in the positive half-plane:  
	 *     l^T p > 0.0
	 */
	void setLineRight(double theta, double rho) {
		Eigen::Vector3d l;
		l << cos(theta), sin(theta), -rho;
		setLineRight(l);
	}

	/** Sets the homogenous coordinates of line of the terminal part 
	 *	of the pipe in left image:
	 *     l^T p = x * l(0) + y * l(1) + l(2) = 0
	 * The choice of line sign is not arbitrary: the cylinder must lies 
	 * in the positive half-plane:  
	 *     l^T p > 0.0
	 */
	void setTerminalLineLeft(const Eigen::Vector3d& l) {
//		std::cout << "LEFT TERMINAL LINE " << l.transpose() << std::endl;
//		std::cout << "LEFT PROJECTION MATRIX\n" << PLeft_.transpose() << std::endl;
		Eigen::Vector4d plane = PLeft_.transpose() * l;
            Eigen::Vector3d nTerminal = plane.block<3,1>(0,0);
            double cTerminal = plane(3);
		double knorm = nTerminal.norm();
		if (knorm > 1e-3) { 
	            nTerminal = nTerminal / knorm;
			cTerminal = cTerminal / knorm;
			plane.block<3, 1>(0, 0) = nTerminal;
			plane(3) = cTerminal;
//			std::cout << "***********\nLEFT TERMINAL " << plane.transpose() << std::endl;
		}
		translatePlaneTerminalToCylinder(plane);
		terminalPlanes_.push_back(plane);
	}

	/** Sets the homogenous coordinates of line of the terminal part 
	 *	of the pipe in left image:
	 *     l^T p = x * cos(theta) + y * sin(theta) - rho = 0
	 * The choice of line sign is not arbitrary: the cylinder must lies 
	 * in the positive half-plane:  
	 *     l^T p > 0.0
	 */
	void setTerminalLineLeft(double theta, double rho) {
		Eigen::Vector3d line2d;
		line2d << cos(theta), sin(theta), -rho;
		setTerminalLineLeft(line2d);
	}

	/** Sets the homogenous coordinates of line of the terminal part 
	 *	of the pipe in right image.
	 *     l^T p = x * l(0) + y * l(1) + l(2) = 0
	 * The choice of line sign is not arbitrary: the cylinder must lies 
	 * in the positive half-plane:  
	 *     l^T p > 0.0
	 */
	void setTerminalLineRight(const Eigen::Vector3d& l) {
//		std::cout << "RIGHT TERMINAL LINE " << l.transpose() << std::endl;
//		std::cout << "RIGHT PROJECTION MATRIX\n" << PRight_.transpose() << std::endl;
		Eigen::Vector4d plane = PRight_.transpose() * l;
		Eigen::Vector3d nTerminal = plane.block<3,1>(0,0);
            double cTerminal = plane(3);
		double knorm = nTerminal.norm();
//		std::cout << "***********\nRIGHT TERMINAL\n before normalization " << plane.transpose() << std::endl;
		if (knorm > 1e-3) { 
	            nTerminal = nTerminal / knorm;
			cTerminal = cTerminal / knorm;
			plane.block<3, 1>(0, 0) = nTerminal;
			plane(3) = cTerminal;
		}
//		std::cout << " line " << l.transpose() << "\n plane " << plane.transpose() << std::endl;
		translatePlaneTerminalToCylinder(plane);
		terminalPlanes_.push_back(plane);
	}

	/** Sets the homogenous coordinates of line of the terminal part 
	 *	of the pipe in right image:
	 *     l^T p = x * cos(theta) + y * sin(theta) - rho = 0
	 * The choice of line sign is not arbitrary: the cylinder must lies 
	 * in the positive half-plane:  
	 *     l^T p > 0.0
	 */
	void setTerminalLineRight(double theta, double rho) {
		Eigen::Vector3d line2d;
		line2d << cos(theta), sin(theta), -rho;
		setTerminalLineRight(line2d);
	}

	/** Set the terminal lines flag to false
	 * 
	 */
	void resetTerminalLines() {
		terminalLeft_ = false;
		terminalRight_ = false;
		terminalPlanes_.clear();
	}

	/** Reset all the lines vectors
	 * 
	 */
	void resetLines() {
		planes_.clear();
	}


	/** Computes the principal axis as a line with equation:
	 *      p(t) = p0 + d * t   
	 * (p0 is arbitrary, d is normalized).
	 */
	void computeCylinderAxis(Eigen::Vector3d& d, Eigen::Vector3d& p0);

	/** Computes reference frame on cylinder according to 
	 */
	bool computeReferenceFrame(Eigen::Matrix4d& transf, Eigen::Vector3d& vy, Eigen::Vector3d& pos);

	/**
	 *  Enable the frame by frame tracking system
	 */
	void enableTracking(bool tracking) {
		tracking_ = tracking;
	}

private:
	// Matrices for left/right cameras descriptions
	Matrix4x3d PLeft_;
	Matrix4x3d PRight_;
	// Planes given by lines
	std::vector<Eigen::Vector4d> planes_;
	std::vector<Eigen::Vector4d> terminalPlanes_;


	Eigen::Vector3d line2DLeft_;
	Eigen::Vector3d line2DRight_;
	//	Eigen::Vector3d terminalLine2DLeft_;
	//	Eigen::Vector3d terminalLine2DRight_;
	double radius_;
	double length_;
	Eigen::Vector3d nadir_;
	bool terminalRight_;
	bool terminalLeft_;

	Eigen::Vector3d tempDirectionVector_;
	bool tracking_;


	/** The planes of left and right segments are tangent to the cylinder 
	 * with given radius_. 
	 * The parallel plane can be obtained by translating both planes 
	 * of radius_ in (normalized) norm n direction:
	 * n.transpose() * (p - r * n) + c = 0
	 * n.transpose() * p + (c - n.transpose() * n * r) = 0
	 * n.transpose() * p + (c - r) = 0   since n.transpose()*n = 1!
	 * Thus, the new c' = c - r as performed below. 
	 */
	void translatePlaneToCylinder(Eigen::Vector4d& plane);

	/** The terminal planes are translated toward the center of the cylinder
	 * given the length (height?) of the cylinder. 
	 * The parallel plane can be obtained by translating both planes 
	 * of 0.5*length_ in (normalized) norm n direction:
	 * n.transpose() * (p + 0.5*l * n) + c = 0
	 *   ... (see above)
	 * n.transpose() * p + (c + 0.5*l) + c = 0
	 */
	void translatePlaneTerminalToCylinder(Eigen::Vector4d& plane);
};

#endif
