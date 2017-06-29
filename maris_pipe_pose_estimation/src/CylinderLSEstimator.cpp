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
#include <maris_pipe_pose_estimation/CylinderLSEstimator.h>

CylinderLSEstimator::CylinderLSEstimator() {
	//  mKRLeft_ = Eigen::Matrix3d::Identity(); 
	//  mKtLeft_ = Eigen::Vector3d::Zero(); 
	//  mKRRight_ = Eigen::Matrix3d::Identity(); 
	//  mKtRight_ = Eigen::Vector3d::Zero();

	radius_ = 0.0;
	length_ = 0.0;
	terminalLeft_ = false;
	terminalRight_ = false;
	//	directionFound_ = false;
	tempDirectionVector_ = Eigen::Vector3d(0, 0, 0);
}

void CylinderLSEstimator::computeCylinderAxis(Eigen::Vector3d& d, Eigen::Vector3d& p0) {
	Eigen::MatrixXd A; // matrix of plane normals
	Eigen::VectorXd b; // vector of plane known terms

	// It there are only two planes with norm n1 and n2, the cylinder axis 
	// is estimated as
	//    d = n1.cross(n2)
	// Otherwise a general Least-square optimization is required.
	// Let N be the matrix whose rows are the normal vectors of planes 
	// though the cylinder axis (with estimation error, of course!):
	//        [ n_1^T ]
	//    A = [ ...   ]    (matrix size: m x 3)
	//        [ n_m^T ]
	//        [ 0     ]    <- row reserved for terminal planes (no effect on d!):
      //        [ ...   ]       there are max(t,1) zero rows
      //        [ 0     ]
	// The cylinder axis direction d is estimated as the UNIT vector that 
	// satisfies homogeneous linear equation: 
	//    A * d = 0
	// which is
	//    min d' * (A'*A) * d
	//    s.t. d' * d = 1
	// Although the unit vector constraint is non-linear, the solution to
	// such problem is well known: it is the right-singular vector of N 
	// corresponding to the smallest singular value (or, equivalently,
	// the eigenvector corresponding to the smallest eigenvalue of A'*A).
	int m = planes_.size();
	int t = terminalPlanes_.size();
	A = Eigen::MatrixXd::Zero(m+std::max(t,1), 3);
	b = Eigen::VectorXd::Zero(m+std::max(t,1));

	if (m == 2) {
		A.block<1, 3>(0, 0) = planes_[0].block<3, 1>(0, 0).transpose();
		A.block<1, 3>(1, 0) = planes_[1].block<3, 1>(0, 0).transpose();
		b(0) = -planes_[0](3, 0);
		b(1) = -planes_[1](3, 0);
		d = planes_[0].block<3, 1>(0, 0).cross(planes_[1].block<3, 1>(0, 0));
	} else if (m > 2) {
		for (int i = 0; i < m; ++i) {
			A.block<1, 3>(i, 0) = planes_[i].block<3, 1>(0, 0).transpose();
			b(i) = -planes_[i](3, 0);
		}
		Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes;
		saes.compute(A.transpose() * A);
		d = saes.eigenvectors().col(0);
	} else {
		std::cerr << "Error: line estimation requires at least 2 planes!" << std::endl;
		return;
	}
	d = d / d.norm();


	if (tracking_) {
            if (d[0]>0 ){
                d = -d;
            }
//		if (tempDirectionVector_.dot(-d) > tempDirectionVector_.dot(d)) {
//			d = -d;
//		}
//		tempDirectionVector_ = d;
	}
      
      // If there is at least a terminal plane, it uses it for intersection.
      // Otherwise set a conventional plane through the camera origin
	if (t > 0) {
		for (int i = 0; i < t; ++i) {
			A.block<1, 3>(m + i, 0) = terminalPlanes_[i].block<3, 1>(0, 0).transpose();
			b(m+i) = -terminalPlanes_[i](3, 0);
		}
//		std::cout << "\nterminal plane " << terminalPlanes_[0].transpose() << std::endl;
//            std::cout << "  normal multiplication: " << terminalPlanes_[0].block<3, 1>(0, 0).dot(d) << std::endl;
//		std::cout << std::endl;
      }
	else {
		A.block<1, 3>(m, 0) << 1, 0, 0;
		b(m) = 0.0;
	}

	// A point p0 on the line can be found by intersecting 
	// 1. all the planes through the cylinder axes
	// 2. the terminal planes
	// We re-use the matrix A by adding the row corresponding to 
	// terminal plane.
//	A.block<1, 3>(m, 0) = nTerminal.transpose();
//	b(m) = -cTerminal;
//	std::cout << "A:\n" << A.matrix() << std::endl;
//	std::cout << "b:\n" << b.matrix() << std::endl;
	p0 = A.fullPivLu().solve(b);
//      std::cout << "p0: " << p0.transpose() << std::endl;
//	std::cout << std::endl;

//	if (sign >= 0.0) {
//		p0 = p0 + d * length_ / 2;
//	} else {
//		p0 = p0 - d * length_ / 2;
//	}

	//	std::cout << "sign: " << sign << std::endl;
}

bool CylinderLSEstimator::computeReferenceFrame(Eigen::Matrix4d& transf, Eigen::Vector3d& vy, Eigen::Vector3d& pos) {
	Eigen::Vector3d vx;
//	Eigen::Vector3d vy;
	Eigen::Vector3d vz;
//	Eigen::Vector3d pos;
	//	bool found = false;

	// The axis y is along the cylinder axis
	computeCylinderAxis(vy, pos);

	// If the left or right pipe terminal is found, then the reference frame 
	// origin pos can be placed on the center of the tube.
	if (std::abs(vy.norm()) < 1e-6) {
		return false;
	}
	vy = vy / vy.norm();

	if (terminalPlanes_.size() == 0) {
		return false;
	}

	// The axis z is the orthonormalized nadir_
	vz = nadir_ - nadir_.dot(vy) * vy;

	// The axis x is given by the cross product of vy and vz
	vx = vy.cross(vz);

	// The transformation matrix
	transf = Eigen::Matrix4d::Zero();
	transf.block<3, 1>(0, 0) = vx;
	transf.block<3, 1>(0, 1) = vy;
	transf.block<3, 1>(0, 2) = vz;
	transf.block<3, 1>(0, 3) = pos;
	transf(3, 3) = 1.0;

	return true;
}

// The planes of left and right segments are tangent to the cylinder 
// with given radius_. 
// The parallel plane can be obtained by translating both planes 
// of radius_ in (normalized) norm n direction:
//   n.transpose() * (p - r * n) + c = 0
//   n.transpose() * p + (c - n.transpose() * n * r) = 0
//   n.transpose() * p + (c - r) = 0   since n.transpose()*n = 1!
// Thus, the new c' = c - r as performed below. 

void CylinderLSEstimator::translatePlaneToCylinder(Eigen::Vector4d& plane) {
	Eigen::Vector3d n = plane.block<3, 1>(0, 0);
	double c = plane(3, 0);
	double l = n.norm();
	if (std::abs(l) < 1e-5) {
		return;
	}
	n = n / l;
	c = c / l - radius_;
	plane.block<3, 1>(0, 0) = n;
	plane(3, 0) = c;
}

void CylinderLSEstimator::translatePlaneTerminalToCylinder(Eigen::Vector4d& plane) {
	Eigen::Vector3d n = plane.block<3, 1>(0, 0);
	double c = plane(3, 0);
	double l = n.norm();
	if (std::abs(l) < 1e-5) {
		return;
	}
	n = n / l;
	c = c / l - 0.5*length_;
	plane.block<3, 1>(0, 0) = n;
	plane(3, 0) = c;
}


