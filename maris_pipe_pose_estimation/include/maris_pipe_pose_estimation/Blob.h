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
#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class Blob{
public:
	cv::Point m_centroid;
	cv::Moments m_moments;
	std::vector<cv::Point> m_hull;
	std::vector<cv::Point> m_contour;
	double m_angle;
	cv::Mat m_image;
	
	void computeInfoFromMoments(){
		cv::Moments& m = m_moments;
		m_angle = 0.5 * std::atan2(2 * m.mu11, (m.mu20 - m.mu02));
		m_centroid.x = m.m10 / m.m00;
		m_centroid.y = m.m01 / m.m00;
	}
};
