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

#include <opencv2/imgproc/imgproc.hpp>
#include <eigen3/Eigen/Dense>

class LineRansac {
public:

	LineRansac(int minData = 2, int minForModel = 10, int numIteration = 50, double modelTh = 0.5) :
	m_minData(minData), m_minForModel(minForModel), m_numIteration(numIteration), m_modelTh(modelTh*modelTh) {
		srand(time(0));
		//		srand(0);
	}

	void getBestLineRansac(const cv::Mat& in, Eigen::Vector3d& model) {
		using namespace cv;
		using namespace std;
		//		Mat t = in.clone();
		m_data.clear();

		std::vector<Eigen::Vector3d> bestConsensus;
		m_whitePixel = countNonZero(in);
		if (m_whitePixel > m_minData) {
			m_data.resize(m_whitePixel);

			int count = 0;
			for (int j = 0; j < in.rows; j++) {
				const uint8_t* rowj = in.ptr<uint8_t>(j);
				for (int i = 0; i < in.cols; i++) {
					if (rowj[i] > 0) {
						m_data[count][0] = i;
						m_data[count][1] = j;
						m_data[count++][2] = 1;
					}
				}
			}

			double error = Ransac(m_data, m_minData, m_minForModel, m_numIteration, bestConsensus, model, m_modelTh);
		}

	}
private:
	int m_minData;
	int m_minForModel;
	int m_numIteration;
	double m_modelTh;
	//	Eigen::Vector3d m_bestModel;

	int m_whitePixel;
	std::vector<Eigen::Vector3d> m_data;

	void generateLine1(const std::vector<Eigen::Vector3d>& cons, Eigen::Vector3d& model) {
		using namespace cv;
		using namespace std;
		std::vector<Point> points;

		points.resize(cons.size());

		for (int i = 0; i < cons.size(); i++) {
			points[i].x = cons[i][0];
			points[i].y = cons[i][1];
		}

		Vec4f l;
		fitLine(points, l, CV_DIST_HUBER, 0, 0.01, 0.01);

		double theta = atan2(l[1], l[0]) - M_PI / 2.0;
		double rho = l[2] * cos(theta) + l[3] * sin(theta);

		model[0] = cos(theta);
		model[1] = sin(theta);
		model[2] = -rho;
	}

	void generateLine2(const std::vector<Eigen::Vector3d >& cons, Eigen::Vector3d& model) {
		// Least square line. We pick the minimum (rho,theta) that minimizes error:
		// \sum_i (x_i * cos(theta) + y_i * sin(theta) - rho)^2
		double sxx = 0.0;
		double syy = 0.0;
		double sxy = 0.0;
		double sx = 0.0;
		double sy = 0.0;
		int num = 0;
		for (unsigned int i = 0; i < cons.size(); ++i) {
			sxx += cons[i](0) * cons[i](0);
			syy += cons[i](1) * cons[i](1);
			sxy += cons[i](0) * cons[i](1);
			sx += cons[i](0);
			sy += cons[i](1);
			++num;
		}

		double msxx = (sxx - sx * sx / num) / num;
		double msyy = (syy - sy * sy / num) / num;
		double msxy = (sxy - sx * sy / num) / num;
		double b = 2.0 * msxy;
		double a = msxx;
		double c = msyy;
		double theta = 0.5 * (atan2(b, a - c) + M_PI);
		theta = atan2(sin(theta), cos(theta));
		double rho = (sx * cos(theta) + sy * sin(theta)) / num;

		if (rho < 0) {
			theta = atan2(sin(theta + M_PI), cos(theta + M_PI));
			rho = -rho;
		}
		model(0) = cos(theta);
		model(1) = sin(theta);
		model(2) = -rho;
	}

	void checkData(std::vector<Eigen::Vector3d>& data, std::vector<Eigen::Vector3d>& consensus, Eigen::Vector3d& model, double modelTh) {
		double a = model(0);
		double b = model(1);
		double c = model(2);
		double den = a * a + b * b;
		for (unsigned int i = 0; i < data.size(); ++i) {
			Eigen::Vector3d& p = data[i];
			double num = std::pow(a * p(0) + b * p(1) + c, 2);
			if ((num / den) < modelTh) {
				consensus.push_back(p);
			}
		}
	}

	template<class fwditer>
	fwditer random_unique(fwditer begin, fwditer end, size_t num_random) {
		size_t left = std::distance(begin, end);
		while (num_random--) {
			fwditer r = begin;
			std::advance(r, rand() % left);
			std::swap(*begin, *r);
			++begin;
			--left;
		}
		return begin;
	}

	double Ransac(std::vector<Eigen::Vector3d>& data, int minData, int minForModel, int numIteration, std::vector<Eigen::Vector3d>& bestConsensus, Eigen::Vector3d& bestModel, double modelTh) {
		bestConsensus.clear();
		double error = 0.0;
		std::vector<Eigen::Vector3d> consensus;
		consensus.reserve(m_whitePixel);
		Eigen::Vector3d possibleModelLine;
		for (int iter = 0; iter < numIteration; ++iter) {
			random_unique(data.begin(), data.end(), minData);

			for (unsigned int i = 0; i < minData; ++i) {
				consensus.push_back(data[i]);
			}

			generateLine1(consensus, possibleModelLine);

			checkData(data, consensus, possibleModelLine, modelTh);

			if (consensus.size() > minForModel && consensus.size() > bestConsensus.size()) {
				bestConsensus = consensus;
			}
		}

		generateLine1(bestConsensus, bestModel);
		//		std::cout << "consensus line: " << bestConsensus.size() << "  data: " << data.size() << std::endl;
		//		std::cout << "consensus line %: " << double(bestConsensus.size()*100) / data.size() << std::endl;
		error = 1 - (double(bestConsensus.size()) / data.size());
		return error;
	}
};

