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
#include <thread>
#include <fstream>
#include <vector>

class GradientOrientation {
public:

	GradientOrientation(int rows = 4, int cols = 4)
	: m_rows(rows), m_cols(cols), m_gridSize(m_rows * m_cols),
	m_ic(m_gridSize), m_oc(m_gridSize), m_tc(m_gridSize),
	m_x(m_gridSize), m_y(m_gridSize) {
		for (unsigned int i = 0; i < m_gridSize; ++i) {
			m_x[i] = (i % m_cols);
			m_y[i] = (i / m_rows);
		}
	}

	void run(const cv::Mat& in, cv::Mat& out, float th = 50.0) {
		using namespace cv;
		using namespace std;

		m_w = in.cols / m_cols;
		m_h = in.rows / m_rows;
		m_th = th * th;

		m_ic.clear();
		m_oc.clear();
		m_tc.clear();

		for (unsigned int i = 0; i < m_gridSize; ++i) {
			m_ic[i] = Mat(in, cv::Rect(m_x[i] * m_w, m_y[i] * m_h, m_w, m_h));
			m_oc[i] = Mat(out, cv::Rect(m_x[i] * m_w, m_y[i] * m_h, m_w, m_h));

			m_tc[i] = thread(bind(&GradientOrientation::threadProcess, this, m_ic[i], m_oc[i]));
		}

		for (unsigned int i = 0; i < m_gridSize; ++i) {
			m_tc[i].join();
		}

	}

private:

	int m_rows;
	int m_cols;
	int m_gridSize;
	int m_w;
	int m_h;

	float m_th;

	std::vector<cv::Mat> m_ic;
	std::vector<cv::Mat> m_oc;
	std::vector<std::thread> m_tc;
	std::vector<int> m_x;
	std::vector<int> m_y;

	void threadProcess(const cv::Mat& in, cv::Mat& out) {
		using namespace cv;
		using namespace std;

		Mat X, Y;

		Sobel(in, X, CV_32F, 1, 0);
		Sobel(in, Y, CV_32F, 0, 1);

		float BMag;
		float GMag;
		float RMag;
		float ori, x, y;

		for (int j = 0; j < in.rows; j++) {
			Vec3f* Xrowj = X.ptr<Vec3f>(j);
			Vec3f* Yrowj = Y.ptr<Vec3f>(j);
			//			const uint8_t* maskrowj = mask.ptr<uint8_t>(j);
			uint8_t* outrowj = out.ptr<uint8_t>(j);
			for (int i = 0; i < in.cols; i++) {
				//				if (maskrowj[i]) 
				{
					ori = 0;
					BMag = Xrowj[i][0] * Xrowj[i][0] + Yrowj[i][0] * Yrowj[i][0];
					GMag = Xrowj[i][1] * Xrowj[i][1] + Yrowj[i][1] * Yrowj[i][1];
					RMag = Xrowj[i][2] * Xrowj[i][2] + Yrowj[i][2] * Yrowj[i][2];
					if (RMag > BMag && RMag > GMag) {
						if (RMag > m_th) {
							ori = atan2(Yrowj[i][2], Xrowj[i][2]) * 180.0 / M_PI;
						}
					} else {
						if (BMag > GMag) {
							if (BMag > m_th) {
								ori = atan2(Yrowj[i][0], Xrowj[i][0]) * 180.0 / M_PI;
							}
						} else {
							if (GMag > m_th) {
								ori = atan2(Yrowj[i][1], Xrowj[i][1]) * 180.0 / M_PI;
							}
						}
					}

					outrowj[i] = static_cast<uint8_t> (static_cast<int> (180 + ori) % 180);
				}
			}
		}
	}
};

