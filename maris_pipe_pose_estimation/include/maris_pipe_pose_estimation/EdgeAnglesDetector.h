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
#pragma once

#include "LS.h"
#include "Timer.h"
#include "Profiler.h"
#include "GradientOrientation.h"
#include <algorithm>

/// Function prototype for DetectEdgesByED exported by EDLinesLib.a
LS *DetectLinesByED(unsigned char *srcImg, int width, int height, int *pNoLines);

static bool pairCompare(const std::pair<double, int>& firstElem, const std::pair<double, int>& secondElem) {
	return firstElem.first > secondElem.first;
}

class EdgeAnglesDetector {
public:

	EdgeAnglesDetector(int width, int height)
	: m_grad(height, width, CV_8UC1, cv::Scalar(0)),
	m_go(4, 4) {};

	void getAngles(const cv::Mat& in, std::vector<float>& angles, float gradientTh) {
		using namespace std;
		using namespace cv;
		angles.clear();
		angles.resize(180, 0.0);

//		g_prof.run("gradientOr");
//				gradientOrientation(in, m_grad, 50.0);
		m_go.run(in, m_grad, gradientTh);
//		g_prof.stop("gradientOr");
//		imwrite("/media/d/Presentations/secondo_anno/images/maris/grad.png", m_grad);
		cv::Mat edl(m_grad.rows, m_grad.cols, CV_8UC1, Scalar(0));
//				imshow("m_grad", m_grad);
//				waitKey(5000000);

//		g_prof.run("EdLines");
		LS* l = DetectLinesByED(m_grad.data, m_grad.cols, m_grad.rows, &m_numLines);
		for (int i = 0; i < m_numLines; ++i){
			cv::line(edl, cv::Point(l[i].sx, l[i].sy), cv::Point(l[i].ex, l[i].ey), Scalar(255));
		}
//		imwrite("/media/d/Presentations/secondo_anno/images/maris/edl.png", edl);
//		std::vector<LS> l;
//		m_edl.process(m_grad, l);
//		m_numLines = l.size();
//		g_prof.stop("EdLines");

//		m_squaredMax = findSquaredMax(l);

		vector<pair<float, int> > indexedAngles(180, make_pair(0, 0));
		vector<pair<float, int> > indexedAnglesSmooth(180, make_pair(0, 0));
		
		for (int i = 0; i < 180; ++i) {
			indexedAngles[i].second = i;
			indexedAnglesSmooth[i].second = i;
		}

		for (int i = 0; i < m_numLines; ++i) {
//			DEB(i);
			LS ln = l[i];
			int index = 0;
			if (abs(ln.ex - ln.sx) < 0.001) {
				index = 0;
			} else {
				index = std::round(90 + 180 / M_PI * std::atan((ln.ey - ln.sy) / (ln.ex - ln.sx)));
			}
//			DEB(index);
			indexedAngles[index].first += (segmentSquaredLenght(ln));// / m_squaredMax);
		}
		indexedAnglesSmooth[0].first = indexedAngles[0].first;
		for(int i = 1; i < indexedAngles.size()-1; ++i){
			indexedAnglesSmooth[i].first = (indexedAngles[i-1].first + indexedAngles[i].first + indexedAngles[i + 1].first) / 3.0;
		}
		indexedAnglesSmooth[indexedAngles.size()-1].first = indexedAngles[indexedAngles.size()-1].first;
		
		std::sort(indexedAngles.begin(), indexedAngles.end(), pairCompare);

		for (int i = 0; i < 180; ++i) {
			angles[i] = indexedAngles[i].second;
		}
	}
private:
	cv::Mat m_grad;
	int m_numLines;
	float m_squaredMax;
	GradientOrientation m_go;

	Timer m_tm;
	//	cv::Mat blur, X, Y
	
	void gradientOrientation(const cv::Mat& in, cv::Mat& out, float th) {
		using namespace cv;
		using namespace std;
		
		Mat X, Y;
		
		Sobel(in, X, CV_32F, 1, 0);
		Sobel(in, Y, CV_32F, 0, 1);
		
		float BMag;
		float GMag;
		float RMag;
		float ori, x, y;
		
		th *= th;

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
						if (RMag > th) {
							ori = atan2(Yrowj[i][2], Xrowj[i][2]) * 180.0 / M_PI;
						}
					} else {
						if (BMag > GMag) {
							if (BMag > th) {
								ori = atan2(Yrowj[i][0], Xrowj[i][0]) * 180.0 / M_PI;
							}
						} else {
							if (GMag > th) {
								ori = atan2(Yrowj[i][1], Xrowj[i][1]) * 180.0 / M_PI;
							}
						}
					}

					outrowj[i] = static_cast<uint8_t> (static_cast<int> (180 + ori) % 180);
				}
			}
		}
	}

	float findSquaredMax(LS* l) {
		float max = 0;
		for (int i = 0; i < m_numLines; ++i) {
			float len = segmentSquaredLenght(l[i]);
			if (max < len) {
				max = len;
			}
		}
		return max;
	}

//	float findSquaredMax(std::vector<LS>& l) {
//		float max = 0;
//		for (int i = 0; i < m_numLines; ++i) {
//			float len = segmentSquaredLenght(l[i]);
//			if (max < len) {
//				max = len;
//			}
//		}
//		return max;
//	}

	float segmentSquaredLenght(const LS& l) {
		return ((l.ey - l.sy)*(l.ey - l.sy) + (l.ex - l.sx)*(l.ex - l.sx));
	}

};
