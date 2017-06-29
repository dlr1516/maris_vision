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

#include "CVUtils.h"
#include "EdgeAnglesDetector.h"
#include "Timer.h"
#include "Profiler.h"
#include "LineRansac.h"
#include "LineFitting.h"
#include "Blob.h"

#include <thread>

#define DEB(X)\
std::cout << #X << ":\t" << X << std::endl;

class MarisPipeEdgeDetector {
public:

	MarisPipeEdgeDetector(int width = 0, int height = 0, int downsampling = 1)
	: m_cols(width), m_rows(height), m_diag(std::sqrt(m_cols*m_cols + m_rows*m_rows)),
	//	m_hsvBoostMask(m_rows, m_cols, CV_8UC1, cv::Scalar(0)),
	//	m_limit(m_rows, m_cols, CV_8UC1, cv::Scalar(0)),
	m_downsampling(downsampling),
	m_square15(15, 15, CV_8UC1, cv::Scalar(1)),
	m_square7(7, 7, CV_8UC1, cv::Scalar(1)),
	m_square3(3, 3, CV_8UC1, cv::Scalar(1)),
	m_square5(5, 5, CV_8UC1, cv::Scalar(1)),
	m_square25(25, 25, CV_8UC1, cv::Scalar(1)),
	m_square50(50, 50, CV_8UC1, cv::Scalar(1)),
	m_horLine50(1, 50, CV_8UC1, cv::Scalar(1)),
	m_horLine25(1, 25, CV_8UC1, cv::Scalar(1)),
	m_verLine25(25, 1, CV_8UC1, cv::Scalar(1)),
	m_utils(m_cols, m_rows), /*m_bcd(boostFilename),*/
	m_useArmMask(false),
	m_eadL(m_cols, m_rows),
	m_eadR(m_cols, m_rows) {
		m_terminalHistMaxL = -1;
		m_terminalHistMaxR = -1;
	};

	SETTER(setMinMaskSize, m_minMaskSize);
	SETTER2(setPipeColor, m_lowColor, m_highColor);
	SETTER(setBlurSize, m_blurSize);
	SETTER(setBlurDev, m_blurDev);
	SETTER(setMaskThreshold, m_maskTh);
	SETTER(setAngleDistThreshold, m_angleDistTh);
	SETTER(setAngleDistBreak, m_angleDistBreak);
	SETTER(setEdgeAngleDetectorThreshold, m_edgeAngleDetectorTh);
	SETTER(setEdgeMapThreshold, m_edgeMapTh);
	SETTER(setTerminalHistThreshold, m_terminalHistTh);
	SETTER(setEdgeHistDeviation, m_edgeHistDev);
	SETTER(setColorCorrection, m_colorCorrection);

	bool isTerminalOnLeft() {
		return m_terminalOnLeft;
	}

	bool isTerminalFound() {
		return m_terminalFound;
	}

	bool isEdgeLUFound() {
		return m_edgeXLUFound;
	}

	bool isEdgeRUFound() {
		return m_edgeXRUFound;
	}

	bool isEdgeLDFound() {
		return m_edgeXLDFound;
	}

	bool isEdgeRDFound() {
		return m_edgeXRDFound;
	}

	bool isTerminalLLFound() {
		return m_edgeYLLFound;
	}

	bool isTerminalLRFound() {
		return m_edgeYLRFound;
	}

	bool isTerminalRLFound() {
		return m_edgeYRLFound;
	}

	bool isTerminalRRFound() {
		return m_edgeYRRFound;
	}

	cv::Point2d getLeftMaskCentroid() {
		return m_centrL;
	}

	cv::Point2d getRightMaskCentroid() {
		return m_centrR;
	}

	double getLeftMaskRotation() {
		return m_rotAngleL * M_PI / 180.0;
	}

	double getRightMaskRotation() {
		return m_rotAngleR * M_PI / 180.0;
	}

	void setArmMasks(const cv::Mat& armL, const cv::Mat& armR) {
		m_armMaskL = armL;
		m_armMaskR = armR;
		m_useArmMask = true;
	}

	bool findEdgeLines(cv::Mat& inL, cv::Mat& inR) {
		using namespace std;
		using namespace cv;
		//		Mat inL, inR;
		int imW = inL.cols;
		int imH = inL.rows;
		int imWd = inL.cols / m_downsampling;
		int imHd = inL.rows / m_downsampling;

		m_result = true;
		////////////PARALLEL
		if (m_downsampling > 1) {
			//			thread resL(bind(cv::resize, bgrL, inL, Size(imWd, imHd), 0, 0, INTER_LINEAR));
			resize(inL, m_bgrL, Size(imWd, imHd));
			resize(inR, m_bgrR, Size(imWd, imHd));
			if (m_useArmMask){
				resize(m_armMaskL, m_armMaskL, Size(imWd, imHd));
				resize(m_armMaskR, m_armMaskR, Size(imWd, imHd));
			}
		} else {
			m_bgrL = inL;
			m_bgrR = inR;
		}
		//
		//				imwrite("/media/d/Presentations/secondo_anno/images/maris/real.png", m_bgrL);

		if (m_colorCorrection) {
			m_utils.grayWorldColorCorrection(m_bgrL);
			m_utils.grayWorldColorCorrection(m_bgrR);
		}

		//				imwrite("/media/d/Presentations/secondo_anno/images/maris/greyworld.png", m_bgrL);

		cvtColor(m_bgrL, m_hsvL, CV_BGR2HSV_FULL);
		cvtColor(m_bgrR, m_hsvR, CV_BGR2HSV_FULL);

		bool validMaskL = findHSVBlobMask(m_hsvL, m_maskL, m_blobsL, "_L");
		bool validMaskR = findHSVBlobMask(m_hsvR, m_maskR, m_blobsR, "_R");

		//				imwrite("/media/d/Presentations/secondo_anno/images/maris/mask.png", m_maskL);
		//		SHOW(m_maskL, m_maskR);

		/////////////////////


		/////////////SERIAL
		if (!validMaskL && !validMaskR) {
			m_result = false;
			if (m_downsampling > 1) {
				resize(m_bgrL, m_bgrL, Size(imW, imH), INTER_LANCZOS4);
				resize(m_bgrR, m_bgrR, Size(imW, imH), INTER_LANCZOS4);
			}
			return false;
		}

		//		m_initialAngleGuessL = getRotationFromMoments(m_maskL);
		//		m_initialAngleGuessR = getRotationFromMoments(m_maskR);
		//		
		//		if (m_initialAngleGuessL * m_initialAngleGuessR < 0) {
		//			m_initialAngleGuessR = m_initialAngleGuessR + 180;
		//		}
		//				SHOW(m_maskL, m_maskR);
		////////////////////

		////////////PARALLEL
		//		getRotationAngle(m_eadL, m_bgrL, m_rotAngleL, m_initialAngleGuessL);
		//		getRotationAngle(m_eadR, m_bgrR, m_rotAngleR, m_initialAngleGuessR);

		Mat bgrLtmp(m_bgrL.rows, m_bgrL.cols, CV_8UC3, Scalar(0, 0, 0));
		Mat bgrRtmp(m_bgrR.rows, m_bgrR.cols, CV_8UC3, Scalar(0, 0, 0));
		m_bgrL.copyTo(bgrLtmp, m_maskL);
		m_bgrR.copyTo(bgrRtmp, m_maskR);

		thread angleTL(bind(&MarisPipeEdgeDetector::getTargetBlobIndex, this, ref(m_eadL), ref(bgrLtmp), ref(m_rotAngleL), ref(m_blobIndexL), ref(m_blobsL)));
		thread angleTR(bind(&MarisPipeEdgeDetector::getTargetBlobIndex, this, ref(m_eadR), ref(bgrRtmp), ref(m_rotAngleR), ref(m_blobIndexR), ref(m_blobsR)));
		angleTL.join();
		angleTR.join();

		if (m_blobIndexL == -1 && m_blobIndexR == -1) {
			m_result = false;
			if (m_downsampling > 1) {
				resize(m_bgrL, m_bgrL, Size(imW, imH), INTER_LANCZOS4);
				resize(m_bgrR, m_bgrR, Size(imW, imH), INTER_LANCZOS4);
			}
			return false;
		}


		if (m_blobIndexL > -1) {
			checkBlobMask(m_maskL, m_blobIndexL, m_blobsL);
		} else {
			m_maskL = cv::Scalar(0);
		}

		if (m_blobIndexR > -1) {
			checkBlobMask(m_maskR, m_blobIndexR, m_blobsR);
		} else {
			m_maskR = cv::Scalar(0);
		}


		m_utils.rotate2(m_bgrL, m_rotAngleL, m_bgrRotL);
		m_utils.rotate2(m_maskL, m_rotAngleL, m_maskRotL);
		m_utils.rotate2(m_hsvL, m_rotAngleL, m_hsvRotL);

		m_utils.rotate2(m_bgrR, m_rotAngleR, m_bgrRotR);
		m_utils.rotate2(m_maskR, m_rotAngleR, m_maskRotR);
		m_utils.rotate2(m_hsvR, m_rotAngleR, m_hsvRotR);

		if (m_useArmMask){
			m_utils.rotate2(m_armMaskL, m_rotAngleL, m_armMaskRotL);
			m_utils.rotate2(m_armMaskR, m_rotAngleR, m_armMaskRotR);
//			SHOWP(m_armMaskRotL, m_armMaskRotR);
		}

		//				imwrite("/media/d/Presentations/secondo_anno/images/maris/rot.png", m_bgrRotL);
		//		SHOW(m_bgrRotL, m_bgrRotR);

		inRange(m_hsvRotL, Scalar(0, 0, 0), Scalar(255, 0, 0), m_limitRotL);
		morphologyEx(m_limitRotL, m_limitRotL, MORPH_DILATE, m_square15);

		inRange(m_hsvRotR, Scalar(0, 0, 0), Scalar(255, 0, 0), m_limitRotR);
		morphologyEx(m_limitRotR, m_limitRotR, MORPH_DILATE, m_square15);

		//		findEdgeMap(m_bgrRotL, m_edgeRotXL, m_maskRotL, m_limitRotL);
		//		findEdgeMap(m_bgrRotR, m_edgeRotXR, m_maskRotR, m_limitRotR);

		cv::Moments momL = cv::moments(m_maskRotL, true);
		cv::Moments momR = cv::moments(m_maskRotR, true);
		m_centrL = cv::Point2d(momL.m10 / momL.m00, momL.m01 / momL.m00);
		m_centrR = cv::Point2d(momR.m10 / momR.m00, momR.m01 / momR.m00);

		thread edgeTL(bind(&MarisPipeEdgeDetector::findEdgeMap, this, ref(m_bgrRotL), ref(m_edgeRotXL), ref(m_maskRotL), ref(m_limitRotL), ref(m_centrL)));
		thread edgeTR(bind(&MarisPipeEdgeDetector::findEdgeMap, this, ref(m_bgrRotR), ref(m_edgeRotXR), ref(m_maskRotR), ref(m_limitRotR), ref(m_centrR)));



		thread terTL(bind(&MarisPipeEdgeDetector::findTerminalMap, this, ref(m_bgrRotL), ref(m_edgeRotYLL), ref(m_edgeRotYLR), ref(m_maskRotL), ref(m_armMaskRotL), ref(m_limitRotL), ref(m_edgeYLLFound), ref(m_edgeYLRFound)));
		thread terTR(bind(&MarisPipeEdgeDetector::findTerminalMap, this, ref(m_bgrRotR), ref(m_edgeRotYRL), ref(m_edgeRotYRR), ref(m_maskRotR), ref(m_armMaskRotR), ref(m_limitRotR), ref(m_edgeYRLFound), ref(m_edgeYRRFound)));

		edgeTL.join();
		edgeTR.join();
		terTL.join();
		terTR.join();
		//		SHOW(m_edgeRotYLL, m_edgeRotYLR);
		//		SHOWP(m_edgeRotYRL, m_edgeRotYRR);
		//		m_result = false;
		//		return false;

		//		findTerminalMap(m_bgrRotL, m_edgeRotYL, m_maskRotL, m_limitRotL, m_terminalHistMaxL);
		//		findTerminalMap(m_bgrRotR, m_edgeRotYR, m_maskRotR, m_limitRotR, m_terminalHistMaxR);



		////////////////////

		m_terminalFound = (m_edgeYLLFound || m_edgeYLRFound || m_edgeYRLFound || m_edgeYRRFound);

		if (!checkEdgeSide()) {
			m_result = false;
			if (m_downsampling > 1) {
				resize(m_bgrL, m_bgrL, Size(imW, imH), INTER_LANCZOS4);
				resize(m_bgrR, m_bgrR, Size(imW, imH), INTER_LANCZOS4);
			}
			return false;
		};

		//		std::cout << "pre rotate" << std::endl;


		m_utils.rotate2(m_edgeRotXL, -m_rotAngleL, m_edgeXL, false);
		m_utils.rotate2(m_edgeRotXR, -m_rotAngleR, m_edgeXR, false);
		m_utils.rotate2(m_edgeRotXLU, -m_rotAngleL, m_edgeXLU, false);
		m_utils.rotate2(m_edgeRotXRU, -m_rotAngleR, m_edgeXRU, false);
		m_utils.rotate2(m_edgeRotXLD, -m_rotAngleL, m_edgeXLD, false);
		m_utils.rotate2(m_edgeRotXRD, -m_rotAngleR, m_edgeXRD, false);
		//		m_utils.rotate2(m_edgeRotYL, -m_rotAngleL, m_edgeYL, false);
		//		m_utils.rotate2(m_edgeRotYR, -m_rotAngleR, m_edgeYR, false);
		m_utils.rotate2(m_edgeRotYLL, -m_rotAngleL, m_edgeYLL, false);
		m_utils.rotate2(m_edgeRotYRL, -m_rotAngleR, m_edgeYRL, false);
		m_utils.rotate2(m_edgeRotYLR, -m_rotAngleL, m_edgeYLR, false);
		m_utils.rotate2(m_edgeRotYRR, -m_rotAngleR, m_edgeYRR, false);
		m_utils.rotate2(m_maskRotL, -m_rotAngleL, m_maskL, false);
		m_utils.rotate2(m_maskRotR, -m_rotAngleR, m_maskR, false);

		//				cv::bitwise_or(m_edgeRotYLL, m_edgeRotYLR, m_edgeRotYL);
		//				imwrite("/media/d/Presentations/secondo_anno/images/maris/edgex.png", m_edgeRotXL);
		//				threshold(m_edgeRotYLR, m_edgeRotYLR, 1, 255, THRESH_BINARY);
		//				imwrite("/media/d/Presentations/secondo_anno/images/maris/edgey.png", m_edgeRotYLR);

		//		std::cout << "post rotate" << std::endl;
		m_utils.rotatePoint2(m_centrL, -m_rotAngleL, m_centrL, Point2d(m_edgeXL.cols / 2.0, m_edgeXL.rows / 2.0), false);
		m_utils.rotatePoint2(m_centrR, -m_rotAngleR, m_centrR, Point2d(m_edgeXL.cols / 2.0, m_edgeXL.rows / 2.0), false);

		//		SHOWP(m_edgeXL,m_edgeXR);

		if (m_downsampling > 1) {
			resize(m_bgrL, m_bgrL, Size(imW, imH), INTER_LANCZOS4);
			resize(m_bgrR, m_bgrR, Size(imW, imH), INTER_LANCZOS4);
			resize(m_edgeXL, m_edgeXL, Size(imW, imH), INTER_LANCZOS4);
			resize(m_edgeXR, m_edgeXR, Size(imW, imH), INTER_LANCZOS4);
			resize(m_edgeXLU, m_edgeXLU, Size(imW, imH), INTER_LANCZOS4);
			resize(m_edgeXRU, m_edgeXRU, Size(imW, imH), INTER_LANCZOS4);
			resize(m_edgeXLD, m_edgeXLD, Size(imW, imH), INTER_LANCZOS4);
			resize(m_edgeXRD, m_edgeXRD, Size(imW, imH), INTER_LANCZOS4);
			//			resize(m_edgeYL, m_edgeYL, Size(imW, imH), INTER_LANCZOS4);
			//			resize(m_edgeYR, m_edgeYR, Size(imW, imH), INTER_LANCZOS4);
			resize(m_edgeYLL, m_edgeYLL, Size(imW, imH), INTER_LANCZOS4);
			resize(m_edgeYRL, m_edgeYRL, Size(imW, imH), INTER_LANCZOS4);
			resize(m_edgeYLR, m_edgeYLR, Size(imW, imH), INTER_LANCZOS4);
			resize(m_edgeYRR, m_edgeYRR, Size(imW, imH), INTER_LANCZOS4);
			resize(m_maskL, m_maskL, Size(imW, imH), INTER_LANCZOS4);
			resize(m_maskR, m_maskR, Size(imW, imH), INTER_LANCZOS4);
			if (m_useArmMask) {
				resize(m_armMaskL, m_armMaskL, Size(imW, imH), INTER_LANCZOS4);
				resize(m_armMaskR, m_armMaskR, Size(imW, imH), INTER_LANCZOS4);
			}

			m_centrL = m_centrL * m_downsampling;
			m_centrR = m_centrR * m_downsampling;
		}

		//		SHOW(m_edgeXL,m_edgeXR);


		//						if (m_edgeXLUFound) {
		//							m_lr.getBestLineRansac(m_edgeXLU, m_lineLU);
		//						}
		//						if (m_edgeXLDFound) {
		//							m_lr.getBestLineRansac(m_edgeXLD, m_lineLD);
		//						}
		//						if (m_edgeXRUFound) {
		//							m_lr.getBestLineRansac(m_edgeXRU, m_lineRU);
		//						}
		//						if (m_edgeXRDFound) {
		//							m_lr.getBestLineRansac(m_edgeXRD, m_lineRD);
		//						}
		//				
		//						if (m_terminalFound) {
		//							if (m_terminalOnLeft) {
		//								m_lr.getBestLineRansac(m_edgeYL, m_lineTerminal);
		//							} else {
		//								m_lr.getBestLineRansac(m_edgeYR, m_lineTerminal);
		//							}
		//						}

		if (m_edgeXLUFound) {
			m_lf.getBestLineFitting(m_edgeXLU, m_lineLU);
		}
		if (m_edgeXLDFound) {
			m_lf.getBestLineFitting(m_edgeXLD, m_lineLD);
		}
		if (m_edgeXRUFound) {
			m_lf.getBestLineFitting(m_edgeXRU, m_lineRU);
		}
		if (m_edgeXRDFound) {
			m_lf.getBestLineFitting(m_edgeXRD, m_lineRD);
		}

		if (m_terminalFound) {
			//			if (m_terminalOnLeft) {
			//				m_lf.getBestLineFitting(m_edgeYL, m_lineTerminal);
			//			} else {
			//				m_lf.getBestLineFitting(m_edgeYR, m_lineTerminal);
			//			}
			if (m_edgeYLLFound) {
				m_lf.getBestLineFitting(m_edgeYLL, m_terminalLL);
			}
			if (m_edgeYLRFound) {
				m_lf.getBestLineFitting(m_edgeYLR, m_terminalLR);
			}
			if (m_edgeYRLFound) {
				m_lf.getBestLineFitting(m_edgeYRL, m_terminalRL);
			}
			if (m_edgeYRRFound) {
				m_lf.getBestLineFitting(m_edgeYRR, m_terminalRR);
			}
		}

		checkLineNormal(m_lineLU, m_centrL);
		checkLineNormal(m_lineRU, m_centrR);
		checkLineNormal(m_lineLD, m_centrL);
		checkLineNormal(m_lineRD, m_centrR);
		if (m_terminalFound) {
			checkLineNormal(m_terminalLL, m_centrL);
			checkLineNormal(m_terminalRL, m_centrR);
			checkLineNormal(m_terminalLR, m_centrL);
			checkLineNormal(m_terminalRR, m_centrR);
		}
		return true;

	}

	void drawFrame(cv::Mat& duo, Eigen::Matrix4d& m) {
		using namespace std;
		using namespace cv;

		Mat dL(duo, cv::Rect(0, 0, m_bgrL.cols, m_bgrL.rows));
		Mat dR(duo, cv::Rect(m_bgrL.cols, 0, m_bgrL.cols, m_bgrL.rows));

		//		Mat mp0l = PL * wp0;
		//		Mat mp0r = PR * wp0;
		//
		//		Mat mp1l = PL * wp1;
		//		Mat mp2l = PL * wp2;
		//
		//		Mat mp1r = PR * wp1;
		//		Mat mp2r = PR * wp2;
		//
		//
		//		m_p1L = Point2d(mp1l.at<double>(0, 0) / mp1l.at<double>(2, 0), mp1l.at<double>(1, 0) / mp1l.at<double>(2, 0));
		//		m_p2L = Point2d(mp2l.at<double>(0, 0) / mp2l.at<double>(2, 0), mp2l.at<double>(1, 0) / mp2l.at<double>(2, 0));
		//
		//		m_p1R = Point2d(mp1r.at<double>(0, 0) / mp1r.at<double>(2, 0), mp1r.at<double>(1, 0) / mp1r.at<double>(2, 0));
		//		m_p2R = Point2d(mp2r.at<double>(0, 0) / mp2r.at<double>(2, 0), mp2r.at<double>(1, 0) / mp2r.at<double>(2, 0));
		//
		//		m_p0L = Point2d(mp0l.at<double>(0, 0) / mp0l.at<double>(2, 0), mp0l.at<double>(1, 0) / mp0l.at<double>(2, 0));
		//		m_p0R = Point2d(mp0r.at<double>(0, 0) / mp0r.at<double>(2, 0), mp0r.at<double>(1, 0) / mp0r.at<double>(2, 0));

	}

	void output(cv::Mat& duo) {
		using namespace std;
		using namespace cv;

		Mat dL(duo, cv::Rect(0, 0, m_bgrL.cols, m_bgrL.rows));
		Mat dR(duo, cv::Rect(m_bgrL.cols, 0, m_bgrL.cols, m_bgrL.rows));

		if (m_result) {
			Mat bgrMaskL, bgrMaskR;


			cvtColor(m_maskL, bgrMaskL, CV_GRAY2BGR);
			cvtColor(m_maskR, bgrMaskR, CV_GRAY2BGR);

			addWeighted(m_bgrL, 1, bgrMaskL, 0.2, 0.0, dL);
			addWeighted(m_bgrR, 1, bgrMaskR, 0.2, 0.0, dR);

			if (m_useArmMask) {
				Mat bgrArmMaskL, bgrArmMaskR;

				cvtColor(m_armMaskL, bgrArmMaskL, CV_GRAY2BGR);
				cvtColor(m_armMaskR, bgrArmMaskR, CV_GRAY2BGR);

				//				for (auto it = bgrArmMaskL.begin(); it != bgrArmMaskL.end(); it++) {
				//					if (*it == Vec3b(255, 255, 255)) {
				//						*it = Vec3b(0, 0, 255);
				//					}
				//				}
				//				for (auto::iterator it = bgrArmMaskR.begin(); it != bgrArmMaskR.end(); it++) {
				//					if (*it == Vec3b(255, 255, 255)) {
				//						*it = Vec3b(0, 0, 255);
				//					}
				//				}

				addWeighted(dL, 1, bgrArmMaskL, 0.2, 0.0, dL);
				addWeighted(dR, 1, bgrArmMaskR, 0.2, 0.0, dR);
			}

			if (m_edgeXLUFound) {
				drawLine(dL, m_lineLU, Scalar(0, 255, 255));
			}
			if (m_edgeXRUFound) {
				drawLine(dR, m_lineRU, Scalar(0, 255, 255));
			}
			if (m_edgeXLDFound) {
				drawLine(dL, m_lineLD, Scalar(0, 255, 255));
			}
			if (m_edgeXRDFound) {
				drawLine(dR, m_lineRD, Scalar(0, 255, 255));
			}

			if (m_terminalFound) {
				//				if (m_terminalOnLeft) {
				//					drawLine(dL, m_lineTerminal, Scalar(0, 255, 0));
				//				} else {
				//					drawLine(dR, m_lineTerminal, Scalar(0, 255, 0));
				//				}
				if (m_edgeYLLFound) {
					drawLine(dL, m_terminalLL, Scalar(0, 255, 255));
				}
				if (m_edgeYLRFound) {
					drawLine(dL, m_terminalLR, Scalar(0, 255, 255));
				}
				if (m_edgeYRLFound) {
					drawLine(dR, m_terminalRL, Scalar(0, 255, 255));
				}
				if (m_edgeYRRFound) {
					drawLine(dR, m_terminalRR, Scalar(0, 255, 255));
				}
			}

			//			m_bgrR.copyTo(dR);



			if (m_terminalFound) {
				line(dL, m_p1L, m_p2L, Scalar(255, 0, 0), 4, CV_AA);
				line(dR, m_p1R, m_p2R, Scalar(255, 0, 0), 4, CV_AA);
				circle(dL, m_p0L, 5, Scalar(255, 0, 0), 4);
				circle(dR, m_p0R, 5, Scalar(255, 0, 0), 4);
			} else {
				line(dL, m_p1L, m_p2L, Scalar(0, 0, 255), 4, CV_AA);
				line(dR, m_p1R, m_p2R, Scalar(0, 0, 255), 4, CV_AA);
				circle(dL, m_p0L, 5, Scalar(0, 0, 255), 4);
				circle(dR, m_p0R, 5, Scalar(0, 0, 255), 4);
			}
		} else {
			//			cvtColor(m_bgrL, m_bgrL, CV_BGR2GRAY);
			//			cvtColor(m_bgrR, m_bgrR, CV_BGR2GRAY);
			//			cvtColor(m_bgrL, m_bgrL, CV_GRAY2BGR);
			//			cvtColor(m_bgrR, m_bgrR, CV_GRAY2BGR);
			m_bgrL.copyTo(dL);
			m_bgrR.copyTo(dR);
		}
		//		resize(duo, duo, Size(duo.cols / 2, duo.rows / 2));

	}

	void drawLineDuo(cv::Mat& duo, const Eigen::Vector3d& line, bool left, const cv::Scalar& color) {
		using namespace std;
		using namespace cv;

		if (left) {
			Mat dL(duo, cv::Rect(0, 0, m_bgrL.cols, m_bgrL.rows));
			drawLine(dL, line, color);
		} else {
			Mat dR(duo, cv::Rect(m_bgrL.cols, 0, m_bgrL.cols, m_bgrL.rows));
			drawLine(dR, line, color);
		}
	}

	void projectLine(const Eigen::Vector3d& d, const Eigen::Vector3d& p0, cv::Mat& PL, cv::Mat& PR, double len) {
		using namespace std;
		using namespace cv;

		if (!m_terminalFound) {
			len = 3;
		}

		Eigen::Vector3d dn = d / d.norm();
		//		std::cout << "d:\n" << d.transpose() << "\np0:\n" << p0.transpose() << std::endl;

		Mat_<double> wp0(4, 1);
		Mat_<double> wp1(4, 1);
		Mat_<double> wp2(4, 1);

		wp0 << p0[0], p0[1], p0[2], 1;

		wp1 << p0[0] + len * dn[0], p0[1] + len * dn[1], p0[2] + len * dn[2], 1;

		wp2 << p0[0] - len * dn[0], p0[1] - len * dn[1], p0[2] - len * dn[2], 1;


		Mat mp0l = PL * wp0;
		Mat mp0r = PR * wp0;

		Mat mp1l = PL * wp1;
		Mat mp2l = PL * wp2;

		Mat mp1r = PR * wp1;
		Mat mp2r = PR * wp2;


		m_p1L = Point2d(mp1l.at<double>(0, 0) / mp1l.at<double>(2, 0), mp1l.at<double>(1, 0) / mp1l.at<double>(2, 0));
		m_p2L = Point2d(mp2l.at<double>(0, 0) / mp2l.at<double>(2, 0), mp2l.at<double>(1, 0) / mp2l.at<double>(2, 0));

		m_p1R = Point2d(mp1r.at<double>(0, 0) / mp1r.at<double>(2, 0), mp1r.at<double>(1, 0) / mp1r.at<double>(2, 0));
		m_p2R = Point2d(mp2r.at<double>(0, 0) / mp2r.at<double>(2, 0), mp2r.at<double>(1, 0) / mp2r.at<double>(2, 0));

		m_p0L = Point2d(mp0l.at<double>(0, 0) / mp0l.at<double>(2, 0), mp0l.at<double>(1, 0) / mp0l.at<double>(2, 0));
		m_p0R = Point2d(mp0r.at<double>(0, 0) / mp0r.at<double>(2, 0), mp0r.at<double>(1, 0) / mp0r.at<double>(2, 0));
	}

	Eigen::Vector3d getLineLU() {
		return m_lineLU;
	}

	Eigen::Vector3d getLineRU() {
		return m_lineRU;
	}

	Eigen::Vector3d getLineLD() {
		return m_lineLD;
	}

	Eigen::Vector3d getLineRD() {
		return m_lineRD;
	}

	Eigen::Vector3d getTerminalLL() {
		return m_terminalLL;
	}

	Eigen::Vector3d getTerminalLR() {
		return m_terminalLR;
	}

	Eigen::Vector3d getTerminalRL() {
		return m_terminalRL;
	}

	Eigen::Vector3d getTerminalRR() {
		return m_terminalRR;
	}
private:
	int m_cols;
	int m_rows;
	int m_diag;

	cv::Mat m_bgrL, m_bgrR;
	cv::Mat m_hsvL, m_hsvR;
	cv::Mat m_limitL, m_limitR;
	cv::Mat m_maskL, m_maskR;

	cv::Mat m_bgrRotL, m_bgrRotR;
	cv::Mat m_hsvRotL, m_hsvRotR;
	cv::Mat m_limitRotL, m_limitRotR;
	cv::Mat m_maskRotL, m_maskRotR;

	cv::Mat m_edgeXL, m_edgeXLU, m_edgeXLD, m_edgeYL, m_edgeYLL, m_edgeYLR;
	cv::Mat m_edgeXR, m_edgeXRU, m_edgeXRD, m_edgeYR, m_edgeYRL, m_edgeYRR;

	cv::Mat m_edgeRotXL, m_edgeRotXLU, m_edgeRotXLD, m_edgeRotYL, m_edgeRotYLL, m_edgeRotYLR;
	cv::Mat m_edgeRotXR, m_edgeRotXRU, m_edgeRotXRD, m_edgeRotYR, m_edgeRotYRL, m_edgeRotYRR;


	cv::Point2d m_centrL, m_centrR;

	cv::Scalar m_lowColor, m_highColor;

	cv::Mat m_square15, m_square7, m_square5, m_square3, m_square50, m_square25;
	cv::Mat m_horLine50, m_horLine25, m_verLine25;

	cv::Mat m_armMaskL;
	cv::Mat m_armMaskR;
	cv::Mat m_armMaskRotL;
	cv::Mat m_armMaskRotR;
	bool m_useArmMask;
	//	cv::Mat m_limit;

	//	std::vector<float> m_angles;
	float m_rotAngleL, m_rotAngleR;
	//	std::vector<float> m_initialAngleGuessL;
	//	std::vector<float> m_initialAngleGuessR;
	float m_downsampling;

	int m_minMaskSize;
	int m_blurSize;
	int m_blurDev;
	int m_maskTh;

	int m_angleDistTh;
	int m_angleDistBreak;

	float m_edgeAngleDetectorTh;
	float m_edgeMapTh;

	float m_terminalHistTh;
	float m_edgeHistDev;

	float m_terminalHistMaxL;
	float m_terminalHistMaxR;
	std::vector<Blob> m_blobsL;
	std::vector<Blob> m_blobsR;
	int m_blobIndexL;
	int m_blobIndexR;

	bool m_result;
	bool m_terminalOnLeft;
	bool m_terminalFound;

	bool m_colorCorrection;

	bool m_edgeXLUFound;
	bool m_edgeXRUFound;
	bool m_edgeXLDFound;
	bool m_edgeXRDFound;

	bool m_edgeYLLFound;
	bool m_edgeYRLFound;
	bool m_edgeYLRFound;
	bool m_edgeYRRFound;

	CVUtils m_utils;
	//	BoostColorDetector m_bcd;
	EdgeAnglesDetector m_eadL;
	EdgeAnglesDetector m_eadR;
	LineRansac m_lr;
	LineFitting m_lf;

	Eigen::Vector3d m_lineLU;
	Eigen::Vector3d m_lineRU;
	Eigen::Vector3d m_lineLD;
	Eigen::Vector3d m_lineRD;
	Eigen::Vector3d m_terminalLL;
	Eigen::Vector3d m_terminalLR;
	Eigen::Vector3d m_terminalRL;
	Eigen::Vector3d m_terminalRR;

	cv::Point2d m_p0L;
	cv::Point2d m_p0R;
	cv::Point2d m_p1L;
	cv::Point2d m_p1R;
	cv::Point2d m_p2L;
	cv::Point2d m_p2R;

	Timer m_tm;

	void drawLine(cv::Mat& im, Eigen::Vector3d& l) {
		cv::Point pt1, pt2;
		double a = l[0], b = l[1], rho = -l[2];
		double x0 = a*rho, y0 = b*rho;
		pt1.x = cvRound(x0 + 2000 * (-b));
		pt1.y = cvRound(y0 + 2000 * (a));
		pt2.x = cvRound(x0 - 2000 * (-b));
		pt2.y = cvRound(y0 - 2000 * (a));
		cv::line(im, pt1, pt2, cv::Scalar(255), 3, CV_AA);
	}

	void drawLine(cv::Mat& im, const Eigen::Vector3d& l, const cv::Scalar& color) {
		cv::Point pt1, pt2;
		double a = l[0], b = l[1], rho = -l[2];
		double x0 = a*rho, y0 = b*rho;
		pt1.x = cvRound(x0 + 2000 * (-b));
		pt1.y = cvRound(y0 + 2000 * (a));
		pt2.x = cvRound(x0 - 2000 * (-b));
		pt2.y = cvRound(y0 - 2000 * (a));
		cv::line(im, pt1, pt2, color, 3, CV_AA);
	}

	void getTargetBlobIndex(EdgeAnglesDetector& ead, const cv::Mat& in, float& angle, int& index, const std::vector<Blob>& blobs) {
		std::vector<float> angles;

		ead.getAngles(in, angles, m_edgeAngleDetectorTh);

		int blobIndex = 0;
		int maxAngleIndex = 0;
		int minAngDist = 36000;
		bool found = false;
		for (int i = 0; i < angles.size() && !found; ++i) {
			for (int j = 0; j < blobs.size() && !found; ++j) {
				float angDist = std::abs(angles[i] - 90 - (blobs[j].m_angle * 180.0 / M_PI));
				if (angDist < m_angleDistTh) {
					if (angDist < minAngDist) {
						minAngDist = angDist;
						blobIndex = j;
						maxAngleIndex = i;
					}
					if (minAngDist < m_angleDistBreak) found = true;
				}
			}
		}
		if (minAngDist < 36000) {
			index = blobIndex;
			angle = angles[maxAngleIndex] - 90;
		} else {
			index = -1;
			angle = 0;
		}
		//		rotAngle = angles[0] - 90;
	}

	void checkBlobMask(cv::Mat& mask, int index, const std::vector<Blob>& blobs) {
		//		std::cout << blobs.size() << " " << index << std::endl;
		mask = blobs[index].m_image.clone();
		cv::Mat tmp;
		for (int i = 0; i < blobs.size(); ++i) {
			if (i != index) {
				cv::bitwise_or(mask, blobs[i].m_image, tmp);
				cv::Moments m = cv::moments(tmp, true);
				float a2 = 0.5 * std::atan2(2 * m.mu11, (m.mu20 - m.mu02));
				float a1 = blobs[index].m_angle;
				if (std::abs(fmodulus(a2 - a1 + M_PI, 2.0 * M_PI) - M_PI) < 0.1) {
					cv::bitwise_or(mask, blobs[i].m_image, mask);
				}
			}
		}
	}

	bool findHSVMask(const cv::Mat& in, cv::Mat& mask, std::vector<float>& ori, const std::string& name) {
		using namespace std;
		using namespace cv;

		bool valid = false;

		Mat tmp(in.rows, in.cols, CV_8UC1, cv::Scalar(0));

		mask = tmp.clone();
		inRange(in, m_lowColor, m_highColor, tmp);
		GaussianBlur(tmp, tmp, Size(m_blurSize, m_blurSize), m_blurDev);

		double minVal;
		double maxVal;
		minMaxLoc(tmp, &minVal, &maxVal);

		//		double th = maxVal * m_maskTh / 100.0 ;
		double th = 255 * m_maskTh / 100.0;
		threshold(tmp, tmp, th, 255, THRESH_BINARY);

		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;

		ori.clear();
		findContours(tmp.clone(), contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
		vector<vector<Point> >hull(contours.size());
		for (int i = 0; i < contours.size(); i++) {
			Mat tmp2(in.rows, in.cols, CV_8UC1, cv::Scalar(0));
			convexHull(Mat(contours[i]), hull[i], false);
			drawContours(tmp2, hull, i, Scalar(255), -1);
			if (contourArea(hull[i]) >= m_minMaskSize) {
				drawContours(mask, hull, i, Scalar(255), -1); //, 8, vector<Vec4i>(), 0, Point());
				ori.push_back(getRotationFromMoments(tmp2));
				valid = true;
			}
		}
		return valid;
	}

	bool findHSVBlobMask(const cv::Mat& in, cv::Mat& mask, std::vector<Blob>& blob, const std::string& name) {
		using namespace std;
		using namespace cv;

		bool valid = false;

		Mat tmp(in.rows, in.cols, CV_8UC1, cv::Scalar(0));

		mask = tmp.clone();
		inRange(in, m_lowColor, m_highColor, tmp);
		GaussianBlur(tmp, tmp, Size(m_blurSize, m_blurSize), m_blurDev);

		double minVal;
		double maxVal;
		minMaxLoc(tmp, &minVal, &maxVal);

		//		double th = maxVal * m_maskTh / 100.0 ;
		double th = 255 * m_maskTh / 100.0;
		threshold(tmp, tmp, th, 255, THRESH_BINARY);

		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;

		blob.clear();
		findContours(tmp.clone(), contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
		vector<vector<Point> >hull(contours.size());
		for (int i = 0; i < contours.size(); i++) {
			convexHull(Mat(contours[i]), hull[i], false);
			if (contourArea(hull[i]) >= m_minMaskSize) {
				Blob b;
				b.m_contour = contours[i];
				b.m_hull = hull[i];
				b.m_image = cv::Mat(in.rows, in.cols, CV_8UC1, cv::Scalar(0));
				drawContours(b.m_image, hull, i, Scalar(255), -1);
				b.m_moments = cv::moments(b.m_image, true);
				b.computeInfoFromMoments();
				blob.push_back(std::move(b));
				valid = true;
				bitwise_or(mask, b.m_image, mask);
			}
		}
		return valid;
	}

	void findEdgeMap(const cv::Mat& bgr, cv::Mat& outx, const cv::Mat& mask, const cv::Mat& limit, const cv::Point2d& cent) {
		using namespace std;
		using namespace cv;
		Mat m, dx;

		morphologyEx(mask, m, MORPH_DILATE, m_square7);
		bitwise_and(m, 255 - limit, m);

		Sobel(bgr, dx, CV_32F, 0, 1, 5);
		Mat edgex(dx.rows, dx.cols, CV_8UC3, Scalar(0, 0, 0));
		dx.copyTo(edgex, m);
		//		edgex = abs(edgex);
		convertScaleAbs(edgex, edgex);
		//		outx = edgex;
		//		return;


		morphologyEx(edgex, edgex, MORPH_OPEN, m_horLine25);
		//		cvtColor(edgex, outx, CV_BGR2GRAY);
		outx = Mat(dx.rows, dx.cols, CV_8UC1, Scalar(0));
		cv::transform(edgex, outx, cv::Matx13f(1, 1));

		double minVal;
		double maxVal;
		minMaxLoc(outx, &minVal, &maxVal);
		threshold(outx, outx, int(maxVal * m_edgeMapTh), 255, THRESH_BINARY);

		std::vector<int> hx(outx.rows, 0);
		//		std::vector<int> hmean(outx.cols, 0);
		float maxU = 0;
		float maxD = 0;
		for (int j = 0; j < outx.rows; j++) {
			uint8_t* outxj = outx.ptr<uint8_t>(j);
			for (int i = 0; i < outx.cols; i++) {
				if (outxj[i] > 0) {
					hx[j] += 1;

					if (j < cent.y) {
						if (hx[j] > hx[maxU]) {
							maxU = j;
						}
					} else {
						if (hx[j] > hx[maxD]) {
							maxD = j;
						}
					}
					//					if (i > 0) {
					//						hy[i - 1] += outxj[i];
					//					}
					//					if (i < outx.cols - 1) {
					//						hy[i + 1] += outxj[i];
					//					}
				}
			}
		}

		//		DEB(cent.y);
		//		DEB(maxU);
		//		DEB(maxD);

		Mat edgeMask(dx.rows, dx.cols, CV_8UC1, Scalar(0));
		for (int j = maxU - m_edgeHistDev; j <= maxU + m_edgeHistDev; j++) {
			if (j >= 0 && j < edgeMask.rows) {
				cv::line(edgeMask, Point(0, j), Point(edgeMask.cols - 1, j), Scalar(255));
			}
		}
		for (int j = maxD - m_edgeHistDev; j <= maxD + m_edgeHistDev; j++) {
			if (j >= 0 && j < edgeMask.rows) {
				cv::line(edgeMask, Point(0, j), Point(edgeMask.cols - 1, j), Scalar(255));
			}
		}
		cv::Mat tempbho;
		outx.copyTo(tempbho, edgeMask);
		outx = tempbho.clone();
	}

	void findTerminalMap(const cv::Mat& bgr, cv::Mat& outyL, cv::Mat& outyR, const cv::Mat& mask, const cv::Mat& armMask, const cv::Mat& limit, bool& validL, bool& validR) {
		using namespace std;
		using namespace cv;
		Mat m, dy, outy;

		//		HUD hud;

		if (countNonZero(mask) < 10) {
			outyL = Mat(bgr.rows, bgr.cols, CV_8UC1, Scalar(0));
			outyR = Mat(bgr.rows, bgr.cols, CV_8UC1, Scalar(0));
			validL = false;
			validR = false;
			return;
		};


		morphologyEx(mask, m, MORPH_DILATE, m_square7);

		std::vector<cv::Point> points;
		cv::Mat_<uchar>::const_iterator it = m.begin<uchar>();
		cv::Mat_<uchar>::const_iterator end = m.end<uchar>();
		for (; it != end; ++it) {
			if (*it) points.push_back(it.pos());
		}

		Rect maskLimits = boundingRect(points);

		bitwise_and(m, 255 - limit, m);
		if (m_useArmMask) {
			//			imwrite("/home/fab/tmp/img/armMask.png", 255 - armMask);
			//			waitKey(3);
						bitwise_and(m, 255 - armMask, m);
		}


		Sobel(bgr, dy, CV_32F, 1, 0, 5);
		Mat edgey(dy.rows, dy.cols, CV_8UC3, Scalar(0, 0, 0));
		dy.copyTo(edgey, m);
		convertScaleAbs(edgey, edgey);




		morphologyEx(edgey, edgey, MORPH_OPEN, m_verLine25);


		//		cvtColor(outy, outy, CV_BGR2GRAY);
		outy = Mat(dy.rows, dy.cols, CV_8UC1, Scalar(0));
		//		cv::transform(edgey, outy, cv::Matx13f(1, 1));
		cvtColor(edgey, outy, CV_BGR2GRAY);

		//		imshow("outy", outy);
		//		waitKey(500090);

		//		Mat ttt;
		//		cvtColor(outy, ttt, CV_GRAY2BGR);
		//		rectangle(ttt, maskLimits, Scalar(255, 0, 0), 3);
		//		imshow("edgey", edgey + ttt);
		//		waitKey(5000000);

		std::vector<int> hy(outy.cols, 0);
		std::vector<int> hmean(outy.cols, 0);
		for (int j = 0; j < outy.rows; j++) {
			uint8_t* outyj = outy.ptr<uint8_t>(j);
			for (int i = 0; i < outy.cols; i++) {
				if (outyj[i] > 0) {
					hy[i] += outyj[i];

					if (i > 0) {
						hy[i - 1] += outyj[i];
					}
					if (i < outy.cols - 1) {
						hy[i + 1] += outyj[i];
					}
				}
			}
		}

		for (int i = 1; i < outy.cols - 1; i++) {
			hmean[i] = (hy[i - 1] + hy[i] + hy[i + 1]) / 3;
		}
		hy.swap(hmean);


		//		int max = 0;
		int maxLimitL = 0;
		int maxLimitR = 0;
		float avg = 0;
		for (int i = 0; i < outy.cols; i++) {
			//			if (hy[i] > hy[max]) {
			//
			//			}
			avg += hy[i];
			if (i <= maskLimits.x + 15) {
				if (hy[i] > hy[maxLimitL]) {
					maxLimitL = i;
				}
			}
			if (i >= maskLimits.x + maskLimits.width - 15) {
				if (hy[i] > hy[maxLimitR]) {
					maxLimitR = i;
				}
			}
		}

		avg /= outy.cols;


		validL = (avg < (hy[maxLimitL] * m_terminalHistTh));
		validR = (avg < (hy[maxLimitR] * m_terminalHistTh));

		outy.copyTo(outyL);
		outy.copyTo(outyR);


		if (validL) {
			Mat roiLout(outyL, Rect(0, 0, std::max(maxLimitL - 1, 0), outy.rows));
			Mat roiRout(outyL, Rect(std::min(outy.cols - 1, maxLimitL + 1), 0, outy.cols - maxLimitL - 1, outy.rows));

			//		if ()
			roiLout.setTo(Scalar(0));
			roiRout.setTo(Scalar(0));
			//			cv::rectangle(outy, maskLimits, cv::Scalar(255), 2);
			//			maxHist = hy[maxLimitL];
		}
		if (validR) {
			Mat roiLout(outyR, Rect(0, 0, std::max(maxLimitR - 1, 0), outy.rows));
			Mat roiRout(outyR, Rect(std::min(outy.cols - 1, maxLimitR + 1), 0, outy.cols - maxLimitR - 1, outy.rows));

			//		if ()
			roiLout.setTo(Scalar(0));
			roiRout.setTo(Scalar(0));
			//			cv::rectangle(outy, maskLimits, cv::Scalar(255), 2);
			//			maxHist = hy[maxLimitR];
		}

		//		imwrite("/media/d/maris_october/video_next_location/phases/outy.png", outyR);
	}

	float getRotationFromMoments(cv::Mat& im) {
		cv::Moments m = cv::moments(im, true);
		double xc = m.m10 / m.m00;
		double yc = m.m01 / m.m00;
		cv::circle(im, cv::Point(xc, yc), 5, cv::Scalar(20), 2);
		double angle = 0.5 * std::atan2(2 * m.mu11, (m.mu20 - m.mu02));
		double xl = 30 * cos(angle);
		double yl = 30 * sin(angle);
		cv::circle(im, cv::Point(xc + xl, yc + yl), 2, cv::Scalar(64), 2);
		return angle * 180 / M_PI;
	}

	void checkLineNormal(Eigen::Vector3d& l, cv::Point p) {
		if ((l(0) * p.x + l(1) * p.y + l(2)) < 0) {
			l = -l;
		};
	}

	double fmodulus(double x, double y) {
		assert(y > 0);
		double n = x / y;
		if (n > 0) {
			if (n > 1) {
				return (x - floor(n)*(y));
			} else {
				return x;
			}
		} else {
			if (n < -1) {
				return (x + (y) + std::abs(floor(n))*(y));
			} else {
				return (x + (y));
			}
		}
	}

	void rotatePoint(cv::Point2d& in, double angle, cv::Point2d& out, const cv::Point2d& piv) {
		double rad = angle * M_PI / 180.0;
		cv::Point2d t, t2;
		t = in - piv;
		t2.x = std::cos(rad) * t.x + std::sin(rad) * t.y;
		t2.y = -std::sin(rad) * t.x + std::cos(rad) * t.y;
		out = t2 + piv;
	}

	void rectFinder(cv::Mat& in, int minArea, std::vector<cv::Rect>& vec) {
		using namespace cv;
		using namespace std;
		Mat t;
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		morphologyEx(in, t, MORPH_DILATE, m_square5);
		findContours(t, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

		vec.clear();
		vec.resize(contours.size());

		for (int i = 0; i < contours.size(); i++) {
			if (contourArea(contours[i]) > minArea) {
				vec[i] = boundingRect(Mat(contours[i]));
			}
		}
	}
public:

	bool checkEdgeSide() {
		using namespace std;
		using namespace cv;

		m_edgeRotXL.copyTo(m_edgeRotXLU);
		m_edgeRotXL.copyTo(m_edgeRotXLD);
		m_edgeRotXR.copyTo(m_edgeRotXRU);
		m_edgeRotXR.copyTo(m_edgeRotXRD);

		//		SHOW(m_edgeRotXLU, m_edgeRotXRU);
		//		SHOWP(m_edgeRotXLD, m_edgeRotXRD);

		//		RectFinder rrf;
		vector<Rect> rectEdgexl;
		vector<Rect> rectEdgexr;
		vector<Rect> rectRegl;
		vector<Rect> rectRegr;

		Rect maxrl;
		Rect maxrr;

		rectFinder(m_edgeRotXL, 0, rectEdgexl);
		rectFinder(m_edgeRotXR, 0, rectEdgexr);

		rectFinder(m_maskRotL, 200, rectRegl);
		rectFinder(m_maskRotR, 200, rectRegr);

		if (rectEdgexl.size() == 0) return false;
		if (rectEdgexr.size() == 0) return false;

		if (rectRegl.size() > 0) {
			maxrl = rectRegl[0];

			for (int i = 1; i < rectRegl.size(); ++i) {
				if (rectRegl[i].height * rectRegl[i].width > maxrl.height * maxrl.width) {
					maxrl = rectRegl[i];
				}
			}
		} else {
			return false;
		}

		if (rectRegr.size() > 0) {
			maxrr = rectRegr[0];

			for (int i = 1; i < rectRegr.size(); ++i) {
				//			rectangle(rr, rectRegr[i], Scalar(255, 255, 0), 1, 8);
				if (rectRegr[i].height * rectRegr[i].width > maxrr.height * maxrr.width) {
					maxrr = rectRegr[i];
				}
			}
		} else {
			return false;
		}




		//		m_centrL.x = maxrl.x + maxrl.width / 2.0;
		//		m_centrL.y = maxrl.y + maxrl.height / 2.0;
		//		m_centrR.x = maxrr.x + maxrr.width / 2.0;
		//		m_centrR.y = maxrr.y + maxrr.height / 2.0;

		int hl = maxrl.y + maxrl.height / 2;
		int hr = maxrr.y + maxrr.height / 2;

		for (int i = 0; i < rectEdgexl.size(); ++i) {
			Rect r = rectEdgexl[i];
			if (r.y + r.height <= hl) {
				rectangle(m_edgeRotXLD, r, Scalar(0), -1, 8);
				//				rectangle(m_edgeRotXLU, r, Scalar(255), 1, 8);
			} else if (r.y >= hl) {
				rectangle(m_edgeRotXLU, r, Scalar(0), -1, 8);
				//				rectangle(m_edgeRotXLD, r, Scalar(255), 1, 8);
			} else {
				rectangle(m_edgeRotXLU, r, Scalar(0), -1, 8);
				rectangle(m_edgeRotXLD, r, Scalar(0), -1, 8);
			}
		}
		for (int i = 0; i < rectEdgexr.size(); ++i) {
			Rect r = rectEdgexr[i];
			if (r.y + r.height <= hr) {
				rectangle(m_edgeRotXRD, r, Scalar(0), -1, 8);
			} else if (r.y >= hr) {
				rectangle(m_edgeRotXRU, r, Scalar(0), -1, 8);
			} else {
				rectangle(m_edgeRotXRU, r, Scalar(0), -1, 8);
				rectangle(m_edgeRotXRD, r, Scalar(0), -1, 8);
			}
		}

		m_edgeXLUFound = (countNonZero(m_edgeRotXLU) > 0);
		m_edgeXRUFound = (countNonZero(m_edgeRotXRU) > 0);
		m_edgeXLDFound = (countNonZero(m_edgeRotXLD) > 0);
		m_edgeXRDFound = (countNonZero(m_edgeRotXRD) > 0);


		return true;
	}
};
