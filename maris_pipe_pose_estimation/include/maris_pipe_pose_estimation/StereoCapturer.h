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

#include <string>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

struct CameraInfoData {
	double height;
	double width;
	std::string distortion_model;
	cv::Mat K, D, R, P;
	int bin_x, bin_y;
};

class StereoCapturer {
public:

	StereoCapturer(const std::string& path, const std::string& format, const std::string& ext, bool gray = false, bool mask = false) :
	m_seqNameL(path + format + "_l." + ext), m_seqNameR(path + format + "_r." + ext), m_seqNameMask(path + format + "_mask." + ext),
	m_capL(m_seqNameL), m_capR(m_seqNameR),
	m_fsL(path + "camera_info_left.yaml", cv::FileStorage::READ),
	m_fsR(path + "camera_info_right.yaml", cv::FileStorage::READ),
	m_count(0), m_gray(gray) {
		m_fsL["height"] >> m_infoL.height;
		m_fsL["width"] >> m_infoL.width;
		m_fsL["distortion_model"] >> m_infoL.distortion_model;
		m_fsL["K"] >> m_infoL.K;
		m_fsL["D"] >> m_infoL.D;
		m_fsL["R"] >> m_infoL.R;
		m_fsL["P"] >> m_infoL.P;
		m_fsL["binning_x"] >> m_infoL.bin_x;
		m_fsL["binning_y"] >> m_infoL.bin_y;
		
		m_fsR["height"] >> m_infoR.height;
		m_fsR["width"] >> m_infoR.width;
		m_fsR["distortion_model"] >> m_infoR.distortion_model;
		m_fsR["K"] >> m_infoR.K;
		m_fsR["D"] >> m_infoR.D;
		m_fsR["R"] >> m_infoR.R;
		m_fsR["P"] >> m_infoR.P;
		m_fsR["binning_x"] >> m_infoR.bin_x;
		m_fsR["binning_y"] >> m_infoR.bin_y;
		
		m_fsL.release();
		m_fsR.release();
	};

	void get(cv::Mat& left, cv::Mat& right) {
		m_capL.read(left);
		m_capR.read(right);
		if (left.empty() || right.empty()) {
			m_capL.release();
			m_capR.release();

			m_capL.open(m_seqNameL);
			m_capR.open(m_seqNameR);

			m_capL.read(left);
			m_capR.read(right);
			m_count = 0;
		}
		if (m_gray) {
			if (left.channels() > 1) {
				cv::cvtColor(left, left, CV_BGR2GRAY);
			}
			if (right.channels() > 1) {
				cv::cvtColor(right, right, CV_BGR2GRAY);
			}
		}
		m_count++;
	}

	void get2(cv::Mat& left, cv::Mat& right) {
		m_capL >> left;
		m_capR >> right;
		if (m_gray) {
			if (left.channels() > 1) {
				cv::cvtColor(left, left, CV_BGR2GRAY);
			}
			if (right.channels() > 1) {
				cv::cvtColor(right, right, CV_BGR2GRAY);
			}
		}
		m_count++;
	}

	void get3(cv::Mat& left, cv::Mat& right) {
		char filenameL[255];
		char filenameR[255];
		sprintf(filenameL, m_seqNameL.c_str(), m_count);
		sprintf(filenameR, m_seqNameR.c_str(), m_count);
		left = cv::imread(filenameL, (m_gray ? CV_LOAD_IMAGE_GRAYSCALE : CV_LOAD_IMAGE_COLOR));
		right = cv::imread(filenameR, (m_gray ? CV_LOAD_IMAGE_GRAYSCALE : CV_LOAD_IMAGE_COLOR));
		if (left.data == NULL || right.data == NULL) {
			m_count = 0;
			sprintf(filenameL, m_seqNameL.c_str(), m_count);
			sprintf(filenameR, m_seqNameR.c_str(), m_count);
			left = cv::imread(filenameL, (m_gray ? CV_LOAD_IMAGE_GRAYSCALE : CV_LOAD_IMAGE_COLOR));
			right = cv::imread(filenameR, (m_gray ? CV_LOAD_IMAGE_GRAYSCALE : CV_LOAD_IMAGE_COLOR));
		}
		m_count++;
	}
	
	void get3(cv::Mat& left, cv::Mat& right, cv::Mat& mask) {
		char filenameL[255];
		char filenameR[255];
		char filenameMask[255];
		sprintf(filenameL, m_seqNameL.c_str(), m_count);
		sprintf(filenameR, m_seqNameR.c_str(), m_count);
		sprintf(filenameMask, m_seqNameMask.c_str(), m_count);
		left = cv::imread(filenameL, (m_gray ? CV_LOAD_IMAGE_GRAYSCALE : CV_LOAD_IMAGE_COLOR));
		right = cv::imread(filenameR, (m_gray ? CV_LOAD_IMAGE_GRAYSCALE : CV_LOAD_IMAGE_COLOR));
		mask = cv::imread(filenameMask, (m_gray ? CV_LOAD_IMAGE_GRAYSCALE : CV_LOAD_IMAGE_COLOR));
		if (left.data == NULL || right.data == NULL || mask.data == NULL) {
			m_count = 0;
			sprintf(filenameL, m_seqNameL.c_str(), m_count);
			sprintf(filenameR, m_seqNameR.c_str(), m_count);
			sprintf(filenameMask, m_seqNameMask.c_str(), m_count);
			left = cv::imread(filenameL, (m_gray ? CV_LOAD_IMAGE_GRAYSCALE : CV_LOAD_IMAGE_COLOR));
			right = cv::imread(filenameR, (m_gray ? CV_LOAD_IMAGE_GRAYSCALE : CV_LOAD_IMAGE_COLOR));
			mask = cv::imread(filenameMask, (m_gray ? CV_LOAD_IMAGE_GRAYSCALE : CV_LOAD_IMAGE_COLOR));
		}
		m_count++;
	}
	
	bool getIndex(cv::Mat& left, cv::Mat& right, int index) {
		char filenameL[255];
		char filenameR[255];
		sprintf(filenameL, m_seqNameL.c_str(), index);
		sprintf(filenameR, m_seqNameR.c_str(), index);
		left = cv::imread(filenameL, (m_gray ? CV_LOAD_IMAGE_GRAYSCALE : CV_LOAD_IMAGE_COLOR));
		right = cv::imread(filenameR, (m_gray ? CV_LOAD_IMAGE_GRAYSCALE : CV_LOAD_IMAGE_COLOR));
		if (left.data == NULL || right.data == NULL) {
			return false;
		}
		m_count = index;
		return true;
	}
	
	bool getIndex(cv::Mat& left, cv::Mat& right, cv::Mat& mask, int index) {
		char filenameL[255];
		char filenameR[255];
		char filenameMask[255];
		sprintf(filenameL, m_seqNameL.c_str(), index);
		sprintf(filenameR, m_seqNameR.c_str(), index);
		sprintf(filenameMask, m_seqNameMask.c_str(), index);
		left = cv::imread(filenameL, (m_gray ? CV_LOAD_IMAGE_GRAYSCALE : CV_LOAD_IMAGE_COLOR));
		right = cv::imread(filenameR, (m_gray ? CV_LOAD_IMAGE_GRAYSCALE : CV_LOAD_IMAGE_COLOR));
		mask = cv::imread(filenameMask, (m_gray ? CV_LOAD_IMAGE_GRAYSCALE : CV_LOAD_IMAGE_COLOR));
		if (left.data == NULL || right.data == NULL || mask.data == NULL) {
			return false;
		}
		m_count = index;
		return true;
	}

	int getCount() {
		return m_count;
	}
	
	void getCameraInfoData(CameraInfoData& l, CameraInfoData& r){
		l = m_infoL;
		r = m_infoR;
	}
	
private:
	std::string m_seqNameL;
	std::string m_seqNameR;
	std::string m_seqNameMask;
	cv::VideoCapture m_capL;
	cv::VideoCapture m_capR;
	cv::FileStorage m_fsL;
	cv::FileStorage m_fsR;
	unsigned int m_count;
	bool m_gray;

	CameraInfoData m_infoL;
	CameraInfoData m_infoR;
};

