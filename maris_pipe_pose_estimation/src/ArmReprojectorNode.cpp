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
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <dynamic_reconfigure/server.h>


#include <iostream>
#include <fstream>
#include <string>
#include <thread>
#include <boost/lexical_cast.hpp>

#include <maris_pipe_pose_estimation/Timer.h>
#include <maris_pipe_pose_estimation/CVUtils.h>
#include <maris_pipe_pose_estimation/Profiler.h>

#include <maris_ros_msgs/TransfMatrix.h>

#include <eigen3/Eigen/Dense>


using namespace std;
using namespace cv;

typedef message_filters::sync_policies::ApproximateTime< sensor_msgs::Image,
sensor_msgs::CameraInfo,
sensor_msgs::Image,
sensor_msgs::CameraInfo> StereoSyncPolicy;

void eigenToMaris(const Eigen::Matrix4d& eigenTransf, maris_ros_msgs::TransfMatrix& marisTransf) {
	for (int i = 0; i < 16; ++i) {
		int r = i % 4;
		int c = i / 4;
		marisTransf.cwmatrix[i] = eigenTransf(r, c);
	}
}

void marisToEigen(const maris_ros_msgs::TransfMatrix& marisTransf, Eigen::Matrix4d& eigenTransf) {
	for (int i = 0; i < 16; ++i) {
		int r = i % 4;
		int c = i / 4;
		eigenTransf(r, c) = marisTransf.cwmatrix[i];
	}
}

class ArmReprojector {
public:

	ArmReprojector(ros::NodeHandle& nh, int imW, int imH, int downsampling) :
	m_nh(nh),
	m_infoPL(cv::Mat_<double> (3, 4)),
	m_infoPR(cv::Mat_<double> (3, 4)),
	m_sync(StereoSyncPolicy(10), m_subSyncLeftImage, m_subSyncLeftCameraInfo, m_subSyncRightImage, m_subSyncRightCameraInfo),
	m_imW(imW), m_imH(imH), m_downsampling(downsampling),
	m_imWd(m_imW / m_downsampling), m_imHd(m_imH / m_downsampling),
	m_armL(imH, imW, CV_8UC1, cv::Scalar(0)),
	m_armR(imH, imW, CV_8UC1, cv::Scalar(0)),
	m_bgrL(imH, imW, CV_8UC3, cv::Scalar(0, 0, 0)),
	m_bgrR(imH, imW, CV_8UC3, cv::Scalar(0, 0, 0)),
	m_output(false),
	m_vTbReceived(false), m_vTcReceived(false),
	m_bTxReceived(8, false), m_armPointsCvL(8), m_armPointsCvR(8), m_hullCvL(8), m_hullCvR(8), m_armPoints(8), m_bTxMat(8) //[0] non si verrà mai usato
	{
		std::string ns;
		if (!m_nh.getParam("namespace", ns)) {
			ROS_ERROR("stereo namespace parameter not provided");
			exit(0);
		}

		m_nh.getParam("output", m_output);

		std::string vTbTopicString;
		std::string vTcTopicString;
		std::string bT1TopicString;
		std::string bT2TopicString;
		std::string bT3TopicString;
		std::string bT4TopicString;
		std::string bT5TopicString;
		std::string bT6TopicString;
		std::string bT7TopicString;

		m_nh.getParam("vTb", vTbTopicString);
		m_nh.getParam("vTc", vTcTopicString);
		m_nh.getParam("bT1", bT1TopicString);
		m_nh.getParam("bT2", bT2TopicString);
		m_nh.getParam("bT3", bT3TopicString);
		m_nh.getParam("bT4", bT4TopicString);
		m_nh.getParam("bT5", bT5TopicString);
		m_nh.getParam("bT6", bT6TopicString);
		m_nh.getParam("bT7", bT7TopicString);

		m_vTbSub = m_nh.subscribe(vTbTopicString.c_str(), 1, &ArmReprojector::vTbCallback, this);
		m_vTcSub = m_nh.subscribe(vTcTopicString.c_str(), 1, &ArmReprojector::vTcCallback, this);
		m_bT1Sub = m_nh.subscribe(bT1TopicString.c_str(), 1, &ArmReprojector::bT1Callback, this);
		m_bT2Sub = m_nh.subscribe(bT2TopicString.c_str(), 1, &ArmReprojector::bT2Callback, this);
		m_bT3Sub = m_nh.subscribe(bT3TopicString.c_str(), 1, &ArmReprojector::bT3Callback, this);
		m_bT4Sub = m_nh.subscribe(bT4TopicString.c_str(), 1, &ArmReprojector::bT4Callback, this);
		m_bT5Sub = m_nh.subscribe(bT5TopicString.c_str(), 1, &ArmReprojector::bT5Callback, this);
		m_bT6Sub = m_nh.subscribe(bT6TopicString.c_str(), 1, &ArmReprojector::bT6Callback, this);
		m_bT7Sub = m_nh.subscribe(bT7TopicString.c_str(), 1, &ArmReprojector::bT7Callback, this);

		std::string transfTopic;
		nh.param<std::string>("transf", transfTopic, "1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1");
		paramTransfMatrix(transfTopic, m_transfMat);

		m_subSyncLeftImage.subscribe(m_nh, ns + "/left/image_rect_color", 1);
		m_subSyncRightImage.subscribe(m_nh, ns + "/right/image_rect_color", 1);
		m_subSyncLeftCameraInfo.subscribe(m_nh, ns + "/left/camera_info", 1);
		m_subSyncRightCameraInfo.subscribe(m_nh, ns + "/right/camera_info", 1);
		m_sync.registerCallback(boost::bind(&ArmReprojector::imageCallback, this, _1, _2, _3, _4));

		m_leftArmImagePub = m_nh.advertise<sensor_msgs::Image>(ns + "/arm_left/image_raw", 1);
		m_rightArmImagePub = m_nh.advertise<sensor_msgs::Image>(ns + "/arm_right/image_raw", 1);

		std::string armPointsFilename;
		m_nh.getParam("arm_points", armPointsFilename);
		readArmPoints(armPointsFilename);

		//		dynamic_reconfigure::Server<maris_pipe_pose_estimation::PipePoseEstimationNodeParamsConfig>::CallbackType f = boost::bind(&PipePoseEstimation::configCallback, this, _1, _2);
		//		m_server.setCallback(f);

		m_count = 0;
	}

	void vTbCallback(const maris_ros_msgs::TransfMatrixConstPtr& vTbPtr) {
		cout << "vTb received" << endl;
		if (not m_vTbReceived) {
			marisToEigen(*vTbPtr, m_vTbMat);
			m_vTbReceived = true;
			if (m_vTcReceived) {
				m_cTbMat = m_vTcMat.inverse() * m_vTbMat;
			}
			m_vTbSub.shutdown();
		}
	}

	void vTcCallback(const maris_ros_msgs::TransfMatrixConstPtr& vTcPtr) {
		cout << "vTc received" << endl;
		if (not m_vTcReceived) {
			marisToEigen(*vTcPtr, m_vTcMat);
			m_vTcReceived = true;
			if (m_vTbReceived) {
				m_cTbMat = m_vTcMat.inverse() * m_vTbMat;
			}
			m_vTcSub.shutdown();
		}
	}

	void bT1Callback(const maris_ros_msgs::TransfMatrixConstPtr& bT1Ptr) {
//		cout << "bT1 received" << endl;
		marisToEigen(*bT1Ptr, m_bTxMat[1]);
		m_bTxReceived[1] = true;
		updateArmImage();
	}

	void bT2Callback(const maris_ros_msgs::TransfMatrixConstPtr& bT2Ptr) {
//		cout << "bT2 received" << endl;
		marisToEigen(*bT2Ptr, m_bTxMat[2]);
		m_bTxReceived[2] = true;
		updateArmImage();
	}

	void bT3Callback(const maris_ros_msgs::TransfMatrixConstPtr& bT3Ptr) {
//		cout << "bT3 received" << endl;
		marisToEigen(*bT3Ptr, m_bTxMat[3]);
		m_bTxReceived[3] = true;
		updateArmImage();
	}

	void bT4Callback(const maris_ros_msgs::TransfMatrixConstPtr& bT4Ptr) {
//		cout << "bT4 received" << endl;
		marisToEigen(*bT4Ptr, m_bTxMat[4]);
		m_bTxReceived[4] = true;
		updateArmImage();
	}

	void bT5Callback(const maris_ros_msgs::TransfMatrixConstPtr& bT5Ptr) {
//		cout << "bT5 received" << endl;
		marisToEigen(*bT5Ptr, m_bTxMat[5]);
		m_bTxReceived[5] = true;
		updateArmImage();
	}

	void bT6Callback(const maris_ros_msgs::TransfMatrixConstPtr& bT6Ptr) {
//		cout << "bT6 received" << endl;
		marisToEigen(*bT6Ptr, m_bTxMat[6]);
		m_bTxReceived[6] = true;
		updateArmImage();
	}

	void bT7Callback(const maris_ros_msgs::TransfMatrixConstPtr& bT7Ptr) {
//		cout << "bT7 received" << endl;
		marisToEigen(*bT7Ptr, m_bTxMat[7]);
		m_bTxReceived[7] = true;
		updateArmImage();
	}

	void updateArmImage() {
		m_armL = Mat(m_imH, m_imW, CV_8UC1, cv::Scalar(0));
		m_armR = Mat(m_imH, m_imW, CV_8UC1, cv::Scalar(0));

		for (int joint = 1; joint < 8; ++joint) {
			if (m_bTxReceived[joint]) {
				const int size = m_armPoints[joint].size();
				if (size > 0) {
					for (int i = 0; i < size; ++i) {
						Eigen::Vector4d p = m_cTbMat * m_transfMat *  m_bTxMat[joint]  * m_armPoints[joint][i];
						Eigen::Vector3d pL = m_infoPMatL * p;
						Eigen::Vector3d pR = m_infoPMatR * p;
						m_armPointsCvL[joint][i] = cv::Point(pL[0] / pL[2], pL[1] / pL[2]);
						m_armPointsCvR[joint][i] = cv::Point(pR[0] / pR[2], pR[1] / pR[2]);
					}
					cv::convexHull(cv::Mat(m_armPointsCvL[joint]), m_hullCvL[joint]);
					cv::convexHull(cv::Mat(m_armPointsCvR[joint]), m_hullCvR[joint]);
					
//					drawBox(m_armL, m_armPointsCvL[joint], cv::Scalar(0, 0, 255));
//					drawBox(m_armR, m_armPointsCvR[joint], cv::Scalar(0, 0, 255));
					
					drawContours(m_armL, m_hullCvL, joint, cv::Scalar(255), -1);
					drawContours(m_armR, m_hullCvR, joint, cv::Scalar(255), -1);
				}
			}
		}
	}

	void drawBox(cv::Mat& im, const std::vector<cv::Point>& p, const cv::Scalar& color) {
		cv::Scalar green(0, 255, 0);
		cv::Scalar red(0, 0, 255);
		cv::Scalar blue(255, 0, 0);
		if (p.size() == 8) {
			line(im, p[0], p[2], green);
			line(im, p[2], p[1], red);
			line(im, p[3], p[1], green);
			line(im, p[0], p[3], red);
			line(im, p[4], p[7], red);
			line(im, p[5], p[7], green);
			line(im, p[5], p[6], red);
			line(im, p[4], p[6], green);
			line(im, p[0], p[4], blue);
			line(im, p[6], p[2], blue);
			line(im, p[7], p[3], blue);
			line(im, p[5], p[1], blue);
		}
	}

	void imageCallback(const sensor_msgs::ImageConstPtr& leftImage, const sensor_msgs::CameraInfoConstPtr& leftInfo, const sensor_msgs::ImageConstPtr& rightImage, const sensor_msgs::CameraInfoConstPtr& rightInfo) {
		//		m_tm.start();
		cout << "images received" << endl;

		try {
			m_bgrL = cv_bridge::toCvCopy(leftImage, leftImage->encoding)->image;
			m_bgrR = cv_bridge::toCvCopy(rightImage, rightImage->encoding)->image;
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}


		for (unsigned int i = 0; i < 12; ++i) {
			m_infoPL.at<double>(i / 4, i % 4) = leftInfo->P.elems[i];
			m_infoPR.at<double>(i / 4, i % 4) = rightInfo->P.elems[i];
			m_infoPMatL(i / 4, i % 4) = leftInfo->P.elems[i];
			m_infoPMatR(i / 4, i % 4) = rightInfo->P.elems[i];
		}

		m_headerL = leftImage->header;
		m_headerR = rightImage->header;

		++m_count;
	}

	void publishMask() {
		if (m_vTbReceived && m_vTcReceived /*&& m_bT2Received && m_bT1Received && m_bT1Received && m_bT1Received && m_bT1Received && m_bT1Received && m_bT1Received*/) {

			cout << "publishing" << endl;
			m_leftArmCvImagePtr = boost::shared_ptr<cv_bridge::CvImage>(new cv_bridge::CvImage(m_headerL, sensor_msgs::image_encodings::MONO8, m_armL));
			m_rightArmCvImagePtr = boost::shared_ptr<cv_bridge::CvImage>(new cv_bridge::CvImage(m_headerR, sensor_msgs::image_encodings::MONO8, m_armR));
			m_leftArmImagePub.publish(m_leftArmCvImagePtr);
			m_rightArmImagePub.publish(m_rightArmCvImagePtr);

			if (m_output) {
				if (m_downsampling > 1) {
					resize(m_armL, m_armL, Size(m_imWd, m_imHd));
					resize(m_armR, m_armR, Size(m_imWd, m_imHd));
				}

				Mat duo(m_armL.rows, m_armL.cols * 2, CV_8UC1, cv::Scalar(0));
				Mat dR(duo, cv::Rect(m_armL.cols, 0, m_armL.cols, m_armL.rows));
				Mat dL(duo, cv::Rect(0, 0, m_armL.cols, m_armL.rows));
				m_armL.copyTo(dL);
				m_armR.copyTo(dR);
				imshow("duo", duo);
				waitKey(5);
			}
		}
	}

	void readArmPoints(const std::string& filename) {
		cout << "reading '" << filename << "'" << endl;
		std::ifstream in(filename.c_str());
		if (in.is_open()) {
			string line;
			getline(in, line);
			while (!in.eof()) {
				getline(in, line);
				if (line.length() > 0) {
					stringstream ss(line);
					int jointIndex, numPoints;
					ss >> jointIndex >> numPoints;
					m_armPoints[jointIndex].resize(numPoints);
					m_armPointsCvL[jointIndex].resize(numPoints);
					m_armPointsCvR[jointIndex].resize(numPoints);
					//					m_hullCvL[jointIndex].resize(numPoints);
					//					m_hullCvR[jointIndex].resize(numPoints);
					for (int j = 0; j < numPoints; ++j) {
						getline(in, line);
						if (line.length() > 0) {
							stringstream ss2(line);
							double x, y, z;
							ss2 >> x >> y >> z;
							m_armPoints[jointIndex][j] << x, y, z, 1;
						}
					}
				}
			}
		} else {
			cout << "error reading files" << endl;
		}
		in.close();
		cout << "armPoints file readed" << endl;
		for (int i = 0; i < 8; ++i) {
			std::cout << "joint " << i << " contains " << m_armPoints[i].size() << " points" << endl;
		}
	}

	//	void configCallback(maris_pipe_pose_estimation::PipePoseEstimationNodeParamsConfig& cfg, uint32_t level) {
	//		cout << "config called" << endl;
	//
	//	}
private:
	ros::NodeHandle m_nh;
	ros::Publisher m_leftArmImagePub;
	ros::Publisher m_rightArmImagePub;

	bool m_vTbReceived;
	bool m_vTcReceived;
	std::vector<bool> m_bTxReceived;

	ros::Subscriber m_vTbSub;
	ros::Subscriber m_vTcSub;
	ros::Subscriber m_bT1Sub;
	ros::Subscriber m_bT2Sub;
	ros::Subscriber m_bT3Sub;
	ros::Subscriber m_bT4Sub;
	ros::Subscriber m_bT5Sub;
	ros::Subscriber m_bT6Sub;
	ros::Subscriber m_bT7Sub;

	Eigen::Matrix4d m_cTbMat;
	Eigen::Matrix4d m_vTbMat;
	Eigen::Matrix4d m_vTcMat;
	Eigen::Matrix4d m_transfMat;
	std::vector<Eigen::Matrix4d> m_bTxMat;
	std::vector<std::vector<Eigen::Vector4d> > m_armPoints;
	std::vector<std::vector<cv::Point> > m_armPointsCvL;
	std::vector<std::vector<cv::Point> > m_armPointsCvR;
	std::vector<std::vector<cv::Point> > m_hullCvL;
	std::vector<std::vector<cv::Point> > m_hullCvR;

	message_filters::Subscriber<sensor_msgs::Image> m_subSyncLeftImage;
	message_filters::Subscriber<sensor_msgs::CameraInfo> m_subSyncLeftCameraInfo;
	message_filters::Subscriber<sensor_msgs::Image> m_subSyncRightImage;
	message_filters::Subscriber<sensor_msgs::CameraInfo> m_subSyncRightCameraInfo;

	cv_bridge::CvImagePtr m_leftArmCvImagePtr;
	cv_bridge::CvImagePtr m_rightArmCvImagePtr;

	cv::Mat m_bgrL;
	cv::Mat m_bgrR;
	cv::Mat m_armL;
	cv::Mat m_armR;
	cv::Mat m_infoPL;
	cv::Mat m_infoPR;
	Eigen::Matrix<double, 3, 4> m_infoPMatL;
	Eigen::Matrix<double, 3, 4> m_infoPMatR;
	message_filters::Synchronizer<StereoSyncPolicy> m_sync;
	std_msgs::Header m_headerL, m_headerR;

	//	dynamic_reconfigure::Server<maris_pipe_pose_estimation::PipePoseEstimationNodeParamsConfig> m_server;

	int m_imW, m_imH, m_downsampling;
	int m_imWd, m_imHd;
	bool m_output;

	Timer m_tm;

	int m_count;

	void paramTransfMatrix(const std::string& value, Eigen::Matrix4d & m) {
		std::stringstream ss(value);
		double v;
		for (int i = 0; i < 16 && (ss >> v); ++i) {
			int r = i % 4;
			int c = i / 4;
			m(r, c) = v;
		}
	}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "ArmReprojectorNode");
	ros::NodeHandle nh("~");

	int imW, imH, downsampling;

	if (!nh.getParam("width", imW)) {
		ROS_ERROR("width parameter not provided");
		exit(0);
	}

	if (!nh.getParam("height", imH)) {
		ROS_ERROR("height parameter not provided");
		exit(0);
	}

	if (!nh.getParam("downsampling", downsampling)) {
		ROS_ERROR("downsampling parameter not provided");
		exit(0);
	}


	ArmReprojector ar(nh, imW, imH, downsampling);

	ros::Rate loopRate(10);
	while (ros::ok()) {
		ros::spinOnce();
		ar.publishMask();
		loopRate.sleep();
	}
	return 0;
}
