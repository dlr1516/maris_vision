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
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <dynamic_reconfigure/server.h>
#include <maris_pipe_pose_estimation/PipePoseEstimationNodeParamsConfig.h>
#include <maris_pipe_pose_estimation/Line3D.h>


#include <iostream>
#include <string>
#include <thread>
#include <boost/lexical_cast.hpp>

#include <maris_pipe_pose_estimation/Timer.h>
#include <maris_pipe_pose_estimation/StereoCapturer.h>
#include <maris_pipe_pose_estimation/HUD.h>
#include <maris_pipe_pose_estimation/CVUtils.h>
#include <maris_pipe_pose_estimation/MarisPipeEdgeDetector.h>
#include <maris_pipe_pose_estimation/CylinderLSEstimator.h>
#include <maris_pipe_pose_estimation/Profiler.h>
#include <maris_pipe_pose_estimation/PipeEdgeTracker.h>

#include <maris_ros_msgs/TransfMatrix.h>
#include <maris_ros_msgs/VehiclePosition.h>


using namespace std;
using namespace cv;

typedef message_filters::sync_policies::ExactTime< sensor_msgs::Image,
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

class PipePoseEstimation {
public:

	PipePoseEstimation(ros::NodeHandle& nh, int imW, int imH, int downsampling) :
	m_nh(nh),
	m_infoPL(cv::Mat_<double> (3, 4)),
	m_infoPR(cv::Mat_<double> (3, 4)),
	m_sync(StereoSyncPolicy(10), m_subSyncLeftImage, m_subSyncLeftCameraInfo, m_subSyncRightImage, m_subSyncRightCameraInfo),
	m_imW(imW), m_imH(imH), m_downsampling(downsampling),
	m_imWd(m_imW / m_downsampling), m_imHd(m_imH / m_downsampling),
	m_duo(imH, imW * 2, CV_8UC3, cv::Scalar(0, 0, 0)),
	m_ped(m_imWd, m_imHd, m_downsampling),
	m_useArmRep(false), m_armMaskLReceived(false), m_armMaskRReceived(false),
	m_hud(1) {
		std::string ns;
		if (!m_nh.getParam("namespace", ns)) {
			ROS_ERROR("stereo namespace parameter not provided");
			exit(0);
		}

		std::string wTv;
		m_nh.getParam("wTv", wTv);
		m_clse.setNadir(Eigen::Vector3d(0, 0, 1));
		m_clse.enableTracking(true);
		
		m_nh.getParam("use_arm_reprojection", m_useArmRep);

		if (!m_nh.getParam("downsampling_output", m_downsampling_output)) {
			ROS_ERROR("downsampling_output parameter not provided");
			exit(0);
		}


		m_wTvSub = m_nh.subscribe(wTv.c_str(), 1, &PipePoseEstimation::wTvCallback, this);

		if (m_useArmRep){
			m_armLeftSub = m_nh.subscribe(ns + "/arm_left/image_raw", 1, &PipePoseEstimation::leftArmCb, this);
			m_armRightSub = m_nh.subscribe(ns + "/arm_right/image_raw", 1, &PipePoseEstimation::rightArmCb, this);
		}
		
		m_subSyncLeftImage.subscribe(m_nh, ns + "/left/image_rect_color", 1);
		m_subSyncRightImage.subscribe(m_nh, ns + "/right/image_rect_color", 1);
		m_subSyncLeftCameraInfo.subscribe(m_nh, ns + "/left/camera_info", 1);
		m_subSyncRightCameraInfo.subscribe(m_nh, ns + "/right/camera_info", 1);
		m_sync.registerCallback(boost::bind(&PipePoseEstimation::imageCallback, this, _1, _2, _3, _4));
		m_duoImagePub = m_nh.advertise<sensor_msgs::Image>(ns + "/duo/image_raw", 1);
		m_lineMarkerPub = m_nh.advertise<visualization_msgs::Marker>(ns + "/lineMarker", 1);
		m_pipePosePub = m_nh.advertise<geometry_msgs::PoseStamped>(ns + "/pipePose", 1);
		m_pipePoseMarisPub = m_nh.advertise<maris_ros_msgs::TransfMatrix>("/vision/cToRaw", 1);
		m_vTcPub = m_nh.advertise<maris_ros_msgs::TransfMatrix>("/vision/vTc", 1);
		m_vToPub = m_nh.advertise<maris_ros_msgs::TransfMatrix>("/vision/vToRaw", 1);
		m_cTlPub = m_nh.advertise<maris_ros_msgs::TransfMatrix>("/vision/cTlRaw", 1);
		m_vTgPub = m_nh.advertise<maris_ros_msgs::TransfMatrix>("/vision/vTgRaw", 1);
		m_vLgPub = m_nh.advertise<maris_pipe_pose_estimation::Line3D>("/vision/vLg", 1);
		m_cLlPub = m_nh.advertise<maris_pipe_pose_estimation::Line3D>("/vision/cLl", 1);

		dynamic_reconfigure::Server<maris_pipe_pose_estimation::PipePoseEstimationNodeParamsConfig>::CallbackType f = boost::bind(&PipePoseEstimation::configCallback, this, _1, _2);
		m_server.setCallback(f);

		std::string vTcStr;
		std::string oTlStr;
		std::string oTgStr;

		nh.param<std::string>("vTcTransf", vTcStr, "1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1");
		nh.param<std::string>("oTlTransf", oTlStr, "1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1");
		nh.param<std::string>("oTgTransf", oTgStr, "1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1");

		//        m_vTcMat << 0.05770, -0.95340, -0.29614, 0.27246,
		//                0.99818, 0.06034, 0.00028, -0.07184,
		//                0.01760, -0.29562, 0.95515, 1.10210,
		//                0.00000, 0.00000, 0.00000, 1.00000;
		//        std::cout << m_vTcMat.transpose() << std::endl;
		//        m_oTlMat << 1.0, 0.0, 0.0, 0.0,
		//                0.0, 1.0, 0.0, 0.0, //-m_pipeLenght / 4.0,
		//                0.0, 0.0, 1.0, 0.0,
		//                0.0, 0.0, 0.0, 1.0;
		//        m_oTgMat << 1.0, 0.0, 0.0, 0.0,
		//                0.0, 1.0, 0.0, -m_pipeLenght / 4.0,
		//                0.0, 0.0, 1.0, -0.60, //-0.5, //<<<<<<<<<<<<<<<<<<<<<<GOAL
		//                0.0, 0.0, 0.0, 1.0;

		paramTransfMatrix(vTcStr, m_vTcMat);
		paramTransfMatrix(oTlStr, m_oTlMat);
		paramTransfMatrix(oTgStr, m_oTgMat);

		m_lineMarker.id = 0;
		m_lineMarker.type = visualization_msgs::Marker::ARROW;
		m_lineMarker.action = visualization_msgs::Marker::ADD;
		m_lineMarker.ns = ns;
		m_lineMarker.points.resize(2);

		m_count = 0;
	}

	void wTvCallback(const maris_ros_msgs::VehiclePositionConstPtr& wTvPtr) {
		const maris_ros_msgs::TransfMatrix& m = wTvPtr->wTv;
		m_clse.setNadir(Eigen::Vector3d(m.cwmatrix[8], m.cwmatrix[9], m.cwmatrix[10]));
	}

	void leftArmCb(const sensor_msgs::ImageConstPtr& image){
		m_armMaskL = cv_bridge::toCvCopy(image, image->encoding)->image;
		m_armMaskLReceived = true;
	}
	
	void rightArmCb(const sensor_msgs::ImageConstPtr& image){
		m_armMaskR = cv_bridge::toCvCopy(image, image->encoding)->image;
		m_armMaskRReceived = true;
	}
	
	void imageCallback(const sensor_msgs::ImageConstPtr& leftImage, const sensor_msgs::CameraInfoConstPtr& leftInfo, const sensor_msgs::ImageConstPtr& rightImage, const sensor_msgs::CameraInfoConstPtr& rightInfo) {
		m_tm.start();
		double deltaT = leftImage->header.stamp.toSec() - m_lastFrameTime;
		cout << "STEREO Callback" << endl;
		m_cToMat = Eigen::Matrix4d::Identity();
		bool isFrame = false;
		try {
			m_bgrL = cv_bridge::toCvShare(leftImage, leftImage->encoding)->image;
			m_bgrR = cv_bridge::toCvShare(rightImage, rightImage->encoding)->image;
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}



		for (unsigned int i = 0; i < 12; ++i) {
			m_infoPL.at<double>(i / 4, i % 4) = leftInfo->P.elems[i];
			m_infoPR.at<double>(i / 4, i % 4) = rightInfo->P.elems[i];
		}



		m_clse.setCameraMatrixLeft(m_infoPL);
		m_clse.setCameraMatrixRight(m_infoPR);
		
		if (m_useArmRep && m_armMaskLReceived && m_armMaskRReceived){
			m_ped.setArmMasks(m_armMaskL, m_armMaskR);
		}

		PRUN("ped  ");
		bool result = m_ped.findEdgeLines(m_bgrL, m_bgrR);
		PSTOP("ped  ");

		//		g_prof.stop("ped    ");

		if (m_count > 0) {
			if (result) {
				cv::Point2d traslL, traslR;
				m_pet.computeTranslation(m_ped.getLeftMaskCentroid(), m_lastLeftCentroid, traslL);
				m_pet.computeTranslation(m_ped.getRightMaskCentroid(), m_lastRightCentroid, traslR);
				double rotL = m_ped.getLeftMaskRotation() - m_lastLeftRotation;
				double rotR = m_ped.getRightMaskRotation() - m_lastRightRotation;
				//				std::cout << "deltaT =\t" << deltaT << std::endl;
				//				std::cout << "traslL =\t" << traslL.x << "\t" << traslL.y << std::endl;
				//				std::cout << "traslR =\t" << traslR.x << "\t" << traslR.y << std::endl;
				//				std::cout << "angleL =\t" << rotL << std::endl;
				//				std::cout << "angleR =\t" << rotR << std::endl;

				if (m_ped.isEdgeLUFound()) {
					m_pet.addObservationLU(m_ped.getLineLU());
					DEB("Add LU");
				}

				if (m_ped.isEdgeLDFound()) {
					m_pet.addObservationLD(m_ped.getLineLD());
					DEB("Add LD");
				}

				if (m_ped.isEdgeRUFound()) {
					m_pet.addObservationRU(m_ped.getLineRU());
					DEB("Add RU");
				}


				if (m_ped.isEdgeRDFound()) {
					m_pet.addObservationRD(m_ped.getLineRD());
					DEB("Add RD");
				}

				//				m_pet.setSpeedLU(traslL, rotL, deltaT);
				//				m_pet.setSpeedLD(traslL, rotL, deltaT);
				//				m_pet.setSpeedRU(traslR, rotR, deltaT);
				//				m_pet.setSpeedRD(traslR, rotR, deltaT);

				if (m_ped.isTerminalFound()) {
					//				if (m_ped.isTerminalLLFound()) {
					//					m_clse.setTerminalLineLeft(m_ped.getLineTerminal());
					//				} else {
					//					m_clse.setTerminalLineRight(m_ped.getLineTerminal());
					//				}
					if (m_ped.isTerminalLLFound()) {
						m_pet.addObservationLL(m_ped.getTerminalLL());
						DEB("Add LL");
					}
					if (m_ped.isTerminalLRFound()) {
						m_pet.addObservationLR(m_ped.getTerminalLR());
						DEB("Add LR");
					}
					if (m_ped.isTerminalRLFound()) {
						m_pet.addObservationRL(m_ped.getTerminalRL());
						DEB("Add RL");
					}
					if (m_ped.isTerminalRRFound()) {
						m_pet.addObservationRR(m_ped.getTerminalRR());
						DEB("Add RR");
					}
				}

			}

			m_pet.trackEdges(deltaT);

			////////////////////////////////////////////////
			//		if (result) {
			int countLine = 0;
			int countTerminal = 0;
			if (m_pet.isLU()) {
				m_clse.setLineLeft(m_pet.getLU());
				countLine++;
			}
			if (m_pet.isLD()) {
				m_clse.setLineLeft(m_pet.getLD());
				countLine++;
			}
			if (m_pet.isRU()) {
				m_clse.setLineRight(m_pet.getRU());
				countLine++;
			}
			if (m_pet.isRD()) {
				m_clse.setLineRight(m_pet.getRD());
				countLine++;
			}


			if (m_pet.isLL()) {
				m_clse.setTerminalLineLeft(m_pet.getLL());
				countTerminal++;
			}
			if (m_pet.isLR()) {
				m_clse.setTerminalLineLeft(m_pet.getLR());
				countTerminal++;
			}
			if (m_pet.isRL()) {
				m_clse.setTerminalLineRight(m_pet.getRL());
				countTerminal++;
			}
			if (m_pet.isRR()) {
				m_clse.setTerminalLineRight(m_pet.getRR());
				countTerminal++;
			}


			std::cout << "countLine: " << countLine << std::endl;
			if (countLine > 1) {
				//				m_clse.computeCylinderAxis(m_axisDirection, m_axisPoint0);
				isFrame = m_clse.computeReferenceFrame(m_cToMat, m_axisDirection, m_axisPoint0);

				setLineMarker(m_lineMarker, m_axisDirection, m_axisPoint0, m_pipeLenght / 2.0, m_pipeRadius, (countTerminal > 0));
				m_lineMarker.header = leftImage->header;
				m_lineMarkerPub.publish(m_lineMarker);

				m_ped.projectLine(m_axisDirection, m_axisPoint0, m_infoPL, m_infoPR, m_pipeLenght / 2);

				if (isFrame) {
					//				cout << m_pipeFrame.matrix() << endl;
					setLinePose(m_pipePose, m_cToMat);
					m_pipePose.header = leftImage->header;
					m_pipePosePub.publish(m_pipePose);
					setLinePoseMaris(m_pipePoseMaris, m_cToMat);
					m_pipePoseMarisPub.publish(m_pipePoseMaris);
					// Publish cTl				
					m_cTlMat = m_cToMat * m_oTlMat;
					maris_ros_msgs::TransfMatrix cTlTransfMatrix;
					eigenToMaris(m_cTlMat, cTlTransfMatrix);
					m_cTlPub.publish(cTlTransfMatrix);
					// Publish vTg
					maris_ros_msgs::TransfMatrix vTgTransfMatrix;
					m_cTgMat = m_cToMat * m_oTgMat;
					m_vTgMat = m_vTcMat * m_cTgMat;
					eigenToMaris(m_vTgMat, vTgTransfMatrix);
					m_vTgPub.publish(vTgTransfMatrix);
					m_vToMat = m_vTcMat * m_cToMat;
					maris_ros_msgs::TransfMatrix vToTransfMatrix;
					eigenToMaris(m_vToMat, vToTransfMatrix);
					m_vToPub.publish(vToTransfMatrix);
				} else {
					Eigen::Vector4d m_cp0, m_vp0;
					m_cp0 << m_axisPoint0[0], m_axisPoint0[1], m_axisPoint0[2], 1;
					Eigen::Vector4d m_cpd, m_vpd;
					m_cpd << m_axisPoint0[0] + 5.0 * m_axisDirection[0], m_axisPoint0[1] + 5.0 * m_axisDirection[1], m_axisPoint0[2] + 5.0 * m_axisDirection[2], 1;

					m_vp0 = m_vTcMat * m_cp0;
					m_vpd = m_vTcMat * m_cpd;
					Eigen::Vector3d m_vd;
					m_vd = m_vpd.block<3, 1>(0, 0) - m_vp0.block<3, 1>(0, 0);
					m_vd = m_vd / m_vd.norm();

					m_vLg.p0.x = m_vp0[0];
					m_vLg.p0.y = m_vp0[1];
					m_vLg.p0.z = m_vp0[2];

					m_vLg.d.x = m_vd[0];
					m_vLg.d.y = m_vd[1];
					m_vLg.d.z = m_vd[2];

					m_vLgPub.publish(m_vLg);

					m_cLl.p0.x = m_axisPoint0[0];
					m_cLl.p0.y = m_axisPoint0[1];
					m_cLl.p0.z = m_axisPoint0[2];

					m_cLl.d.x = m_axisDirection[0];
					m_cLl.d.y = m_axisDirection[1];
					m_cLl.d.z = m_axisDirection[2];

					m_cLlPub.publish(m_cLl);
				}
				//		} else {
				//			cout << "FAILED" << endl;
				//		}
				//		double total = totalTimer.getTime().milli;
				//		cout << "Total Time Elapsed " << total << endl;
				//		g_prof.printInfo(total);
			}
			m_clse.resetTerminalLines();
			m_clse.resetLines();

			maris_ros_msgs::TransfMatrix vTcTransfMatrix;
			eigenToMaris(m_vTcMat, vTcTransfMatrix);
			m_vTcPub.publish(vTcTransfMatrix);
		}

		m_ped.output(m_duo);
		if (m_pet.isLU()) {
			m_ped.drawLineDuo(m_duo, m_pet.getLU(), true, cv::Scalar(255, 0, 255));
		}
		if (m_pet.isLD()) {
			m_ped.drawLineDuo(m_duo, m_pet.getLD(), true, cv::Scalar(255, 0, 255));
		}
		if (m_pet.isRU()) {
			m_ped.drawLineDuo(m_duo, m_pet.getRU(), false, cv::Scalar(255, 0, 255));
		}
		if (m_pet.isRD()) {
			m_ped.drawLineDuo(m_duo, m_pet.getRD(), false, cv::Scalar(255, 0, 255));
		}

		if (m_pet.isLL()) {
			m_ped.drawLineDuo(m_duo, m_pet.getLL(), true, cv::Scalar(0, 255, 0));
		}
		if (m_pet.isLR()) {
			m_ped.drawLineDuo(m_duo, m_pet.getLR(), true, cv::Scalar(0, 255, 0));
		}
		if (m_pet.isRL()) {
			m_ped.drawLineDuo(m_duo, m_pet.getRL(), false, cv::Scalar(0, 255, 0));
		}
		if (m_pet.isRR()) {
			m_ped.drawLineDuo(m_duo, m_pet.getRR(), false, cv::Scalar(0, 255, 0));
		}

//		Mat dR(m_duo, cv::Rect(m_bgrL.cols, 0, m_bgrL.cols, m_bgrL.rows));
//		imwrite("/media/d/Presentations/secondo_anno/images/maris/outR.png", dR);
//		Mat dL(m_duo, cv::Rect(0, 0, m_bgrL.cols, m_bgrL.rows));
//        m_bgrR.copyTo(dL);
//        char filenameR[255];
//		sprintf(filenameR, "/media/d/maris_march/results/out%05d.png", m_count);
//		imwrite(filenameR, m_duo);
        

		//        if (m_count > 0) {
		//            cv::Point2d transl;
		//            m_pet.computeTranslation(m_ped.getLeftMaskCentroid(), m_lastLeftCentroid, transl);
		//            //			std::cout << transl.x << "\t" << transl.y << std::endl;
		//            m_trackedPos = m_trackedPos + transl;
		//            cv::circle(m_duo, m_trackedPos, 10, cv::Scalar(0, 0, 255), -1);
		//        }

		//		if (isFrame) {
		//			m_ped.drawFrame(m_duo, m_pipeFrame);
		//		}

		//		m_hud.set(0, "index " + boost::lexical_cast<std::string>(leftImage->header.seq));
		//		m_hud.drawHUD(m_duo);


		if (m_downsampling_output > 1) {
			Mat resi;
			resize(m_duo, resi, Size(m_duo.cols / m_downsampling_output, m_duo.rows / m_downsampling_output));
			m_hud.drawHUD(resi);
			m_duoCvImagePtr = boost::shared_ptr<cv_bridge::CvImage>(new cv_bridge::CvImage(leftImage->header, sensor_msgs::image_encodings::BGR8, resi));
		} else {
			m_hud.drawHUD(m_duo);
			m_duoCvImagePtr = boost::shared_ptr<cv_bridge::CvImage>(new cv_bridge::CvImage(leftImage->header, sensor_msgs::image_encodings::BGR8, m_duo));
		}


		//		imshow("duo", m_duoCvImagePtr->image);
		//		waitKey(3);

		m_duoImagePub.publish(m_duoCvImagePtr->toImageMsg());
		//		char m_filename[255];
		//		sprintf(m_filename, ("/media/d/maris_october/video_next_location/res/frame%05d.png"), m_count);
		//		cv::imwrite(m_filename, m_duoCvImagePtr->image);
		int elaps = m_tm.getTime().milli;
		cout << "[ELAPSED TIME]\t" << elaps << endl;
		cout << "[FRAME COUNT]\t" << m_count << endl;
		//		g_prof.printInfo(elaps);
		cout << endl;

		++m_count;
		m_trackedPos = cv::Point2d(m_bgrL.cols / 2, m_bgrL.rows / 2);
		m_lastLeftCentroid = m_ped.getLeftMaskCentroid();
		m_lastLeftRotation = m_ped.getLeftMaskRotation();
		m_lastRightCentroid = m_ped.getRightMaskCentroid();
		m_lastRightRotation = m_ped.getRightMaskRotation();
		m_lastFrameTime = leftImage->header.stamp.toSec();
	}

	void configCallback(maris_pipe_pose_estimation::PipePoseEstimationNodeParamsConfig& cfg, uint32_t level) {
		cout << "config called" << endl;
		//        cfg.Color;
		//        switch (cfg.Color) {
		//            case 0:
		//                m_ped.setPipeColor(Scalar(0, 15, 35), Scalar(10, 255, 255));
		//                //								m_ped.setPipeColor(Scalar(0, 35, 35), Scalar(35, 255, 255));
		//                break;
		//            case 1:
		//                m_ped.setPipeColor(Scalar(140, 35, 35), Scalar(170, 255, 255));
		//                break;
		//            case 2:
		//                m_ped.setPipeColor(Scalar(35, 10, 10), Scalar(50, 255, 255));
		//                break;
		//        }

		cout << "minH " << cfg.minH << std::endl;
		m_ped.setColorCorrection(cfg.colorCorrection);
		m_ped.setPipeColor(Scalar(cfg.minH, cfg.minS, cfg.minV), Scalar(cfg.maxH, cfg.maxS, cfg.maxV));
		m_ped.setAngleDistBreak(cfg.angleDistBreak);
		m_ped.setAngleDistThreshold(cfg.angleDistThreshold);
		m_ped.setBlurDev(cfg.blurStandarDev);
		m_ped.setBlurSize((cfg.blurSize % 2 == 0 ? cfg.blurSize + 1 : cfg.blurSize));
		m_ped.setEdgeAngleDetectorThreshold(cfg.edgeAngleDetectorThreshold);
		m_ped.setEdgeMapThreshold(static_cast<double> (cfg.edgeMapThreshold / 100.0));
		m_ped.setMaskThreshold(cfg.maskThreshold);
		m_ped.setMinMaskSize(cfg.minMaskSize);
		m_ped.setTerminalHistThreshold(cfg.terminalHistThreshold / 100.0);
		m_ped.setEdgeHistDeviation(cfg.edgeHistogramDeviation);
		m_pet.setAlpha(cfg.alphaFilter);
		m_pet.setBeta(cfg.betaFilter);
		m_pet.setRhoThValidation(cfg.rhoThFilter);
		m_pet.setThetaThValidation(cfg.thetaThFilter);
		m_clse.setRadius(cfg.pipeRadius);
		m_clse.setLength(cfg.pipeLenght);

		m_pipeRadius = cfg.pipeRadius;
		m_pipeLenght = cfg.pipeLenght;
	}
private:
	ros::NodeHandle m_nh;
	ros::Publisher m_duoImagePub;
	ros::Publisher m_lineMarkerPub;
	ros::Publisher m_pipePosePub;
	ros::Publisher m_pipePoseMarisPub;
	ros::Publisher m_vTcPub;
	ros::Publisher m_vToPub;
	ros::Publisher m_cTlPub;
	ros::Publisher m_vTgPub;
	ros::Publisher m_vLgPub;
	ros::Publisher m_cLlPub;
	ros::Subscriber m_wTvSub;
	ros::Subscriber m_armLeftSub;
	ros::Subscriber m_armRightSub;
	Eigen::Matrix4d m_vTcMat;
	Eigen::Matrix4d m_vToMat;
	Eigen::Matrix4d m_oTlMat;
	Eigen::Matrix4d m_cTlMat;
	Eigen::Matrix4d m_oTgMat;
	Eigen::Matrix4d m_cTgMat;
	Eigen::Matrix4d m_vTgMat;
	maris_pipe_pose_estimation::Line3D m_vLg;
	maris_pipe_pose_estimation::Line3D m_cLl;
	geometry_msgs::PoseStamped m_pipePose;
	message_filters::Subscriber<sensor_msgs::Image> m_subSyncLeftImage;
	message_filters::Subscriber<sensor_msgs::CameraInfo> m_subSyncLeftCameraInfo;
	message_filters::Subscriber<sensor_msgs::Image> m_subSyncRightImage;
	message_filters::Subscriber<sensor_msgs::CameraInfo> m_subSyncRightCameraInfo;
	cv_bridge::CvImagePtr m_duoCvImagePtr;

	cv::Mat m_armMaskL;
	cv::Mat m_armMaskR;
	bool m_armMaskLReceived;
	bool m_armMaskRReceived;
	
	cv::Mat m_bgrL;
	cv::Mat m_bgrR;
	cv::Mat m_infoPL;
	cv::Mat m_infoPR;
	message_filters::Synchronizer<StereoSyncPolicy> m_sync;

	visualization_msgs::Marker m_lineMarker;

	dynamic_reconfigure::Server<maris_pipe_pose_estimation::PipePoseEstimationNodeParamsConfig> m_server;

	int m_imW, m_imH, m_downsampling;
	int m_downsampling_output;
	int m_imWd, m_imHd;

	cv::Mat m_duo;

	cv::Point2d m_trackedPos;
	cv::Point2d m_lastLeftCentroid;
	double m_lastLeftRotation;
	cv::Point2d m_lastRightCentroid;
	double m_lastRightRotation;
	double m_lastFrameTime;

	CylinderLSEstimator m_clse;
	MarisPipeEdgeDetector m_ped;
	PipeEdgeTracker m_pet;
	Timer m_tm;

	Eigen::Vector3d m_axisDirection;
	Eigen::Vector3d m_axisPoint0;

	Eigen::Matrix4d m_cToMat;

	double m_pipeLenght;
	double m_pipeRadius;
	
	bool m_useArmRep;

	HUD m_hud;

	maris_ros_msgs::TransfMatrix m_pipePoseMaris;

	int m_count;

	void setLineMarker(visualization_msgs::Marker& m, Eigen::Vector3d& d, Eigen::Vector3d p0, double len, double radius, bool ter) {
		if (ter) {
			m.color.a = 1.0;
			m.color.b = 1.0;
			m.color.g = 0.0;
			m.color.r = 0.0;
		} else {
			m.color.a = 1.0;
			m.color.b = 0.0;
			m.color.g = 0.0;
			m.color.r = 1.0;

			len = 5;
		}

		Eigen::Vector3d dn = d / d.norm();

		m.points[0].x = p0[0] + len * dn[0];
		m.points[0].y = p0[1] + len * dn[1];
		m.points[0].z = p0[2] + len * dn[2];

		m.points[1].x = p0[0] - len * dn[0];
		m.points[1].y = p0[1] - len * dn[1];
		m.points[1].z = p0[2] - len * dn[2];

		m.scale.x = radius;
		m.scale.y = radius;
		m.scale.z = 0.01;
	}

	void setLinePose(geometry_msgs::PoseStamped& p, Eigen::Matrix4d& m) {
		Eigen::Quaterniond qr(m.block<3, 3>(0, 0));
		p.pose.orientation.w = qr.w();
		p.pose.orientation.x = qr.x();
		p.pose.orientation.y = qr.y();
		p.pose.orientation.z = qr.z();
		p.pose.position.x = m(0, 3);
		p.pose.position.y = m(1, 3);
		p.pose.position.z = m(2, 3);
	}

	void setLinePoseMaris(maris_ros_msgs::TransfMatrix& tf, Eigen::Matrix4d& m) {
		for (int i = 0; i < 16; ++i) {
			tf.cwmatrix[i] = m.data()[i];
		}
	}

	void paramTransfMatrix(const std::string& value, Eigen::Matrix4d& m) {
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
	ros::init(argc, argv, "PipePoseEstimationNode");
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


	PipePoseEstimation ppe(nh, imW, imH, downsampling);

	ros::Rate loopRate(10);
	while (ros::ok()) {
		ros::spinOnce();
		loopRate.sleep();
	}
	return 0;
}
