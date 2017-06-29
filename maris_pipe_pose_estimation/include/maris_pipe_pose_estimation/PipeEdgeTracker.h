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
#include "AlphaBetaFilter.h"

struct LineState {

	LineState() {
		count = 0;
		isObs = false;
		present = false;
		useSpeed = false;
		toTrack = false;
	}
	int count;
	bool isObs;
	bool present;
	bool useSpeed;
	bool toTrack;
	Eigen::Vector2d lineObs; //[0] = rho, [1] =  theta
	Eigen::Vector2d lineState;
	Eigen::Vector2d speedObs;
};

class PipeEdgeTracker {
	LineState m_LU;
	LineState m_RU;
	LineState m_LD;
	LineState m_RD;
	LineState m_LL;
	LineState m_RL;
	LineState m_LR;
	LineState m_RR;
	AlphaBetaFilter<Eigen::Vector2d> m_LUFilter;
	AlphaBetaFilter<Eigen::Vector2d> m_RUFilter;
	AlphaBetaFilter<Eigen::Vector2d> m_LDFilter;
	AlphaBetaFilter<Eigen::Vector2d> m_RDFilter;
	AlphaBetaFilter<Eigen::Vector2d> m_LLFilter;
	AlphaBetaFilter<Eigen::Vector2d> m_RLFilter;
	AlphaBetaFilter<Eigen::Vector2d> m_LRFilter;
	AlphaBetaFilter<Eigen::Vector2d> m_RRFilter;
	double m_alpha;
	double m_beta;
	double m_rhoTh;
	double m_thetaTh;

public:

	PipeEdgeTracker(int minLife = 3, int maxLife = 15) : m_minLife(minLife), m_maxLife(maxLife) {
	}

	void addObservationLU(const Eigen::Vector3d& line) {
		addObservation(m_LU, m_LUFilter, line);
	}

	void addObservationRU(const Eigen::Vector3d& line) {
		addObservation(m_RU, m_RUFilter, line);
	}

	void addObservationLD(const Eigen::Vector3d& line) {
		addObservation(m_LD, m_LDFilter, line);
	}

	void addObservationRD(const Eigen::Vector3d& line) {
		addObservation(m_RD, m_RDFilter, line);
	}

	void addObservationLL(const Eigen::Vector3d& line) {
		addObservation(m_LL, m_LLFilter, line);
	}

	void addObservationRL(const Eigen::Vector3d& line) {
		addObservation(m_RL, m_RLFilter, line);
	}

	void addObservationLR(const Eigen::Vector3d& line) {
		addObservation(m_LR, m_LRFilter, line);
	}

	void addObservationRR(const Eigen::Vector3d& line) {
		addObservation(m_RR, m_RRFilter, line);
	}

	void setSpeedLU(const cv::Point2d& trasl, double angle, double deltaT) {
		setSpeed(m_LU, computeSpeed(m_LU, trasl, angle, deltaT));
	}

	void setSpeedLD(const cv::Point2d& trasl, double angle, double deltaT) {
		setSpeed(m_LD, computeSpeed(m_LD, trasl, angle, deltaT));
	}

	void setSpeedRU(const cv::Point2d& trasl, double angle, double deltaT) {
		setSpeed(m_RU, computeSpeed(m_RU, trasl, angle, deltaT));
	}

	void setSpeedRD(const cv::Point2d& trasl, double angle, double deltaT) {
		setSpeed(m_RD, computeSpeed(m_RD, trasl, angle, deltaT));
	}

	void setSpeedLL(const cv::Point2d& trasl, double angle, double deltaT) {
		setSpeed(m_LL, computeSpeed(m_LL, trasl, angle, deltaT));
	}

	void setSpeedLR(const cv::Point2d& trasl, double angle, double deltaT) {
		setSpeed(m_LR, computeSpeed(m_LR, trasl, angle, deltaT));
	}

	void setSpeedRL(const cv::Point2d& trasl, double angle, double deltaT) {
		setSpeed(m_RL, computeSpeed(m_RL, trasl, angle, deltaT));
	}

	void setSpeedRR(const cv::Point2d& trasl, double angle, double deltaT) {
		setSpeed(m_RR, computeSpeed(m_RR, trasl, angle, deltaT));
	}

	bool isLU() {
		return m_LU.present;
	}

	bool isLD() {
		return m_LD.present;
	}

	bool isRU() {
		return m_RU.present;
	}

	bool isRD() {
		return m_RD.present;
	}

	bool isLL() {
		return m_LL.present;
	}

	bool isLR() {
		return m_LR.present;
	}

	bool isRL() {
		return m_RL.present;
	}

	bool isRR() {
		return m_RR.present;
	}

	Eigen::Vector3d getLU() {
		return Eigen::Vector3d(std::cos(m_LU.lineState[1]), std::sin(m_LU.lineState[1]), -m_LU.lineState[0]);
	}

	Eigen::Vector3d getLD() {
		return Eigen::Vector3d(std::cos(m_LD.lineState[1]), std::sin(m_LD.lineState[1]), -m_LD.lineState[0]);
	}

	Eigen::Vector3d getRU() {
		return Eigen::Vector3d(std::cos(m_RU.lineState[1]), std::sin(m_RU.lineState[1]), -m_RU.lineState[0]);
	}

	Eigen::Vector3d getRD() {
		return Eigen::Vector3d(std::cos(m_RD.lineState[1]), std::sin(m_RD.lineState[1]), -m_RD.lineState[0]);
	}

	Eigen::Vector3d getLL() {
		return Eigen::Vector3d(std::cos(m_LL.lineState[1]), std::sin(m_LL.lineState[1]), -m_LL.lineState[0]);
	}

	Eigen::Vector3d getLR() {
		return Eigen::Vector3d(std::cos(m_LR.lineState[1]), std::sin(m_LR.lineState[1]), -m_LR.lineState[0]);
	}

	Eigen::Vector3d getRL() {
		return Eigen::Vector3d(std::cos(m_RL.lineState[1]), std::sin(m_RL.lineState[1]), -m_RL.lineState[0]);
	}

	Eigen::Vector3d getRR() {
		return Eigen::Vector3d(std::cos(m_RR.lineState[1]), std::sin(m_RR.lineState[1]), -m_RR.lineState[0]);
	}

	void setAlpha(double alpha) {
		m_alpha = alpha;
	}

	void setBeta(double beta) {
		m_beta = beta;
	}

	void setRhoThValidation(double rhoTh) {
		m_rhoTh = rhoTh;
	}

	void setThetaThValidation(double thetaTh) {
		m_thetaTh = thetaTh;
	}

	void trackSingleLine(LineState& ls, AlphaBetaFilter<Eigen::Vector2d>& filter, double deltaT) {
		if (ls.toTrack) {
			if (ls.useSpeed) {
				filter.setSpeed(ls.speedObs);
				ls.useSpeed = false;
			}
			filter.predict(deltaT);
			if (ls.isObs) {
				filter.computeResidual(ls.lineObs);
				filter.update(m_alpha, m_beta, deltaT);
			} else {
				//ls.count = std::max(0, ls.count - 1);
				ls.count--;
				if (ls.count <= 0) {
					ls.present = false;
					ls.toTrack = false;
					ls.count = 0;
				}
			}
		}
		if (ls.present && !ls.isObs) {
			ls.count--;
			if (ls.count <= 0) {
				ls.present = false;
				ls.toTrack = false;
				ls.count = 0;
			}
		}
		if (!ls.present && !ls.isObs) {
			ls.count = 0;
			ls.present = false;
			ls.toTrack = false;
		}
		ls.isObs = false;
		ls.lineState = filter.getState();
	}

	void trackEdges(double deltaT) {
		//@TODO: if needed compute speed
		trackSingleLine(m_LU, m_LUFilter, deltaT);
		trackSingleLine(m_LD, m_LDFilter, deltaT);
		trackSingleLine(m_RU, m_RUFilter, deltaT);
		trackSingleLine(m_RD, m_RDFilter, deltaT);

		trackSingleLine(m_LL, m_LLFilter, deltaT);
		trackSingleLine(m_LR, m_LRFilter, deltaT);
		trackSingleLine(m_RL, m_RLFilter, deltaT);
		trackSingleLine(m_RR, m_RRFilter, deltaT);
	}

	void computeTranslation(const cv::Point2d& c1, const cv::Point2d& c2, cv::Point2d& p) {
		p = c1 - c2;
	}

	void computeTranslation(const cv::Mat& im1, const cv::Mat& im2, cv::Point2d& p) {
		cv::Mat t1 = im1.clone();
		cv::Mat t2 = im2.clone();
		CVUtils utils(t1.cols, t1.rows);
		utils.grayWorldColorCorrection(t1);
		utils.grayWorldColorCorrection(t2);
		if (t1.channels() > 1) {
			cv::cvtColor(t1, t1, CV_BGR2GRAY);
		}
		if (t2.channels() > 1) {
			cv::cvtColor(t2, t2, CV_BGR2GRAY);
		}

		SHOWP(t1, t2);
		t1.convertTo(t1, CV_32FC1);
		t2.convertTo(t2, CV_32FC1);
		p = cv::phaseCorrelate(t1, t2);
	}

private:

	int m_minLife;
	int m_maxLife;

	void addObservation(LineState& ls, AlphaBetaFilter<Eigen::Vector2d>& filter, const Eigen::Vector3d& line) {
		ls.lineObs = Eigen::Vector2d(-line[2], std::atan2(line[1], line[0]));
		if (ls.present == true && (std::abs(ls.lineObs[0] - ls.lineState[0]) > m_rhoTh || std::abs(ls.lineObs[1] - ls.lineState[1]) > m_thetaTh)) {
			return;
		}
		ls.count = std::min(ls.count + 1, m_maxLife);
		ls.isObs = true;
		if (ls.count == m_minLife && ls.present == false) {
			filter.setState(ls.lineObs);
			ls.present = true;
		}
		if (ls.count > m_minLife) {
			ls.toTrack = true;
		}
	}

	void setSpeed(LineState& ls, const Eigen::Vector2d& speed) {
		ls.useSpeed = true;
		ls.speedObs = speed;
	}

	Eigen::Vector2d computeSpeed(LineState& ls, const cv::Point2d& trasl, double angle, double deltaT) {
		Eigen::Vector2d speed = Eigen::Vector2d::Zero();
		speed[1] = angle;
		speed[0] = trasl.x * std::cos(ls.lineState[1] + angle) + trasl.y * std::sin(ls.lineState[1] + angle);
		speed = speed / deltaT;
		return speed;
	}
};
