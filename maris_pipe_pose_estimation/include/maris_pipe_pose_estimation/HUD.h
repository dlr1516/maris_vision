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
 */#ifndef HUD_H
#define	HUD_H

#include <vector>
#include <string>
#include <opencv2/imgproc/imgproc.hpp>

class HUD {
public:

	HUD(const int lines = 0) : m_hud(lines), m_color(255, 255, 255), m_size(12) {
	};

	void set(int line, const std::string& s) {
		m_hud[line] = s;
	}

	void add(const std::string& s) {
		m_hud.push_back(s);
	}

	void setColor(const cv::Scalar& c) {
		m_color = c;
	}

	void setSize(int s) {
		m_size = s;
	}

	void drawHUD(cv::Mat& in) {
		for (auto ii = 0; ii < m_hud.size(); ++ii) {
			cv::putText(in, m_hud[ii], cv::Point(0, m_size * (ii + 1)), cv::FONT_HERSHEY_SIMPLEX, 0.4, m_color);
		}
	};
private:
	std::vector<std::string> m_hud;
	cv::Scalar m_color;
	int m_size;
};

#endif	/* HUD_H */

