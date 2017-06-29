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

#define SETTER(X, I) template <typename T> inline void X(T f){I = f;};
#define SETTER2(X, I1, I2) template <typename T> inline void X(T f1, T f2){I1 = f1; I2 = f2;};

#define SHOWP(L, R)\
cv::imshow(#L, L);\
cv::imshow(#R, R);\
cv::waitKey(5000000);

#define SHOW(L, R)\
cv::imshow(#L, L);\
cv::imshow(#R, R);\
cv::waitKey(5);

class CVUtils {
private:
	int m_cols;
	int m_rows;
	int m_diag;

	cv::Mat m_gwScl;

	std::vector<int> m_hrow;
	std::vector<int> m_hcol;
public:

	CVUtils(int w, int h)
	: m_cols(w), m_rows(h), m_diag(std::sqrt(m_cols * m_cols + m_rows * m_rows)),
	m_gwScl(m_rows, m_cols, CV_32FC3, cv::Scalar(0, 0, 0)),
	m_hrow(m_rows, 0), m_hcol(m_cols, 0) {
	};

	void undistortAndRectifyImage(cv::Mat& im, const cv::Mat& K, const cv::Mat& D, const cv::Mat& R, const cv::Mat& P) {
		cv::Mat map1, map2;
		cv::initUndistortRectifyMap(K, D, R, P, im.size(), CV_32FC1, map1, map2);
		cv::remap(im, im, map1, map2, cv::INTER_LINEAR);
	}

	void debayerRectification(const cv::Mat& in, cv::Mat& out, const cv::Mat& K, const cv::Mat& D, const cv::Mat& R, const cv::Mat& P) {
		cv::Mat map1, map2;
		cv::cvtColor(in, out, CV_BayerBG2BGR);
		cv::initUndistortRectifyMap(K, D, R, P, out.size(), CV_32FC1, map1, map2);
		cv::remap(out, out, map1, map2, cv::INTER_LINEAR);
	}

	void grayWorldColorCorrection(cv::Mat& src) {
		using namespace std;
		using namespace cv;

		Scalar mn = mean(src);

		double maxc = std::max(std::max(mn[1], mn[2]), mn[0]);

		mn[0] = maxc / (mn[0]);
		mn[1] = maxc / (mn[1]);
		mn[2] = maxc / (mn[2]);

		//			cout << mn << endl;

		m_gwScl.setTo(mn);
		src.convertTo(src, CV_32FC3);
		multiply(src, m_gwScl, src);
		src.convertTo(src, CV_8UC3);
	}

	void grayWorldColorCorrection2(cv::Mat& src) {
		using namespace std;
		using namespace cv;

		Scalar mn = mean(src);

		double maxc = std::max(std::max(mn[1], mn[2]), mn[0]);

		mn[0] = maxc / (mn[0]);
		mn[1] = maxc / (mn[1]);
		mn[2] = maxc / (mn[2]);

		//			cout << mn << endl;

		//		m_gwScl.setTo(mn);
		Mat gwScl(m_rows, m_cols, CV_32FC3, mn);

		src.convertTo(src, CV_32FC3);
		multiply(src, gwScl, src);
		src.convertTo(src, CV_8UC3);
	}

	void contrastCLAHE(cv::Mat& src,int lim = 1) {
		// Creates CLAHE
		cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
		clahe->setClipLimit(lim);
		// Applies CLAHE to all the three color components
		std::vector<cv::Mat> planes;
		cv::split(src,planes);
		clahe->apply(planes[0],planes[0]);
		clahe->apply(planes[1],planes[1]);
		clahe->apply(planes[2],planes[2]);
		cv::merge(planes,src);
	}

	void cleanMask(cv::Mat& in, float th) {
		using namespace std;
		using namespace cv;

		int maxcol = 0;
		int maxrow = 0;

		for (int j = 0; j < in.rows; j++) {
			uint8_t* rowj = in.ptr<uint8_t>(j);
			for (int i = 0; i < in.cols; i++) {
				if (rowj[i]) {
					if ((++m_hcol[i]) > maxcol) {
						maxcol = m_hcol[i];
					};
					if ((++m_hrow[j]) > maxrow) {
						maxrow = m_hrow[j];
					};
				};
			}
		}

		maxcol = static_cast<int> (maxcol * th);
		maxrow = static_cast<int> (maxrow * th);

		for (int j = 0; j < in.rows; j++) {
			uint8_t* rowj = in.ptr<uint8_t>(j);
			for (int i = 0; i < in.cols; i++) {
				if (m_hcol[i] < maxcol || m_hrow[j] < maxrow) {
					rowj[i] = 0;
				}
			}
		}
	}

	void rotate(cv::Mat& src, double angle, cv::Mat& dst) {//,const cv::Size& size) {
		cv::warpAffine(src, dst, cv::getRotationMatrix2D(cv::Point2f(src.cols / 2., src.rows / 2.), angle, 1.0), cv::Size(src.cols, src.rows));
	}

	void rotate2(cv::Mat& src, double angle, cv::Mat& dst, bool expand = true) {
		using namespace std;
		using namespace cv;
		Mat t = src.clone();
		if (expand) {
			if (src.channels() > 1) {
				dst = Mat(m_diag, m_diag, CV_8UC3, Scalar(0, 0, 0));
			} else {
				dst = Mat(m_diag, m_diag, CV_8UC1, Scalar(0));
			}
			Mat roi(dst, Rect((m_diag - m_cols) / 2, (m_diag - m_rows) / 2, m_cols, m_rows));
			t.copyTo(roi);
			cv::warpAffine(dst, dst, cv::getRotationMatrix2D(cv::Point2f(m_diag / 2., m_diag / 2.), angle, 1.0), cv::Size(m_diag, m_diag));
		} else {
			if (src.channels() > 1) {
				dst = Mat(m_rows, m_cols, CV_8UC3, Scalar(0, 0, 0));
			} else {
				dst = Mat(m_rows, m_cols, CV_8UC1, Scalar(0));
			}
			cv::warpAffine(t, t, cv::getRotationMatrix2D(cv::Point2f(m_diag / 2., m_diag / 2.), angle, 1.0), cv::Size(m_diag, m_diag));
			Mat roi(t, Rect((m_diag - m_cols) / 2, (m_diag - m_rows) / 2, m_cols, m_rows));
			roi.copyTo(dst);
		}
	}

	void cleanContours(cv::Mat& in, int lim) {
		for (int j = 0; j < in.rows; j++) {
			uint8_t* rowj = in.ptr<uint8_t>(j);
			for (int i = 0; i < in.cols; i++) {
				if (i < lim || i > in.cols - lim || j < lim || j > in.rows - lim) {
					rowj[i] = 0;
				}
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

	void rotatePoint2(cv::Point2d& in, double angle, cv::Point2d& out, const cv::Point2d& piv, bool expand = true) {
		double rad = angle * M_PI / 180.0;
		cv::Point2d t, t2;
		t = in - piv;
		if (expand) {
			t = t + cv::Point2d((m_diag - m_cols) / 2, (m_diag - m_rows) / 2);
		} else {
			t = t - cv::Point2d((m_diag - m_cols) / 2, (m_diag - m_rows) / 2);
		}
		t2.x = std::cos(rad) * t.x + std::sin(rad) * t.y;
		t2.y = -std::sin(rad) * t.x + std::cos(rad) * t.y;
		out = t2 + piv;
	}

	void wallisOperator(const cv::Mat& im, cv::Mat& out, float A, float B) {
		using namespace std;
		using namespace cv;

		out = im.clone();
		const int ch = im.channels();

		cv::Scalar meanG, stddevG;
		cv::meanStdDev(im, meanG, stddevG);

		double* M = meanG.val;
		double* S = stddevG.val;

		int s = 2;

		for (int j = s; j < im.rows - s; ++j) {
			for (int i = s; i < im.cols - s; ++i) {
				Mat roi(im, Rect(i - s, j - s, 2 * s + 1, 2 * s + 1));
				cv::Scalar meanR, stddevR;
				cv::meanStdDev(roi, meanR, stddevR);

				double* m = meanR.val;
				double* s = stddevR.val;

				if (ch > 1) {
					out.at<Vec3b>(j, i)[0] = static_cast<unsigned char> (S[0] * (im.at<Vec3b>(j, i)[0] - m[0]) / (s[0] + A) + M[0] * B + m[0] * (1 - B));
					out.at<Vec3b>(j, i)[1] = static_cast<unsigned char> (S[1] * (im.at<Vec3b>(j, i)[1] - m[1]) / (s[1] + A) + M[1] * B + m[1] * (1 - B));
					out.at<Vec3b>(j, i)[2] = static_cast<unsigned char> (S[2] * (im.at<Vec3b>(j, i)[2] - m[2]) / (s[2] + A) + M[2] * B + m[2] * (1 - B));
				} else {
					out.at<unsigned char>(j, i) = static_cast<unsigned char> (S[0] * (im.at<unsigned char>(j, i) - m[0]) / (s[0] + A) + M[0] * B + m[0] * (1 - B));
				}
			}
		}

		//		out = t;

	}
};
