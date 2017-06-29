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

template <typename T>
class AlphaBetaFilter {
	T xt;
	T xt_1;
	T vt;
	T vt_1;
	T rt;
public:
	AlphaBetaFilter(){
		xt = T::Zero();
		xt_1 = T::Zero();
		vt = T::Zero();
		vt_1 = T::Zero();
		rt = T::Zero();
	}
	
	AlphaBetaFilter(const T& _xt, const T& _vt){
		xt   = _xt;
		xt_1 = T::Zero();
		vt   = _vt;
		vt_1 = T::Zero();
		rt   = T::Zero();
	}
	
	void predict(double deltaT){
		xt_1 = xt;
		vt_1 = vt;
		xt = xt_1 + deltaT * vt_1;
		vt = vt_1;
	}
	
	void computeResidual(const T& xm){
		rt = xm - xt;
	}
	
	void update(double alpha, double beta, double deltaT){
		xt = xt + alpha*rt;
		vt = vt + beta*rt/deltaT;
		
	}
	
	inline T getState(){ return xt; };
	inline void setState(const T& state) { xt = state; };
	inline void setSpeed(const T& speed) { vt = speed; };
};
