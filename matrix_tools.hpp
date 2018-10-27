/*
 * File:   matrix_tools.hpp
 * Author: Paolo Ferrari
 *
 * Created on Saturday Feb 27, 2016 
 */


#ifndef MATRIX_TOOLS_HPP
#define MATRIX_TOOLS_HPP

#include <iostream>
#include "Eigen/SVD"
#include "Eigen/Core"
#include "Eigen/Dense"
//#include "constant_values.hpp"
#include "v_repLib.h"

#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/Robots.h>
#include <RBDyn/FK.h>
#include <RBDyn/IK.h>
#include <RBDyn/FV.h>
#include <RBDyn/Jacobian.h>
#include <RBDyn/MultiBodyGraph.h>
#include <RBDyn/CoM.h>

inline Eigen::Matrix3f rotx(double ax){
	
	Eigen::Matrix3f rx;
	rx.setZero();

	rx(0,0) = 1;
	rx(0,1) = 0;
	rx(0,2) = 0;
	rx(1,0) = 0;
	rx(1,1) = cos(ax);
	rx(1,2) = -sin(ax);
	rx(2,0) = 0;
	rx(2,1) = sin(ax);
	rx(2,2) = cos(ax);
	
	return rx;
		
}


inline Eigen::Matrix3f roty(double ay){
	
	Eigen::Matrix3f ry;
	ry.setZero();

	ry(0,0) = cos(ay);
	ry(0,1) = 0;
	ry(0,2) = sin(ay);
	ry(1,0) = 0;
	ry(1,1) = 1;
	ry(1,2) = 0;
	ry(2,0) = -sin(ay);
	ry(2,1) = 0;
	ry(2,2) = cos(ay);
	
	return ry;
		
}


inline Eigen::Matrix3f rotz(double az){
	
	Eigen::Matrix3f rz;
	rz.setZero();
	
	rz(0,0) = cos(az);
	rz(0,1) = -sin(az);
	rz(0,2) = 0;
	rz(1,0) = sin(az);
	rz(1,1) = cos(az);
	rz(1,2) = 0;
	rz(2,0) = 0;
	rz(2,1) = 0;
	rz(2,2) = 1;
	
	return rz;
		
}

inline Eigen::Matrix3f rot(Eigen::Vector3f eul){
	
	Eigen::Matrix3f r;

	Eigen::Matrix3f rx = rotx(eul(0));
	Eigen::Matrix3f ry = roty(eul(1));
	Eigen::Matrix3f rz = rotz(eul(2));

	r = rx*ry*rz;

	return r;
		
}

inline Eigen::Matrix4f v2t(Eigen::VectorXf v){
	
	Eigen::Matrix4f m = Eigen::Matrix4f::Identity();

	Eigen::Vector3f eul = v.block<3,1>(3,0);
	Eigen::Matrix3f r = rot(eul);

	m.block<3,3>(0,0) = r;
	m.block<3,1>(0,3) = v.block<3,1>(0,0);
	

	return m;
		
}

inline Eigen::VectorXf t2v(Eigen::Matrix4f m) {
	Eigen::VectorXf v(6);

	float beta = atan2( m(0,2), sqrt(pow(m(0,0), 2) + pow(m(0,1), 2)) );
	float alpha = atan2( -m(1,2)/cos(beta), m(2,2)/cos(beta) );
	float gamma = atan2( -m(0,1)/cos(beta), m(0,0)/cos(beta) );
	
	v(0) = m(0,3);
	v(1) = m(1,3);
	v(2) = m(2,3);
	v(3) = alpha;
	v(4) = beta;
	v(5) = gamma;    

	return v;
}

inline Eigen::MatrixXf pinv(Eigen::MatrixXf A){
	 
	Eigen::MatrixXf	pinvA = A.transpose()*(A*A.transpose()).inverse();

	return pinvA;	
}

inline Eigen::MatrixXf p2t(sva::PTransformd PT){
	
	Eigen::Vector3d p = PT.translation();
	Eigen::Matrix3d R = PT.rotation().transpose();
	Eigen::Vector3d e = R.eulerAngles(0,1,2); 
	Eigen::VectorXf v(6);
	v << p(0), p(1), p(2), e(0), e(1), e(2);
	Eigen::MatrixXf T = v2t(v); 	 
	
	return T;	
}

#endif /* MATRIX_TOOLS_HPP */
