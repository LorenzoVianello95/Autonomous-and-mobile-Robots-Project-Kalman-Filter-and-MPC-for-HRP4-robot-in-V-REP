/*
 * File:   Configuration.hpp
 * Author: Paolo Ferrari
 *
 * Created on February 14, 2016
 */

#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <stdio.h>
#include <vector>
#include <iostream>
#include "Eigen/Dense"

typedef Eigen::Matrix<float, 6, 1> Vector6f;

class Configuration {

	private:
		Vector6f q_CoM;
		Eigen::VectorXf q_jnt;

	public:
		Configuration();
		Configuration(int dof);
        ~Configuration();

    void setqCoMPosition(Eigen::Vector3f p);
		void setqCoMOrientation(Eigen::Vector3f o);
		Eigen::Vector3f getqCoMPosition();
		Eigen::Vector3f getqCoMOrientation();
		void setqjntComponent(int i, float val);
		float getqjntComponent(int i);
		Eigen::VectorXf getqjnt();
		void setqjnt(Eigen::VectorXf _qjnt);

};

#endif
