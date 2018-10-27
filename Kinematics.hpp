/*
 * File:   Kinematics.hpp
 * Author: Paolo Ferrari
 *
 * Created on February 16, 2016
 */

#ifndef KINEMATICS_H
#define	KINEMATICS_H

#include "Configuration.hpp"
#include "Eigen/Dense"
#include "Eigen/Core"
#include "enum.h"

class Kinematics {

	public:
		Kinematics();
		virtual ~Kinematics() = 0;

		virtual Eigen::Matrix4f forwardKinematics(Configuration& q, EndEffector ee) = 0;
		virtual Eigen::Matrix4f forwardKinematicsWrtCoM(Configuration& q, EndEffector ee) = 0;
		virtual Eigen::Matrix4f forwardKinematicsWrtTorso(Configuration& q, EndEffector ee) = 0;
		virtual Eigen::VectorXf forwardKinematicsWrtSupportFoot(Configuration& q, EndEffector ee, bool support_foot) = 0;	
		virtual Eigen::MatrixXf getJacobian(Configuration& q, EndEffector base, EndEffector var) = 0;

	private:
		virtual Eigen::MatrixXd rightFoot2CoM(Configuration& q) = 0;
		virtual Eigen::MatrixXd leftFoot2CoM(Configuration& q) = 0;
		virtual Eigen::MatrixXd rightFoot2Torso(Configuration& q) = 0;
		virtual Eigen::MatrixXd leftFoot2Torso(Configuration& q) = 0;
		virtual Eigen::MatrixXd rightFoot2RightHand(Configuration& q) = 0;
		virtual Eigen::MatrixXd rightFoot2LeftHand(Configuration& q) = 0;
		virtual Eigen::MatrixXd rightFoot2Head(Configuration& q) = 0;

		virtual Eigen::MatrixXd jacobianRF2LF(Configuration& q) = 0;
		virtual Eigen::MatrixXd jacobianLF2RF(Configuration& q) = 0;
		virtual Eigen::MatrixXd jacobianRF2LH(Configuration& q) = 0;
		virtual Eigen::MatrixXd jacobianLF2LH(Configuration& q) = 0;
		virtual Eigen::MatrixXd jacobianRF2RH(Configuration& q) = 0;
		virtual Eigen::MatrixXd jacobianLF2RH(Configuration& q) = 0;
		virtual Eigen::MatrixXd jacobianRF2CoM(Configuration& q) = 0;
		virtual Eigen::MatrixXd jacobianLF2CoM(Configuration& q) = 0;

		virtual Eigen::MatrixXd rightFoot2TopCamera(Configuration& q) = 0;
		virtual Eigen::MatrixXd leftFoot2TopCamera(Configuration& q) = 0;
		virtual Eigen::MatrixXd jacobianRF2TopCamera(Configuration& q) = 0;
		virtual Eigen::MatrixXd jacobianLF2TopCamera(Configuration& q) = 0;

		virtual Eigen::MatrixXd jacobianRF2Head(Configuration& q) = 0;
		virtual Eigen::MatrixXd jacobianLF2Head(Configuration& q) = 0;

};

#endif	/* KINEMATICS_H */
