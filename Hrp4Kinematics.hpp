/*
 * File:   Hrp4Kinematics.hpp
 * Author: Paolo Ferrari
 *
 * Created on July 20, 2017
 */

#ifndef HRP4KINEMATICS_H
#define	HRP4KINEMATICS_H

#include "Kinematics.hpp"
#include "Configuration.hpp"
#include "Eigen/Dense"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Eigen/StdVector"
#include "Eigen/SVD"
#include "matrix_tools.hpp"

#include "Robot.hpp"
#include "Hrp4.hpp"

#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/Robots.h>
#include <RBDyn/FK.h>
#include <RBDyn/IK.h>
#include <RBDyn/FV.h>
#include <RBDyn/Jacobian.h>
#include <RBDyn/MultiBodyGraph.h>
#include <RBDyn/CoM.h>


class Hrp4Kinematics : public Kinematics {

	public:
		Hrp4Kinematics();
		~Hrp4Kinematics();

		Eigen::Matrix4f forwardKinematics(Configuration& q, EndEffector ee);  // wrt World
		Eigen::Matrix4f forwardKinematicsWrtCoM(Configuration& q, EndEffector ee);
		Eigen::Matrix4f forwardKinematicsWrtTorso(Configuration& q, EndEffector ee);
		Eigen::VectorXf forwardKinematicsWrtSupportFoot(Configuration& q, EndEffector ee, bool support_foot);
		Eigen::MatrixXf getJacobian(Configuration& q, EndEffector base, EndEffector var);


	private:

		/*
		static auto rm = NULL;
		static auto robots = NULL;
	    static auto & hrp4 = NULL;

		float provaNum;
		*/
		std::shared_ptr<mc_rbdyn::RobotModule> rm;
	    std::shared_ptr<mc_rbdyn::Robots> robots;
	    //mc_rbdyn::Robot hrp4;
		//mc_rbdyn::RobotModule rm;
	    //mc_rbdyn::Robots robots;


		Eigen::MatrixXd rightFoot2CoM(Configuration& q);
		Eigen::MatrixXd leftFoot2CoM(Configuration& q);
		Eigen::MatrixXd rightFoot2Torso(Configuration& q);
		Eigen::MatrixXd leftFoot2Torso(Configuration& q);
		Eigen::MatrixXd rightFoot2RightHand(Configuration& q);
		Eigen::MatrixXd rightFoot2LeftHand(Configuration& q);
		Eigen::MatrixXd rightFoot2Head(Configuration& q);

		Eigen::MatrixXd jacobianRF2LF(Configuration& q);
		Eigen::MatrixXd jacobianLF2RF(Configuration& q);
		Eigen::MatrixXd jacobianRF2LH(Configuration& q);
		Eigen::MatrixXd jacobianLF2LH(Configuration& q);
		Eigen::MatrixXd jacobianRF2RH(Configuration& q);
		Eigen::MatrixXd jacobianLF2RH(Configuration& q);
		Eigen::MatrixXd jacobianRF2CoM(Configuration& q);
		Eigen::MatrixXd jacobianLF2CoM(Configuration& q);

		Eigen::MatrixXd jacobianRF2Torso(Configuration& q);
		Eigen::MatrixXd jacobianLF2Torso(Configuration& q);

		Eigen::MatrixXd rightFoot2TopCamera(Configuration& q);
		Eigen::MatrixXd leftFoot2TopCamera(Configuration& q);
		Eigen::MatrixXd jacobianRF2TopCamera(Configuration& q);
		Eigen::MatrixXd jacobianLF2TopCamera(Configuration& q);

		Eigen::MatrixXd jacobianRF2Head(Configuration& q);
		Eigen::MatrixXd jacobianLF2Head(Configuration& q);
		Robot* robot;


};

#endif	/* HRP4KINEMATICS_H */
