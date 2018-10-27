/*
 * File:   Hrp4Kinematics.cpp
 * Author: Paolo Ferrari
 *
 * Created on July 20, 2017
 */

#include "Hrp4Kinematics.hpp"
#include "Configuration.hpp"
#include "enum.h"
#include <iostream>

#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/Robots.h>
#include <RBDyn/FK.h>
#include <RBDyn/IK.h>
#include <RBDyn/FV.h>
#include <RBDyn/Jacobian.h>
#include <RBDyn/MultiBodyGraph.h>
#include <RBDyn/CoM.h>

Hrp4Kinematics::Hrp4Kinematics(){
	rm = mc_rbdyn::RobotLoader::get_robot_module("HRP4NoHand");
  robots = mc_rbdyn::loadRobot(*rm, rm->rsdf_dir);
	robot = new Hrp4();
}

Hrp4Kinematics::~Hrp4Kinematics(){}

Eigen::Matrix4f Hrp4Kinematics::forwardKinematics(Configuration& q, EndEffector ee){
	// Robot* robot;
	// robot = new Hrp4();
	std::vector<std::string> joint_names = robot->getJointNames();
	// 1 ------------------------------------------------------------------------------------
	Eigen::VectorXf vCoM(6);
	vCoM.block<3,1>(0,0) = q.getqCoMPosition();
	vCoM.block<3,1>(3,0) = q.getqCoMOrientation();
	Eigen::Matrix4f TCoM_w = v2t(vCoM);
	// 2 ------------------------------------------------------------------------------------
	auto & hrp4 = robots->robot();
	for(int i = 0; i < joint_names.size(); i++){
		std::string str = joint_names.at(i);
		char* name = new char[str.length() + 1];
		strcpy(name, str.c_str());
		hrp4.mbc().q[hrp4.mb().jointIndexByName(name)][0] = q.getqjntComponent(i);
	}
	rbd::forwardKinematics(hrp4.mb(), hrp4.mbc());
	rbd::forwardVelocity(hrp4.mb(), hrp4.mbc());
	// 3 ------------------------------------------------------------------------------------
	sva::PTransformd PTtorso_w0 = hrp4.mbc().bodyPosW[hrp4.bodyIndexByName("torso")];
	Eigen::Vector3d ptorso_w0 = PTtorso_w0.translation();
	Eigen::Matrix3d Rtorso_w0 = PTtorso_w0.rotation().transpose();
	Eigen::Vector3d etorso_w0 = Rtorso_w0.eulerAngles(0,1,2);
	// 4 ------------------------------------------------------------------------------------
	Eigen::Vector3d pCoM_w0 = rbd::computeCoM(hrp4.mb(), hrp4.mbc());
	Eigen::Vector3d eCoM_w0 = etorso_w0;
	Eigen::VectorXf vCoM_w0(6);
	vCoM_w0 << pCoM_w0(0), pCoM_w0(1), pCoM_w0(2), eCoM_w0(0), eCoM_w0(1), eCoM_w0(2);
	Eigen::MatrixXf TCoM_w0 = v2t(vCoM_w0);
	// 5 ------------------------------------------------------------------------------------
	Eigen::MatrixXf Tw0_CoM = TCoM_w0.inverse();
	// 6 ------------------------------------------------------------------------------------
	sva::PTransformd PTee_w0;
	if(ee == R_HAND) PTee_w0 = hrp4.mbc().bodyPosW[hrp4.bodyIndexByName("r_wrist")];
	else if(ee == L_HAND) PTee_w0 = hrp4.mbc().bodyPosW[hrp4.bodyIndexByName("l_wrist")];
	else if(ee == R_FOOT) PTee_w0 = hrp4.mbc().bodyPosW[hrp4.bodyIndexByName("R_ANKLE_R_LINK")];
	else if(ee == L_FOOT) PTee_w0 = hrp4.mbc().bodyPosW[hrp4.bodyIndexByName("L_ANKLE_R_LINK")];
	else if(ee == TORSO) PTee_w0 = hrp4.mbc().bodyPosW[hrp4.bodyIndexByName("torso")];

	Eigen::Vector3d pee_w0 = PTee_w0.translation();
	Eigen::Matrix3d Ree_w0 = PTee_w0.rotation().transpose();
	Eigen::Vector3d eee_w0 = Ree_w0.eulerAngles(0,1,2);
	Eigen::VectorXf vee_w0(6);
	vee_w0 << pee_w0(0), pee_w0(1), pee_w0(2), eee_w0(0), eee_w0(1), eee_w0(2);
	Eigen::MatrixXf Tee_w0 = v2t(vee_w0);
	// 7 ------------------------------------------------------------------------------------
	Eigen::MatrixXf Tee_CoM = Tw0_CoM * Tee_w0;
	// 8 ------------------------------------------------------------------------------------
	Eigen::MatrixXf Tee_w = TCoM_w * Tee_CoM;

	// 9 ------------------------------------------------------------------------------------
	Eigen::VectorXf v(6);
	v.setZero();
	if(ee == R_HAND) v << -0.0054, 0.0055, -0.071, 0.0, 0.0, 0.0;
	else if(ee == L_HAND) v << -0.0054, -0.0055, -0.071, 0.0, 0.0, 0.0;
	else if(ee == R_FOOT) v << 0.02, -0.01, -0.084, 0.02107, -0.00907, 0.00017;
	else if(ee == L_FOOT) v << 0.02, 0.01, -0.084, -0.00092, -0.01011, -0.00015;
	Eigen::MatrixXf Tf_ee = v2t(v);
	Tee_w = Tee_w * Tf_ee;

	return Tee_w;

}

Eigen::Matrix4f Hrp4Kinematics::forwardKinematicsWrtTorso(Configuration& q, EndEffector ee){
	Robot* robot;
	robot = new Hrp4();
	std::vector<std::string> joint_names = robot->getJointNames();
	// 1 ------------------------------------------------------------------------------------
    auto & hrp4 = robots->robot();
	for(int i = 0; i < joint_names.size(); i++){
		std::string str = joint_names.at(i);
		char* name = new char[str.length() + 1];
		strcpy(name, str.c_str());
		hrp4.mbc().q[hrp4.mb().jointIndexByName(name)][0] = q.getqjntComponent(i);
	}
	rbd::forwardKinematics(hrp4.mb(), hrp4.mbc());
	rbd::forwardVelocity(hrp4.mb(), hrp4.mbc());
	// 2 ------------------------------------------------------------------------------------
	sva::PTransformd PTtorso_w0 = hrp4.mbc().bodyPosW[hrp4.bodyIndexByName("torso")];
	Eigen::Vector3d ptorso_w0 = PTtorso_w0.translation();
	Eigen::Matrix3d Rtorso_w0 = PTtorso_w0.rotation().transpose();
	Eigen::Vector3d etorso_w0 = Rtorso_w0.eulerAngles(0,1,2);
	Eigen::VectorXf vtorso_w0(6);
	vtorso_w0 << ptorso_w0(0), ptorso_w0(1), ptorso_w0(2), etorso_w0(0), etorso_w0(1), etorso_w0(2);
	Eigen::MatrixXf Ttorso_w0 = v2t(vtorso_w0);
	// 3 ------------------------------------------------------------------------------------
	sva::PTransformd PTee_w0;
	if(ee == R_HAND) PTee_w0 = hrp4.mbc().bodyPosW[hrp4.bodyIndexByName("r_wrist")];
	else if(ee == L_HAND) PTee_w0 = hrp4.mbc().bodyPosW[hrp4.bodyIndexByName("l_wrist")];
	else if(ee == R_FOOT) PTee_w0 = hrp4.mbc().bodyPosW[hrp4.bodyIndexByName("R_ANKLE_R_LINK")];
	else if(ee == L_FOOT) PTee_w0 = hrp4.mbc().bodyPosW[hrp4.bodyIndexByName("L_ANKLE_R_LINK")];
	Eigen::Vector3d pee_w0 = PTee_w0.translation();
	Eigen::Matrix3d Ree_w0 = PTee_w0.rotation().transpose();
	Eigen::Vector3d eee_w0 = Ree_w0.eulerAngles(0,1,2);
	Eigen::VectorXf vee_w0(6);
	vee_w0 << pee_w0(0), pee_w0(1), pee_w0(2), eee_w0(0), eee_w0(1), eee_w0(2);
	Eigen::MatrixXf Tee_w0 = v2t(vee_w0);
	// 4 ------------------------------------------------------------------------------------
	Eigen::MatrixXf Tw0_torso = Ttorso_w0.inverse();
	// 5 ------------------------------------------------------------------------------------
	Eigen::MatrixXf Tee_torso = Tw0_torso * Tee_w0;
	// 6 ------------------------------------------------------------------------------------
	Eigen::VectorXf v(6);
	v.setZero();
	if(ee == R_HAND) v << -0.0054, 0.0055, -0.071, 0.0, 0.0, 0.0;
	else if(ee == L_HAND) v << -0.0054, -0.0055, -0.071, 0.0, 0.0, 0.0;
	else if(ee == R_FOOT) v << 0.02, -0.01, -0.084, 0.02107, -0.00907, 0.00017;
	else if(ee == L_FOOT) v << 0.02, 0.01, -0.084, -0.00092, -0.01011, -0.00015;
	Eigen::MatrixXf Tf_ee = v2t(v);
	Tee_torso = Tee_torso * Tf_ee;

	return Tee_torso;
}

Eigen::Matrix4f Hrp4Kinematics::forwardKinematicsWrtCoM(Configuration& q, EndEffector ee){
	Robot* robot;
	robot = new Hrp4();
	std::vector<std::string> joint_names = robot->getJointNames();
	// 1 ------------------------------------------------------------------------------------
    auto & hrp4 = robots->robot();
	for(int i = 0; i < joint_names.size(); i++){
		std::string str = joint_names.at(i);
		char* name = new char[str.length() + 1];
		strcpy(name, str.c_str());
		hrp4.mbc().q[hrp4.mb().jointIndexByName(name)][0] = q.getqjntComponent(i);
	}
	rbd::forwardKinematics(hrp4.mb(), hrp4.mbc());
	rbd::forwardVelocity(hrp4.mb(), hrp4.mbc());
	// 2 ------------------------------------------------------------------------------------
	sva::PTransformd PTtorso_w0 = hrp4.mbc().bodyPosW[hrp4.bodyIndexByName("torso")];
	Eigen::Vector3d ptorso_w0 = PTtorso_w0.translation();
	Eigen::Matrix3d Rtorso_w0 = PTtorso_w0.rotation().transpose();
	Eigen::Vector3d etorso_w0 = Rtorso_w0.eulerAngles(0,1,2);
	// 3 ------------------------------------------------------------------------------------
	Eigen::Vector3d pCoM_w0 = rbd::computeCoM(hrp4.mb(), hrp4.mbc());
	Eigen::Vector3d eCoM_w0 = etorso_w0;
	Eigen::VectorXf vCoM_w0(6);
	vCoM_w0 << pCoM_w0(0), pCoM_w0(1), pCoM_w0(2), eCoM_w0(0), eCoM_w0(1), eCoM_w0(2);
	Eigen::MatrixXf TCoM_w0 = v2t(vCoM_w0);
	// 4 ------------------------------------------------------------------------------------
	Eigen::MatrixXf Tw0_CoM = TCoM_w0.inverse();
	// 5 ------------------------------------------------------------------------------------
	sva::PTransformd PTee_w0;
	if(ee == R_HAND) PTee_w0 = hrp4.mbc().bodyPosW[hrp4.bodyIndexByName("r_wrist")];
	else if(ee == L_HAND) PTee_w0 = hrp4.mbc().bodyPosW[hrp4.bodyIndexByName("l_wrist")];
	else if(ee == R_FOOT) PTee_w0 = hrp4.mbc().bodyPosW[hrp4.bodyIndexByName("R_ANKLE_R_LINK")];
	else if(ee == L_FOOT) PTee_w0 = hrp4.mbc().bodyPosW[hrp4.bodyIndexByName("L_ANKLE_R_LINK")];
	Eigen::Vector3d pee_w0 = PTee_w0.translation();
	Eigen::Matrix3d Ree_w0 = PTee_w0.rotation().transpose();
	Eigen::Vector3d eee_w0 = Ree_w0.eulerAngles(0,1,2);
	Eigen::VectorXf vee_w0(6);
	vee_w0 << pee_w0(0), pee_w0(1), pee_w0(2), eee_w0(0), eee_w0(1), eee_w0(2);
	Eigen::MatrixXf Tee_w0 = v2t(vee_w0);
	// 6 ------------------------------------------------------------------------------------
	Eigen::MatrixXf Tee_CoM = Tw0_CoM * Tee_w0;
	// 7 ------------------------------------------------------------------------------------
	Eigen::VectorXf v(6);
	v.setZero();
	if(ee == R_HAND) v << -0.0054, 0.0055, -0.071, 0.0, 0.0, 0.0;
	else if(ee == L_HAND) v << -0.0054, -0.0055, -0.071, 0.0, 0.0, 0.0;
	else if(ee == R_FOOT) v << 0.02, -0.01, -0.084, 0.02107, -0.00907, 0.00017;
	else if(ee == L_FOOT) v << 0.02, 0.01, -0.084, -0.00092, -0.01011, -0.00015;
	Eigen::MatrixXf Tf_ee = v2t(v);
	Tee_CoM = Tee_CoM * Tf_ee;

	return Tee_CoM;
}

Eigen::VectorXf Hrp4Kinematics::forwardKinematicsWrtSupportFoot(Configuration& q, EndEffector ee, bool support_foot){

	Eigen::VectorXf vee_Sup(6);
	Eigen::VectorXf vee_w(6);
	Eigen::Matrix4f Tee_w;
	if(ee==COM){
		vee_w<<q.getqCoMPosition(), q.getqCoMOrientation();
		Tee_w=v2t(vee_w);
	}
	else
		Tee_w=forwardKinematics(q, ee);
	//std::cout<<Tee_w.col(3).transpose()<<std::endl;
	Eigen::Matrix4f Tw_sup;
	if(support_foot==true)
		Tw_sup =forwardKinematics(q,R_FOOT).inverse();
	else
		Tw_sup =forwardKinematics(q,L_FOOT).inverse();

	vee_Sup=t2v(Tw_sup*Tee_w);

	return vee_Sup;

}

Eigen::MatrixXf Hrp4Kinematics::getJacobian(Configuration& q, EndEffector base, EndEffector var){
	Eigen::MatrixXf Tbase_w = this->forwardKinematics(q, base);

	Eigen::VectorXf vbase_base0(6);
	vbase_base0.setZero();
	if(base == R_HAND) vbase_base0 << -0.0054, 0.0055, -0.071, 0.0, 0.0, 0.0;
	else if(base == L_HAND) vbase_base0 << -0.0054, -0.0055, -0.071, 0.0, 0.0, 0.0;
	else if(base == R_FOOT) vbase_base0 << 0.02, -0.01, -0.084, 0.02107, -0.00907, 0.00017;
	else if(base == L_FOOT) vbase_base0 << 0.02, 0.01, -0.084, -0.00092, -0.01011, -0.00015;
	Eigen::MatrixXf Tbase_base0 = v2t(vbase_base0);

	Eigen::MatrixXf Tbase0_base = Tbase_base0.inverse();

	Eigen::MatrixXf Tbase0_w = Tbase_w * Tbase0_base;
	Eigen::Matrix3d Rbase0_w = Tbase0_w.block(0,0,3,3).cast<double>();
	Eigen::Quaterniond qbase0_w(Rbase0_w);
	Eigen::Vector3d pbase0_w = Tbase0_w.block(0,3,3,1).cast<double>();

	// Robot* robot;
	// robot = new Hrp4();
	std::vector<std::string> joint_names = robot->getJointNames();
	auto & hrp4 = robots->robot();
	for(int i = 0; i < joint_names.size(); i++){
		std::string str = joint_names.at(i);
		char* name = new char[str.length() + 1];
		strcpy(name, str.c_str());
		hrp4.mbc().q[hrp4.mb().jointIndexByName(name)][0] = q.getqjntComponent(i);
	}
	rbd::forwardKinematics(hrp4.mb(), hrp4.mbc());
	rbd::forwardVelocity(hrp4.mb(), hrp4.mbc());

	rbd::MultiBody hrp4_mb_base0;
	if(base == R_HAND) hrp4_mb_base0 = hrp4.mbg().makeMultiBody("r_wrist", false);
	else if(base == L_HAND) hrp4_mb_base0 = hrp4.mbg().makeMultiBody("l_wrist", false);
	else if(base == R_FOOT) hrp4_mb_base0 = hrp4.mbg().makeMultiBody("R_ANKLE_R_LINK", false);
	else if(base == L_FOOT) hrp4_mb_base0 = hrp4.mbg().makeMultiBody("L_ANKLE_R_LINK", false);

	rbd::MultiBodyConfig hrp4_mbc_base0 = rbd::MultiBodyConfig(hrp4_mb_base0);
	rbd::ConfigConverter cc2(hrp4.mb(), hrp4_mb_base0);
	cc2.convert(hrp4.mbc(), hrp4_mbc_base0);
	hrp4_mbc_base0.q[0] = {qbase0_w.w(), qbase0_w.x(), qbase0_w.y(), qbase0_w.z(), pbase0_w.x(), pbase0_w.y(), pbase0_w.z()};
	rbd::forwardKinematics(hrp4_mb_base0, hrp4_mbc_base0);
	rbd::forwardVelocity(hrp4_mb_base0, hrp4_mbc_base0);

	Eigen::MatrixXd Jvar0_w;
	std::vector<int> joints_path;
	std::vector<std::string> joint_names_in_path;
	Eigen::VectorXi colPos(30);
	if(var == R_HAND){
		//rbd::Jacobian J(hrp4_mb_base0, "r_wrist");
		Eigen::Vector3d p(-0.0054, 0.0055, -0.071);
		rbd::Jacobian J(hrp4_mb_base0, "r_wrist", p);

		Jvar0_w = J.jacobian(hrp4_mb_base0, hrp4_mbc_base0);
		joints_path = J.jointsPath();
		for(int i = 0; i < joints_path.size(); i++){
			std::string str = hrp4_mb_base0.joint(joints_path[i]).name();
			char* name = new char[str.length() + 1];
			strcpy(name, str.c_str());
			joint_names_in_path.push_back(name);
		}
		///////////////////////////////////////////////////////////////////////////////////
		for(int i = 0; i < joint_names.size(); i++){
			int pos = -1;
			std::string str = joint_names.at(i);
			for(int j = 0; j < joint_names_in_path.size(); j++){
				std::string strPath = joint_names_in_path.at(j);
				if(str.compare(strPath) == 0) pos = j+5;
			}
			colPos(i) = pos;
		}
		///////////////////////////////////////////////////////////////////////////////////
	}
	else if(var == L_HAND){
		//rbd::Jacobian J(hrp4_mb_base0, "l_wrist");
		Eigen::Vector3d p(-0.0054, -0.0055, -0.071);
		rbd::Jacobian J(hrp4_mb_base0, "l_wrist", p);

		Jvar0_w = J.jacobian(hrp4_mb_base0, hrp4_mbc_base0);
		joints_path = J.jointsPath();
		for(int i = 0; i < joints_path.size(); i++){
			std::string str = hrp4_mb_base0.joint(joints_path[i]).name();
			char* name = new char[str.length() + 1];
			strcpy(name, str.c_str());
			joint_names_in_path.push_back(name);
		}
		///////////////////////////////////////////////////////////////////////////////////
		for(int i = 0; i < joint_names.size(); i++){
			int pos = -1;
			std::string str = joint_names.at(i);
			for(int j = 0; j < joint_names_in_path.size(); j++){
				std::string strPath = joint_names_in_path.at(j);
				if(str.compare(strPath) == 0) pos = j+5;
			}
			colPos(i) = pos;
		}
		///////////////////////////////////////////////////////////////////////////////////
	}
	else if(var == R_FOOT){
		//rbd::Jacobian J(hrp4_mb_base0, "R_ANKLE_R_LINK");
		Eigen::Vector3d p(0.02, -0.01, -0.084);
		rbd::Jacobian J(hrp4_mb_base0, "R_ANKLE_R_LINK", p);

		Jvar0_w = J.jacobian(hrp4_mb_base0, hrp4_mbc_base0);
		joints_path = J.jointsPath();
		for(int i = 0; i < joints_path.size(); i++){
			std::string str = hrp4_mb_base0.joint(joints_path[i]).name();
			char* name = new char[str.length() + 1];
			strcpy(name, str.c_str());
			joint_names_in_path.push_back(name);
		}
		///////////////////////////////////////////////////////////////////////////////////
		for(int i = 0; i < joint_names.size(); i++){
			int pos = -1;
			std::string str = joint_names.at(i);
			for(int j = 0; j < joint_names_in_path.size(); j++){
				std::string strPath = joint_names_in_path.at(j);
				if(str.compare(strPath) == 0) pos = j+5;
			}
			colPos(i) = pos;
		}
		///////////////////////////////////////////////////////////////////////////////////
	}
	else if(var == L_FOOT){
		//rbd::Jacobian J(hrp4_mb_base0, "L_ANKLE_R_LINK");
		Eigen::Vector3d p(0.02, 0.01, -0.084);
		rbd::Jacobian J(hrp4_mb_base0, "L_ANKLE_R_LINK", p);

		Jvar0_w = J.jacobian(hrp4_mb_base0, hrp4_mbc_base0);
		joints_path = J.jointsPath();
		for(int i = 0; i < joints_path.size(); i++){
			std::string str = hrp4_mb_base0.joint(joints_path[i]).name();
			char* name = new char[str.length() + 1];
			strcpy(name, str.c_str());
			joint_names_in_path.push_back(name);
		}
		///////////////////////////////////////////////////////////////////////////////////
		for(int i = 0; i < joint_names.size(); i++){
			int pos = -1;
			std::string str = joint_names.at(i);
			for(int j = 0; j < joint_names_in_path.size(); j++){
				std::string strPath = joint_names_in_path.at(j);
				if(str.compare(strPath) == 0) pos = j+5;
			}
			colPos(i) = pos;
		}
		///////////////////////////////////////////////////////////////////////////////////
	}
	else if(var == TORSO){
		rbd::Jacobian J(hrp4_mb_base0, "torso");
		Jvar0_w = J.jacobian(hrp4_mb_base0, hrp4_mbc_base0);
		joints_path = J.jointsPath();
		for(int i = 0; i < joints_path.size(); i++){
			std::string str = hrp4_mb_base0.joint(joints_path[i]).name();
			char* name = new char[str.length() + 1];
			strcpy(name, str.c_str());
			joint_names_in_path.push_back(name);
		}
		///////////////////////////////////////////////////////////////////////////////////
		for(int i = 0; i < joint_names.size(); i++){
			int pos = -1;
			std::string str = joint_names.at(i);
			for(int j = 0; j < joint_names_in_path.size(); j++){
				std::string strPath = joint_names_in_path.at(j);
				if(str.compare(strPath) == 0) pos = j+5;
			}
			colPos(i) = pos;
		}
		///////////////////////////////////////////////////////////////////////////////////
	}
	else if(var == COM){
		rbd::CoMJacobian J(hrp4_mb_base0);
		Jvar0_w = J.jacobian(hrp4_mb_base0, hrp4_mbc_base0);
		for(int i = 0; i < joint_names.size(); i++){
			std::string str = joint_names.at(i);
			char* name = new char[str.length() + 1];
			strcpy(name, str.c_str());
			colPos(i) = hrp4_mb_base0.jointPosInDof(hrp4_mb_base0.jointIndexByName(name));
		}
	}

	int nrows = Jvar0_w.rows();
	int ncols = 30;
	Eigen::MatrixXf JFvar0_w(nrows, ncols);  // Full Jacobian
	JFvar0_w.setZero();
	for(int i = 0; i < ncols; i++){
		if(colPos(i) != -1){
			int j = colPos(i);
			JFvar0_w.block(0,i,nrows,1) = Jvar0_w.block(0,j,nrows,1).cast<float>();
		}
	}

	Eigen::Matrix3f Rw_base0 = Rbase0_w.inverse().cast<float>();
	Eigen::MatrixXf RFw_base0;
	if(nrows == 3){
		RFw_base0.resize(3,3);
		RFw_base0 = Rw_base0;
	}
	else{
		RFw_base0.resize(6,6);
		RFw_base0.setZero();
		RFw_base0.block(0,0,3,3) = Rw_base0;
		RFw_base0.block(3,3,3,3) = Rw_base0;
	}
	Eigen::MatrixXf JFvar0_base0 = RFw_base0 * JFvar0_w;

	Eigen::Matrix3f Rbase0_base = Tbase0_base.block(0,0,3,3);
	Eigen::MatrixXf RFbase0_base;
	if(nrows == 3){
		RFbase0_base.resize(3,3);
		RFbase0_base = Rbase0_base;
	}
	else{
		RFbase0_base.resize(6,6);
		RFbase0_base.setZero();
		RFbase0_base.block(0,0,3,3) = Rbase0_base;
		RFbase0_base.block(3,3,3,3) = Rbase0_base;
	}
	Eigen::MatrixXf JFvar0_base = RFbase0_base * JFvar0_base0;

	Eigen::MatrixXf Jres;
	if(nrows == 3){
		Jres.resize(3,30);
		Jres = JFvar0_base;
	}
	else{
		Jres.resize(6,30);
		Jres.block(0,0,3,30) = JFvar0_base.block(3,0,3,30);
		Jres.block(3,0,3,30) = JFvar0_base.block(0,0,3,30);
	}

	return Jres;
}

Eigen::MatrixXd Hrp4Kinematics::rightFoot2CoM(Configuration& q){}
Eigen::MatrixXd Hrp4Kinematics::leftFoot2CoM(Configuration& q){}
Eigen::MatrixXd Hrp4Kinematics::rightFoot2Torso(Configuration& q){}
Eigen::MatrixXd Hrp4Kinematics::leftFoot2Torso(Configuration& q){}
Eigen::MatrixXd Hrp4Kinematics::rightFoot2RightHand(Configuration& q){}
Eigen::MatrixXd Hrp4Kinematics::rightFoot2LeftHand(Configuration& q){}
Eigen::MatrixXd Hrp4Kinematics::rightFoot2Head(Configuration& q){}

Eigen::MatrixXd Hrp4Kinematics::jacobianRF2LF(Configuration& q){}
Eigen::MatrixXd Hrp4Kinematics::jacobianLF2RF(Configuration& q){}
Eigen::MatrixXd Hrp4Kinematics::jacobianRF2LH(Configuration& q){}
Eigen::MatrixXd Hrp4Kinematics::jacobianLF2LH(Configuration& q){}
Eigen::MatrixXd Hrp4Kinematics::jacobianRF2RH(Configuration& q){}
Eigen::MatrixXd Hrp4Kinematics::jacobianLF2RH(Configuration& q){}
Eigen::MatrixXd Hrp4Kinematics::jacobianRF2CoM(Configuration& q){}
Eigen::MatrixXd Hrp4Kinematics::jacobianLF2CoM(Configuration& q){}

Eigen::MatrixXd Hrp4Kinematics::jacobianRF2Torso(Configuration& q){}
Eigen::MatrixXd Hrp4Kinematics::jacobianLF2Torso(Configuration& q){}

Eigen::MatrixXd Hrp4Kinematics::rightFoot2TopCamera(Configuration& q){}
Eigen::MatrixXd Hrp4Kinematics::leftFoot2TopCamera(Configuration& q){}
Eigen::MatrixXd Hrp4Kinematics::jacobianRF2TopCamera(Configuration& q){}
Eigen::MatrixXd Hrp4Kinematics::jacobianLF2TopCamera(Configuration& q){}

Eigen::MatrixXd Hrp4Kinematics::jacobianRF2Head(Configuration& q){}
Eigen::MatrixXd Hrp4Kinematics::jacobianLF2Head(Configuration& q){}
