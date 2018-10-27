/*
 * File:   Hrp4.cpp
 * Author: Paolo Ferrari
 *
 * Created on July 20, 2017
 */


#include "Hrp4.hpp"
#include "Joint.hpp"
#include "Configuration.hpp"
#include <sstream>
#include <iostream>
#include <string.h>
#include <vector>
#include "v_repLib.h"
#include "Eigen/Core"

#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/Robots.h>
#include <RBDyn/FK.h>
#include <RBDyn/IK.h>
#include <RBDyn/FV.h>
#include <RBDyn/Jacobian.h>
#include <RBDyn/MultiBodyGraph.h>
#include <RBDyn/CoM.h>

Hrp4::Hrp4(){  
	
	//std::vector<std::string> joint_names;
	joint_names.push_back("NECK_Y");
	joint_names.push_back("NECK_P");
	joint_names.push_back("R_SHOULDER_P");
	joint_names.push_back("R_SHOULDER_R"); 
	joint_names.push_back("R_SHOULDER_Y");
	joint_names.push_back("R_ELBOW_P");
	joint_names.push_back("R_WRIST_Y");
	joint_names.push_back("R_WRIST_P");
	joint_names.push_back("R_WRIST_R");
	joint_names.push_back("L_SHOULDER_P");
	joint_names.push_back("L_SHOULDER_R");
	joint_names.push_back("L_SHOULDER_Y");
	joint_names.push_back("L_ELBOW_P");
	joint_names.push_back("L_WRIST_Y");
	joint_names.push_back("L_WRIST_P");
	joint_names.push_back("L_WRIST_R");
	joint_names.push_back("CHEST_P");
	joint_names.push_back("CHEST_Y");
	joint_names.push_back("R_HIP_Y");
	joint_names.push_back("R_HIP_R");
	joint_names.push_back("R_HIP_P");
	joint_names.push_back("R_KNEE_P");
	joint_names.push_back("R_ANKLE_P");
	joint_names.push_back("R_ANKLE_R");
	joint_names.push_back("L_HIP_Y");
	joint_names.push_back("L_HIP_R");
	joint_names.push_back("L_HIP_P");
	joint_names.push_back("L_KNEE_P");
	joint_names.push_back("L_ANKLE_P");
	joint_names.push_back("L_ANKLE_R");

	std::string str;
	char* name;
	int h;
	simFloat range[2];
	simBool cyclic = false;
	for(int i = 0; i < joint_names.size(); i++){
		str = joint_names.at(i);
		name = new char[str.length() + 1];
		strcpy(name, str.c_str());
		h = simGetObjectHandle(name);		
		delete [] name;
		simGetJointInterval(h, &cyclic, range);
		Joint jnt(h, range[0], range[0]+range[1], 0.0, 0.0);    
		joints.push_back(jnt);
	}	

}
	
Hrp4::~Hrp4(){}

void Hrp4::setConfigurationStat(Configuration& q){

	Eigen::Vector3f pCoM_w = q.getqCoMPosition();
	Eigen::Vector3f eCoM_w = q.getqCoMOrientation();
	Eigen::VectorXf vCoM_w(6);
	vCoM_w << pCoM_w(0), pCoM_w(1), pCoM_w(2), eCoM_w(0), eCoM_w(1), eCoM_w(2); 
	Eigen::MatrixXf TCoM_w = v2t(vCoM_w); 
	
	auto rm = mc_rbdyn::RobotLoader::get_robot_module("HRP4NoHand");
    auto robots = mc_rbdyn::loadRobot(*rm, rm->rsdf_dir);
    auto & hrp4 = robots->robot();
	for(int i = 0; i < joint_names.size(); i++){
		std::string str = joint_names.at(i);
		char* name = new char[str.length() + 1];
		strcpy(name, str.c_str());				
		hrp4.mbc().q[hrp4.mb().jointIndexByName(name)][0] = q.getqjntComponent(i);	
	}
	rbd::forwardKinematics(hrp4.mb(), hrp4.mbc());
	rbd::forwardVelocity(hrp4.mb(), hrp4.mbc());

	sva::PTransformd PTb_w0 = hrp4.mbc().bodyPosW[hrp4.bodyIndexByName("base_link")];
	Eigen::Vector3d pb_w0 = PTb_w0.translation();
	Eigen::Matrix3d Rb_w0 = PTb_w0.rotation().transpose();
	Eigen::Vector3d eb_w0 = Rb_w0.eulerAngles(0,1,2); 	
	Eigen::VectorXf vb_w0(6);
	vb_w0 << pb_w0(0), pb_w0(1), pb_w0(2), eb_w0(0), eb_w0(1), eb_w0(2);
	Eigen::MatrixXf Tb_w0 = v2t(vb_w0);

	sva::PTransformd PTtorso_w0 = hrp4.mbc().bodyPosW[hrp4.bodyIndexByName("torso")];
	Eigen::Vector3d ptorso_w0 = PTtorso_w0.translation();
	Eigen::Matrix3d Rtorso_w0 = PTtorso_w0.rotation().transpose();
	Eigen::Vector3d etorso_w0 = Rtorso_w0.eulerAngles(0,1,2); 	
	Eigen::VectorXf vtorso_w0(6);
	vtorso_w0 << ptorso_w0(0), ptorso_w0(1), ptorso_w0(2), etorso_w0(0), etorso_w0(1), etorso_w0(2);
	Eigen::MatrixXf Ttorso_w0 = v2t(vtorso_w0);
	
	Eigen::Vector3f pCoM_w0 = rbd::computeCoM(hrp4.mb(), hrp4.mbc()).cast<float>();
	Eigen::Matrix3f RCoM_w0 = Rtorso_w0.cast<float>();  // same rotation of torso
	Eigen::Vector3f eCoM_w0 = RCoM_w0.eulerAngles(0,1,2); 
	Eigen::VectorXf vCoM_w0(6);
	vCoM_w0 << pCoM_w0(0), pCoM_w0(1), pCoM_w0(2), eCoM_w0(0), eCoM_w0(1), eCoM_w0(2);
	Eigen::MatrixXf TCoM_w0 = v2t(vCoM_w0);
	
	Eigen::MatrixXf Tw0_b = Tb_w0.inverse(); 
	
	Eigen::MatrixXf TCoM_b = Tw0_b * TCoM_w0; 
	
	Eigen::MatrixXf Tb_CoM = TCoM_b.inverse(); 
	
	Eigen::MatrixXf Tb_w = TCoM_w * Tb_CoM; 
	Eigen::VectorXf vb_w = t2v(Tb_w);

	simFloat posBase[3] = {vb_w(0), vb_w(1), vb_w(2)};
	simFloat eulBase[3] = {vb_w(3), vb_w(4), vb_w(5)};
	simInt base_handle = simGetObjectHandle("base_link_visual");
	simSetObjectPosition(base_handle, -1, posBase);
	simSetObjectOrientation(base_handle, -1, eulBase);

	this->setJointsConfiguration(q);
	
}

void Hrp4::setJointsConfiguration(Configuration& q){

	simInt h;
	for(int i = 0; i < joints.size(); i++){
		h = joints.at(i).getJointHandle();
		simSetJointMode(h, sim_jointmode_passive, 0);
		simSetJointPosition(h, q.getqjntComponent(i));	
	}

}

void Hrp4::setConfigurationDyn(Configuration& q){

	simInt h;
	for(int i = 0; i < joints.size(); i++){
		h = joints.at(i).getJointHandle();
		simSetJointMode(h, sim_jointmode_force, 0);  //
		simSetJointTargetPosition(h, q.getqjntComponent(i));	
	}

}


Configuration Hrp4::getCurrentConfiguration(){
	
	int dof = joints.size();
	Configuration q(dof);
	Eigen::VectorXf qjnt(joint_names.size());		
	
	for(int i = 0; i < dof; i++){
		simInt h = joints.at(i).getJointHandle();
		simFloat val;		
		simGetJointPosition(h, &val);
		q.setqjntComponent(i,val);	
		qjnt(i) = val;
	}		
	
	auto rm = mc_rbdyn::RobotLoader::get_robot_module("HRP4NoHand");
    auto robots = mc_rbdyn::loadRobot(*rm, rm->rsdf_dir);
    auto & hrp4 = robots->robot();
	
	float qb_w[4], pb_w[3];
	simGetObjectQuaternion(simGetObjectHandle("base_link_visual"),-1, qb_w);
	simGetObjectPosition(simGetObjectHandle("base_link_visual"),-1, pb_w);	
	Eigen::Quaterniond qb_wE(qb_w[3], qb_w[0], qb_w[1], qb_w[2]);
	hrp4.mbc().q[0] = {qb_wE.w(), qb_wE.x(), qb_wE.y(), qb_wE.z(), pb_w[0], pb_w[1], pb_w[2]};
	for(int i = 0; i < joint_names.size(); i++){
		std::string str = joint_names.at(i);
		char* name = new char[str.length() + 1];
		strcpy(name, str.c_str());				
		hrp4.mbc().q[hrp4.mb().jointIndexByName(name)][0] = qjnt(i);	
	}
	rbd::forwardKinematics(hrp4.mb(), hrp4.mbc());
	rbd::forwardVelocity(hrp4.mb(), hrp4.mbc());
	
	sva::PTransformd PTtorso_w = hrp4.mbc().bodyPosW[hrp4.bodyIndexByName("torso")];
	Eigen::Vector3d ptorso_w = PTtorso_w.translation();
	Eigen::Matrix3d Rtorso_w = PTtorso_w.rotation().transpose();
	
	Eigen::Vector3f pCoM_w = rbd::computeCoM(hrp4.mb(), hrp4.mbc()).cast<float>();
	Eigen::Matrix3f RCoM_w = Rtorso_w.cast<float>();
	Eigen::Vector3f eCoM_w = RCoM_w.eulerAngles(0,1,2); 
	q.setqCoMPosition(pCoM_w);
	q.setqCoMOrientation(eCoM_w);
	
	return q;
}


std::vector<Joint> Hrp4::getJoints(){
	return joints;
}

std::vector<std::string> Hrp4::getJointNames(){
	return joint_names;
} 




