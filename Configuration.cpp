/* 
 * File:   Configuration.hpp
 * Author: Paolo Ferrari
 *
 * Created on February 14, 2016 
 */

#include "Configuration.hpp"
#include <stdlib.h>
#include "Eigen/Core"

Configuration::Configuration(){ }

Configuration::Configuration(int dof){
	for(int i = 0; i < q_CoM.rows(); i++) q_CoM(i) = 0.0;	
	q_jnt.resize(dof);	
	for(int i = 0; i < dof; i++) q_jnt(i) = 0.0;
}

Configuration::~Configuration(){ }

void Configuration::setqCoMPosition(Eigen::Vector3f p){
	q_CoM(0) = p(0);
	q_CoM(1) = p(1);
	q_CoM(2) = p(2);
}

void Configuration::setqCoMOrientation(Eigen::Vector3f o){
	q_CoM(3) = o(0);
	q_CoM(4) = o(1);
	q_CoM(5) = o(2);
}

Eigen::Vector3f Configuration::getqCoMPosition(){
	Eigen::Vector3f p;
	p(0) = q_CoM(0);
	p(1) = q_CoM(1);
	p(2) = q_CoM(2);
	return p;
}

Eigen::Vector3f Configuration::getqCoMOrientation(){
	Eigen::Vector3f o;
	o(0) = q_CoM(3);
	o(1) = q_CoM(4);
	o(2) = q_CoM(5);
	return o;
}

void Configuration::setqjntComponent(int i, float val){
	q_jnt(i) = val;
}

float Configuration::getqjntComponent(int i){
	return q_jnt(i);
}

Eigen::VectorXf Configuration::getqjnt(){
	return q_jnt;
}

void Configuration::setqjnt(Eigen::VectorXf _qjnt){
	q_jnt.resize(_qjnt.size());
	for(int i = 0; i < _qjnt.size(); i++) q_jnt(i) = _qjnt(i); 
}


