/*
 * File:   Joint.cpp
 * Author: Paolo Ferrari
 *
 * Created on Feb 9, 2016 
 */

#include "Joint.hpp"
#include "v_repLib.h"
#include <iostream>

Joint::Joint(){
	handle = -1;
	jnt_min = 0;
	jnt_max = 0;
	vel_min = 0;
	vel_max = 0;
}

Joint::Joint(unsigned short int h, float jmin, float jmax, float vmin, float vmax){
	handle = h;
	jnt_min = jmin;
	jnt_max = jmax;
	vel_min = vmin;
	vel_max = vmax;
}
		
Joint::~Joint(){
}
		
void Joint::setJointHandle(unsigned short int h){
	handle = h;
}

void Joint::setJointMinValue(float jmin){
	jnt_min = jmin;
}
		
void Joint::setJointMaxValue(float jmax){
	jnt_max = jmax;
}
		
void Joint::setJointMinVelocity(float vmin){
	vel_min = vmin;
}

void Joint::setJointMaxVelocity(float vmax){
	vel_max = vmax;
}

unsigned short int Joint::getJointHandle(){
	return handle;
}
		
float Joint::getJointMinValue(){
	return jnt_min;
}

float Joint::getJointMaxValue(){
	return jnt_max;
}

float Joint::getJointMinVelocity(){
	return vel_min;
}
		
float Joint::getJointMaxVelocity(){
	return vel_max;
}

void Joint::setJointModePassive(unsigned short int handle){
    simSetJointMode(handle, sim_jointmode_passive, 0);
}

void Joint::setJointModeForceTorque(unsigned short int handle){
	simSetJointMode(handle, sim_jointmode_force, 0);
}

void Joint::printJoint(){
	std::cout << handle << std::endl;	
	std::cout << jnt_min << std::endl;	
	std::cout << jnt_max << std::endl;	
	std::cout << vel_min << std::endl;	
	std::cout << vel_max << std::endl;	
}


