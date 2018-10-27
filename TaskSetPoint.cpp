/*
 * File:   TaskSetPoint.cpp
 * Author: Paolo Ferrari
 *
 * Created on January 31, 2017
 */


#include "TaskSetPoint.hpp"

TaskSetPoint::TaskSetPoint(){
	type = TASK_SETPOINT;	
	lastReached = -1;	
}

TaskSetPoint::~TaskSetPoint(){}

Eigen::Vector3f TaskSetPoint::getPosition(float k){
	Eigen::Vector3f taskPosition;	
	taskPosition = atts.at(lastReached + 1);
	return taskPosition;
}

Eigen::Vector3f TaskSetPoint::getVelocity(float k){
	Eigen::Vector3f taskVelocity;
	taskVelocity.setZero();
	return taskVelocity; 
}

void TaskSetPoint::defineTask(Eigen::Vector3f _init){}
void TaskSetPoint::addAttractorTime(float _t_att){}
float TaskSetPoint::getAttractorTime(float i){return -1;}
float TaskSetPoint::getDuration(){return -1;}
float TaskSetPoint::computeClosestPoint(Eigen::VectorXf vr, float si){return -1;}	


