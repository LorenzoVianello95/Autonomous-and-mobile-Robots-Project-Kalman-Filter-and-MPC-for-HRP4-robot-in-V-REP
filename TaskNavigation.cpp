/*
 * File:   TaskNavigation.cpp
 * Author: Paolo Ferrari
 *
 * Created on January 31, 2017
 */


#include "TaskNavigation.hpp"

TaskNavigation::TaskNavigation(){
	type = TASK_NAVIGATION;
	lastReached = -1;	
}

TaskNavigation::~TaskNavigation(){}

Eigen::Vector3f TaskNavigation::getPosition(float k){
	Eigen::Vector3f taskPosition;
	taskPosition = atts.at(lastReached + 1);
	return taskPosition;
}

Eigen::Vector3f TaskNavigation::getVelocity(float k){
	Eigen::Vector3f taskVelocity;
	taskVelocity.setZero();
	return taskVelocity; 
}

void TaskNavigation::defineTask(Eigen::Vector3f _init){}
void TaskNavigation::addAttractorTime(float _t_att){}
float TaskNavigation::getAttractorTime(float i){return -1;}
float TaskNavigation::getDuration(){return -1;}
float TaskNavigation::computeClosestPoint(Eigen::VectorXf vr, float si){return -1;}	
