/*
 * File:   TaskTrajectoryVisual.cpp
 * Author: Paolo Ferrari
 *
 * Created on February 14, 2017
 */


#include "TaskTrajectoryVisual.hpp"
#include <boost/lexical_cast.hpp>

TaskTrajectoryVisual::TaskTrajectoryVisual(){
	type = TASK_TRAJECTORY_VISUAL;	
	lastReached = -1;
}

TaskTrajectoryVisual::~TaskTrajectoryVisual(){} 

Eigen::Vector3f TaskTrajectoryVisual::getPosition(float k){
	float t = k;
	Eigen::Vector3f taskPosition;
	std::map<float, Eigen::Vector3f>::iterator it;

	if(!taskPos.empty()){
		it = taskPos.end();
		it--;
		float last_t = it->first;
		if(t >= last_t) return atts.at(atts.size() - 1);
	}
	it = taskPos.begin();
	float tc = it->first;
	while(tc < t){
		it++;
		tc = it->first;
	}	
	taskPosition = it->second;
	return taskPosition;
}

Eigen::Vector3f TaskTrajectoryVisual::getVelocity(float k){
	float t = k;
	Eigen::Vector3f taskVelocity;
	std::map<float, Eigen::Vector3f>::iterator it;	

	if(!taskVel.empty()){
		it = taskVel.end();
		it--;
		float last_t = it->first;
		if(t >= last_t){
			taskVelocity.setZero();
			return taskVelocity;
		}
	}
	it = taskVel.begin();
	float tc = it->first;
	while(tc < t){
		it++;
		tc = it->first;
	}	
	taskVelocity = it->second;
	return taskVelocity;
}

void TaskTrajectoryVisual::defineTask(Eigen::Vector3f _init){

	Eigen::Vector3f pos;
	Eigen::Vector3f vel;
	float t;	
 
	if(atts.size() == 1){
		Eigen::Vector3f init = _init;
		Eigen::Vector3f fin = atts.at(atts.size()-1);	
		float t_init = 0.0;
		float t_fin = t_atts.at(t_atts.size()-1);
		dur = t_fin;
		t = t_init;
		Eigen::Vector3f p0, p1, p2, p3;
		p0 = init;
		p1 = init;
		p2 = fin;
		p3 = fin;
	
		simFloat pSphere[3];
		simInt sp1 = simGetObjectHandle("SphereP1");
		simGetObjectPosition(sp1, -1, pSphere);
		p1 << pSphere[0], pSphere[1], pSphere[2];
		simInt sp2 = simGetObjectHandle("SphereP2");
		simGetObjectPosition(sp2, -1, pSphere);
		p2 << pSphere[0], pSphere[1], pSphere[2];

		float s;
		while(t < t_fin){
			s = t / dur;
			pos = p0*std::pow(1.0-s,3) + 3*p1*s*std::pow(1.0-s,2) + 3*p2*std::pow(s,2)*(1.0-s) + p3*std::pow(s,3); 
			taskPos.insert(std::pair<float, Eigen::Vector3f>(t, pos));
			vel =
			(3*p3*std::pow(t,2))/std::pow(dur,3)
			- (3*p2*std::pow(t,2))/std::pow(dur,3)
			- (3*p0*std::pow((t/dur - 1),2))/dur 
			+ (3*p1*std::pow((t/dur - 1),2))/dur 
			+ (6*p1*t*(t/dur - 1))/std::pow(dur,2) 
			- (6*p2*t*(t/dur - 1))/std::pow(dur,2);
			taskVel.insert(std::pair<float, Eigen::Vector3f>(t, vel));			

			t = t + 0.01;	 
		}
		
	}

	std::cout << "The visual task has been defined" << std::endl;
	
}

void TaskTrajectoryVisual::addAttractorTime(float _t_att){	
	t_atts.push_back(_t_att); 
}

float TaskTrajectoryVisual::getAttractorTime(float i){
	float t_att = 0.0;
	for(int k = 0; k <= i; k++){
		t_att = t_att + t_atts.at(k); 	
	}
	return t_att;
}

float TaskTrajectoryVisual::getDuration(){
	return dur;
} 

float TaskTrajectoryVisual::computeClosestPoint(Eigen::VectorXf vr, float si){return -1;};

