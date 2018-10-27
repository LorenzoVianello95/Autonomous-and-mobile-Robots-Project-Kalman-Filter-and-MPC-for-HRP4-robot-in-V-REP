/*
 * File:   TaskTrajectory.cpp
 * Author: Paolo Ferrari
 *
 * Created on January 31, 2017
 */


#include "TaskTrajectory.hpp"
#include <boost/lexical_cast.hpp>

TaskTrajectory::TaskTrajectory(){
	type = TASK_TRAJECTORY;	
	lastReached = -1;
}

TaskTrajectory::~TaskTrajectory(){} 

Eigen::Vector3f TaskTrajectory::getPosition(float k){
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

Eigen::Vector3f TaskTrajectory::getVelocity(float k){
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

void TaskTrajectory::defineTask(Eigen::Vector3f _init){
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
		while(t < dur){
			pos = init + (t / dur) * (fin - init); 
			taskPos.insert(std::pair<float, Eigen::Vector3f>(t, pos));
			vel = (fin - init) / dur;
			taskVel.insert(std::pair<float, Eigen::Vector3f>(t, vel));
			t = t + 0.01;	 
		}
	}

	if(atts.size() == 3){		
		std::string line;		
		std::ifstream pos_file("trajTaskPos.txt");
		while (getline(pos_file,line)){
		    std::stringstream linestream(line);
		    std::string value;
		    Eigen::VectorXf _pd(4);
		    int index = 0;
		    while(getline(linestream,value,',')){
				std::string::iterator end_pos = std::remove(value.begin(), value.end(), ' ');
				value.erase(end_pos, value.end());
		    	float temp = boost::lexical_cast<float>(value);
		        _pd(index) = temp;
		        index++;
		    }
			t = _pd(0);
			pos(0) = _pd(1);
			pos(1) = _pd(2);
			pos(2) = _pd(3);
			taskPos.insert(std::pair<float, Eigen::Vector3f>(t, pos));
		}
		std::ifstream vel_file("trajTaskVel.txt");
		while (getline(vel_file,line)){
		    std::stringstream linestream(line);
		    std::string value;
		    Eigen::VectorXf _vd(4);
		    int index = 0;
		    while(getline(linestream,value,',')){
				std::string::iterator end_pos = std::remove(value.begin(), value.end(), ' ');
				value.erase(end_pos, value.end());
		    	float temp = boost::lexical_cast<float>(value);
		        _vd(index) = temp;
		        index++;
		    }
			t = _vd(0);
			vel(0) = _vd(1);
			vel(1) = _vd(2);
			vel(2) = _vd(3);
			taskVel.insert(std::pair<float, Eigen::Vector3f>(t, vel));
		}	
		dur = t_atts.at(0) + t_atts.at(1) + t_atts.at(2); 
	}

	std::cout << "The trajectory task has been defined" << std::endl;
}

void TaskTrajectory::addAttractorTime(float _t_att){	
	t_atts.push_back(_t_att); 
}

float TaskTrajectory::getAttractorTime(float i){
	float t_att = 0.0;
	for(int k = 0; k <= i; k++){
		t_att = t_att + t_atts.at(k); 	
	}
	return t_att;
}

float TaskTrajectory::getDuration(){
	return dur;
} 

float TaskTrajectory::computeClosestPoint(Eigen::VectorXf vr, float si){return -1;};

