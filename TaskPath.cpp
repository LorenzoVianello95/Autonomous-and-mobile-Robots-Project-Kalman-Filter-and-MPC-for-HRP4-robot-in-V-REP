/*
 * File:   TaskPath.cpp
 * Author: Paolo Ferrari
 *
 * Created on January 31, 2017
 */


#include "TaskPath.hpp"
#include <boost/lexical_cast.hpp>

TaskPath::TaskPath(){
	type = TASK_PATH;	
	lastReached = -1;
}

TaskPath::~TaskPath(){}

Eigen::Vector3f TaskPath::getPosition(float k){
	float s = k;
	Eigen::Vector3f taskPosition;
	std::map<float, Eigen::Vector3f>::iterator it;

	if(!taskPos.empty()){
		it = taskPos.end();
		it--;
		float last_s = it->first;
		if(s >= last_s) return atts.at(atts.size() - 1);
	}
	it = taskPos.begin();
	float sc = it->first;
	while(sc < s){
		it++;
		sc = it->first;
	}	
	taskPosition = it->second;
	return taskPosition;
}

Eigen::Vector3f TaskPath::getVelocity(float k){
	float s = k;
	Eigen::Vector3f taskVelocity;
	std::map<float, Eigen::Vector3f>::iterator it;	

	if(!taskVel.empty()){
		it = taskVel.end();
		it--;
		float last_s = it->first;
		if(s >= last_s){
			taskVelocity.setZero();
			return taskVelocity;
		}
	}
	it = taskVel.begin();
	float sc = it->first;
	while(sc < s){
		it++;
		sc = it->first;
	}	
	taskVelocity = it->second;
	return taskVelocity;
}

void TaskPath::defineTask(Eigen::Vector3f _init){	
	Eigen::Vector3f pos;
	Eigen::Vector3f vel;
	float s;	

	if(atts.size() == 1){
		Eigen::Vector3f init = _init;
		Eigen::Vector3f fin = atts.at(atts.size()-1);	
		float s = 0.0;
		while(s <= 1.0){
			pos = init + s * (fin - init); 
			taskPos.insert(std::pair<float, Eigen::Vector3f>(s, pos));
			vel = fin - init;
			taskVel.insert(std::pair<float, Eigen::Vector3f>(s, vel));
			s = s + 0.001;	 
		}
	}

	if(atts.size() == 3){
		std::string line;		
		std::ifstream pos_file("pathTaskPos.txt");
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
			s = _pd(0);
			pos(0) = _pd(1);
			pos(1) = _pd(2);
			pos(2) = _pd(3);
			taskPos.insert(std::pair<float, Eigen::Vector3f>(s, pos));
		}
		std::ifstream vel_file("pathTaskVel.txt");
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
			s = _vd(0);
			vel(0) = _vd(1);
			vel(1) = _vd(2);
			vel(2) = _vd(3);
			taskVel.insert(std::pair<float, Eigen::Vector3f>(s, vel));
		}
	}	

	std::cout << "The task path has been defined" << std::endl;
} 

void TaskPath::addAttractorTime(float _t_att){}
float TaskPath::getAttractorTime(float i){return -1;}
float TaskPath::getDuration(){return -1;}

float TaskPath::computeClosestPoint(Eigen::VectorXf vr, float si){
	Eigen::Vector3f p;
	p(0) = vr(0);
	p(1) = vr(1);
	p(2) = vr(2);

	Eigen::Vector3f att;
	float dist;	
	for(int i = 0; i < atts.size(); i++){
		att = this->getAttractor(i);
		dist = std::sqrt( std::pow(p(0) - att(0), 2) + std::pow(p(1) - att(1), 2) + std::pow(p(2) - att(2), 2));
		if(dist < 0.01) return (i+1.0)/(float)atts.size();	
	}
		
	float s_star;
	float s_inc = 0.01;
	float s = si + 0.1;	
	float sf;
	if(si == -1){
		s = 0.01;
		sf = 1.0;
	}
	else{
		if(atts.size() == 1) sf = 1.0; // 1 attractor
		else{ // 3 attractors
			if(si <= 1.0/3.0) sf = 1.0/3.0;
			else if(si > 1.0/3.0 && si <= 2.0/3.0) sf = 2.0/3.0;
			else sf = 1.0; 	
		}
	}
	
	Eigen::Vector3f p_curr = this->getPosition(s);
	float dmin = std::sqrt( std::pow(p_curr(0) - p(0), 2) + std::pow(p_curr(1) - p(1), 2) + std::pow(p_curr(2) - p(2), 2) );	
	s_star = s;
	while(s <= sf){
		p_curr = this->getPosition(s);
		float d = std::sqrt( std::pow(p_curr(0) - p(0), 2) + std::pow(p_curr(1) - p(1), 2) + std::pow(p_curr(2) - p(2), 2) );
		if(d < dmin){
			dmin = d;
			s_star = s;
		}
		s = s + s_inc;		
	}
	return s_star;	
}

