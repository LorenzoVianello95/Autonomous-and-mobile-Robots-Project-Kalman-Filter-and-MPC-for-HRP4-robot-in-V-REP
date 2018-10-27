/*
 * File:   TaskPathVisual.cpp
 * Author: Paolo Ferrari
 *
 * Created on March 13, 2017
 */


#include "TaskPathVisual.hpp"
#include <boost/lexical_cast.hpp>

TaskPathVisual::TaskPathVisual(){
	type = TASK_PATH_VISUAL;	
	lastReached = -1;
}

TaskPathVisual::~TaskPathVisual(){} 

Eigen::Vector3f TaskPathVisual::getPosition(float k){
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

Eigen::Vector3f TaskPathVisual::getVelocity(float k){
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

void TaskPathVisual::defineTask(Eigen::Vector3f _init){

	Eigen::Vector3f pos;
	Eigen::Vector3f vel;
	float s;	
 
	if(atts.size() == 1){
		Eigen::Vector3f init = _init;
		Eigen::Vector3f fin = atts.at(atts.size()-1);	
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

		s = 0.0;
		while(s <= 1.0){
			pos = p0*std::pow(1.0-s,3) + 3*p1*s*std::pow(1.0-s,2) + 3*p2*std::pow(s,2)*(1.0-s) + p3*std::pow(s,3); 
			taskPos.insert(std::pair<float, Eigen::Vector3f>(s, pos));
			vel = 3*std::pow(1.0-s,2)*(p1-p0) + 6*(1.0-s)*s*(p2-p1) + 3*std::pow(s,2)*(p3-p2);
			taskVel.insert(std::pair<float, Eigen::Vector3f>(s, vel));

			s = s + 0.001;	 
		}
		
	}

	std::cout << "The visual task has been defined" << std::endl;
	
}

void TaskPathVisual::addAttractorTime(float _t_att){}
float TaskPathVisual::getAttractorTime(float i){return -1;}
float TaskPathVisual::getDuration(){return -1;}

float TaskPathVisual::computeClosestPoint(Eigen::VectorXf vr, float si){

	float s_star = 0.0;
	float s_inc = 0.01;
	float s = si;
		
	float sf = 1.0;

	Eigen::Vector3f p = this->getPosition(s);

	Eigen::MatrixXf Tc_w = v2t(vr);
	Eigen::VectorXf a_c_hom(4), b_c_hom(4);
	a_c_hom << 0.0, 0.0, 0.2, 1.0;
	b_c_hom << 0.0, 0.0, 0.5, 1.0;
	Eigen::VectorXf a_w_hom = Tc_w * a_c_hom;
	Eigen::VectorXf b_w_hom = Tc_w * b_c_hom;
	Eigen::Vector3f a, b;
	a << a_w_hom(0), a_w_hom(1), a_w_hom(2);
	b << b_w_hom(0), b_w_hom(1), b_w_hom(2);
	Eigen::Vector3f v = b - a;
	float d = -(v(0)*p(0) + v(1)*p(1) + v(2)*p(2));	
	float t = -(v(0)*a(0) + v(1)*a(1) + v(2)*a(2) + d) / (std::pow(v(0),2) + std::pow(v(1),2) + std::pow(v(2),2));
	Eigen::Vector3f pl = a + t*v;  // nearest point on the principal axis
	
	float dist_min = std::sqrt(std::pow(p(0)-pl(0), 2) + std::pow(p(1)-pl(1), 2) + std::pow(p(2)-pl(2), 2));	
	
	s_star = s;

	s = s + s_inc;
	while(s <= sf){
		p = this->getPosition(s);
		
		Eigen::VectorXf a_c_hom(4), b_c_hom(4);
		a_c_hom << 0.0, 0.0, 0.2, 1.0;
		b_c_hom << 0.0, 0.0, 0.5, 1.0;
		Eigen::VectorXf a_w_hom = Tc_w * a_c_hom;
		Eigen::VectorXf b_w_hom = Tc_w * b_c_hom;
		Eigen::Vector3f a, b;
		a << a_w_hom(0), a_w_hom(1), a_w_hom(2);
		b << b_w_hom(0), b_w_hom(1), b_w_hom(2);
		Eigen::Vector3f v = b - a;
		float d = -(v(0)*p(0) + v(1)*p(1) + v(2)*p(2));	
		float t = -(v(0)*a(0) + v(1)*a(1) + v(2)*a(2) + d) / (std::pow(v(0),2) + std::pow(v(1),2) + std::pow(v(2),2));
		Eigen::Vector3f pl = a + t*v;  // nearest point on the principal axis
	
		float dist = std::sqrt(std::pow(p(0)-pl(0), 2) + std::pow(p(1)-pl(1), 2) + std::pow(p(2)-pl(2), 2));
		if(dist < dist_min){
			dist_min = dist;
			s_star = s;
		}
		s = s + s_inc;		
	}
	return s_star;	
}

