/*
 * File:   TaskCoM.cpp
 * Author: Paolo Ferrari
 *
 * Created on March 1, 2016
 */

#include "TaskCoM.hpp"
#include "constant_values.hpp"
#include <fstream>
#include <sstream>
#include <boost/lexical_cast.hpp>

  
TaskCoM::TaskCoM(int _type, float _duration, Configuration& qCurr, float _tk, Robot* robot, Kinematics* kin){
	
	type = _type;
	duration = _duration;
	tk = _tk;
	
	constComp.resize(6);
	constComp << 1, 1, 1, 1, 1, 0;
	K_CoM.resize(5);
	K_CoM << 2.0, 2.0, 2.0, 50.0, 50.0;
	    
	EndEffector swingFoot;
	EndEffector supportFoot;
	std::map<float, Eigen::Vector2f> td; 
    std::map<float, Eigen::Vector2f> td_dot;
	float theta = 0.0;
	
	td = loadFromFile("HRP4Gait/Pcom.txt");
	td_dot = loadFromFile("HRP4Gait/Vcom.txt");
	
	supportFoot = L_FOOT;	//
	
	std::cout << "COM tk:: " << tk << std::endl;
	std::cout << "COM duration:: " << duration << std::endl;

	//////////////////////////////////////////////////////////////////////////////////////////////
	if(supportFoot == R_FOOT) swingFoot = L_FOOT;
	else swingFoot = R_FOOT;		
		
	Eigen::MatrixXf Tsup_w = kin->forwardKinematics(qCurr, supportFoot);
	Eigen::MatrixXf Rsup_w = Tsup_w.block(0,0,3,3);
	Eigen::VectorXf vsup_w = t2v(Tsup_w);
	Eigen::MatrixXf Tswg_w = kin->forwardKinematics(qCurr, swingFoot);
	Eigen::MatrixXf Rswg_w = Tswg_w.block(0,0,3,3);
	Eigen::VectorXf vswg_w = t2v(Tswg_w);
	Eigen::MatrixXf Rz = rotz(theta);
	Eigen::MatrixXf Rswg_w_fin = Rz * Rswg_w;
	float bswg_f = atan2( Rswg_w_fin(0,2), sqrt(pow(Rswg_w_fin(0,0), 2) + pow(Rswg_w_fin(0,1), 2)) );
	float aswg_f = atan2( -Rswg_w_fin(1,2)/cos(bswg_f), Rswg_w_fin(2,2)/cos(bswg_f) );
	float gswg_f = atan2( -Rswg_w_fin(0,1)/cos(bswg_f), Rswg_w_fin(0,0)/cos(bswg_f) );
	float asup_f = vsup_w(3);
	float bsup_f = vsup_w(4);
	float gsup_f = vsup_w(5);
	
	Eigen::MatrixXf Tsup_CoM = kin->forwardKinematicsWrtCoM(qCurr, supportFoot);
	Eigen::MatrixXf TCoM_sup = Tsup_CoM.inverse(); 
	Eigen::VectorXf vCoM_sup = t2v(TCoM_sup);
	Eigen::Vector3f oCoMInit = qCurr.getqCoMOrientation();
	float aCoM_i = oCoMInit(0);
	float bCoM_i = oCoMInit(1);
	float gCoM_i = oCoMInit(2);
	
	Eigen::MatrixXf RCoM_w = rot(oCoMInit);
	Eigen::MatrixXf RzCoM = rotz(theta/2.0);
	Eigen::MatrixXf RCoM_w_fin = RzCoM * RCoM_w;
	float bCoM_f = atan2( RCoM_w_fin(0,2), sqrt(pow(RCoM_w_fin(0,0), 2) + pow(RCoM_w_fin(0,1), 2)) );
	float aCoM_f = atan2( -RCoM_w_fin(1,2)/cos(bCoM_f), RCoM_w_fin(2,2)/cos(bCoM_f) );
	float gCoM_f = atan2( -RCoM_w_fin(0,1)/cos(bCoM_f), RCoM_w_fin(0,0)/cos(bCoM_f) );

	std::map<float, Eigen::Vector2f>::iterator it;
	float ti = tk;
	float tf = tk + duration;
	
	std::cout << "COM:: " << ti << " " << tf << std::endl;

	// position
	if(!td.empty()){
		it = td.begin();
		while(it != td.end()){
			//float t = it->first + tk;
			float t = it->first;
			
			Eigen::Vector2f p2 = it->second;
			Eigen::VectorXf pCoM_sup(4);
			pCoM_sup << p2(0), p2(1), vCoM_sup(2), 1.0;
			 
			Eigen::VectorXf p_w(4);
			p_w = Tsup_w * pCoM_sup;

			Eigen::VectorXf o_w(3);
			float s = (t - ti)/(tf - ti);
			o_w(0) = s * aCoM_f + (1.0 - s) * aCoM_i;
			o_w(1) = s * bCoM_f + (1.0 - s) * bCoM_i;
			o_w(2) = s * gCoM_f + (1.0 - s) * gCoM_i; 
			
			Eigen::VectorXf p(5);
			//p << p_w(0), p_w(1), p_w(2), oCoMInit(0), oCoMInit(1);
			p << p_w(0), p_w(1), p_w(2), o_w(0), o_w(1);
			taskPos.insert(std::pair<float, Eigen::VectorXf>(t, p));			
		
			if(t >= ti && t < tf) std::cout << "t:: " << t << std::endl; 
			
			it++;
		}

		/*
		it = td.end();
		it--;

		Eigen::Vector2f p2 = it->second;
		Eigen::VectorXf pCoM_sup(4);
		pCoM_sup << p2(0), p2(1), vCoM_sup(2), 1.0;
			 
		Eigen::VectorXf p_w(4);
		p_w = Tsup_w * pCoM_sup;
		
		Eigen::VectorXf p(5);
		//p << p_w(0), p_w(1), p_w(2), oCoMInit(0), oCoMInit(1);
		p << p_w(0), p_w(1), p_w(2), aCoM_f, bCoM_f;
		taskPos.insert(std::pair<float, Eigen::VectorXf>(tf, p));			
		
		if(tf >= ti && tf <= tf) std::cout << "t:: " << tf << std::endl; 
		*/
	}

	
	//velocity	
	if(!td_dot.empty()){
		it = td_dot.begin();
		while(it != td_dot.end()){
			float t = it->first + tk;
			Eigen::Vector2f v2 = it->second;
			Eigen::VectorXf pCoMdot_sup(3);
			pCoMdot_sup << v2(0), v2(1), 0.0;
			 
			Eigen::VectorXf pdot_w(3);
			pdot_w = Rsup_w * pCoMdot_sup;

			Eigen::VectorXf odot_w(3);
			odot_w(0) = (aCoM_f - aCoM_i) / (tf - ti);
			odot_w(1) = (bCoM_f - bCoM_i) / (tf - ti);
			odot_w(2) = (gCoM_f - gCoM_i) / (tf - ti); 
		
			Eigen::VectorXf v(5);
			v << pdot_w(0), pdot_w(1), pdot_w(2), odot_w(0), odot_w(1);
			taskVel.insert(std::pair<float, Eigen::VectorXf>(t, v));			
			
			it++;
		}

		it = td_dot.end();
		it--;

		Eigen::Vector2f v2 = it->second;
		Eigen::VectorXf pCoMdot_sup(3);
		pCoMdot_sup << v2(0), v2(1), 0.0;
			 
		Eigen::VectorXf pdot_w(3);
		pdot_w = Rsup_w * pCoMdot_sup;
		
		Eigen::VectorXf v(5);
		v << 0.0, 0.0, 0.0, 0.0, 0.0;
		taskVel.insert(std::pair<float, Eigen::VectorXf>(tf, v));			
		
	}
	 
}

TaskCoM::~TaskCoM(){}

Eigen::VectorXf TaskCoM::getPosition(float t){
	Eigen::VectorXf taskPosition(4);
	std::map<float, Eigen::VectorXf>::iterator it;

	it = taskPos.begin();
	float tc = it->first;
	while(tc < t){
		if(it == taskPos.end()){
			it--;		
			break;
		}
		it++;
		tc = it->first;		
	}	
	taskPosition = it->second;
	return taskPosition;
}

Eigen::VectorXf TaskCoM::getVelocity(float t){
	Eigen::VectorXf taskVelocity(4);
	std::map<float, Eigen::VectorXf>::iterator it;	

	it = taskVel.begin();
	float tc = it->first;
	while(tc < t){
		if(it == taskVel.end()){
			it--;		
			break;
		}
		it++;
		tc = it->first;
	}	
	taskVelocity = it->second;
	return taskVelocity;
}

std::map<float, Eigen::Vector2f> TaskCoM::loadFromFile(std::string path_to_file){

	std::map<float, Eigen::Vector2f> ret;
	Eigen::Vector2f val;
	float t;
	
	std::string line;		
	std::ifstream td_file(path_to_file.c_str());
    while(getline(td_file, line)){
		std::stringstream linestream(line);
		std::string value;
		Eigen::VectorXf _r(3);
		int index = 0;
		while(getline(linestream,value,',')){
			std::string::iterator end_pos = std::remove(value.begin(), value.end(), ' ');
			value.erase(end_pos, value.end());
		    float temp = boost::lexical_cast<float>(value);
		    _r(index) = temp;
		    index++;
		}
		t = _r(0);
		val(0) = _r(1);
		val(1) = _r(2);
		ret.insert(std::pair<float, Eigen::Vector2f>(t, val));
	}

	return ret;
}


Eigen::VectorXf TaskCoM::getConstrainedComponents(){
	return constComp;
}

Eigen::VectorXf TaskCoM::getGains(){
	return K_CoM;
}




