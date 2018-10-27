/*
 * File:   TaskSwFoot.cpp
 * Author: Paolo Ferrari
 *
 * Created on March 1, 2016
 */

#include "TaskSwFoot.hpp"
#include <fstream>
#include <sstream>
#include <boost/lexical_cast.hpp>

TaskSwFoot::~TaskSwFoot() {
}

TaskSwFoot::TaskSwFoot(int _type, float _duration, Configuration& qCurr, float _tk, float _height, Robot* robot, Kinematics* kin){
	
	type = _type;
	duration = _duration;
	tk = _tk;
	height = _height;

	EndEffector swingFoot;
	EndEffector supportFoot;
	std::map<float, Eigen::Vector2f> td; 
    std::map<float, Eigen::Vector2f> td_dot;
	float theta = 0.0;
	
	td = loadFromFile("HRP4Gait/Pfoot.txt");
	td_dot = loadFromFile("HRP4Gait/Vfoot.txt");
	
	supportFoot = L_FOOT;	//
	
	std::cout << "SWG tk:: " << tk << std::endl;
	std::cout << "SWG duration:: " << duration << std::endl;

	//////////////////////////////////////////////////////////////////////////////////////////////
	if(supportFoot == R_FOOT) swingFoot = L_FOOT;
	else swingFoot = R_FOOT;		
		
	Eigen::MatrixXf Tsup_w = kin->forwardKinematics(qCurr, supportFoot);
	Eigen::MatrixXf Rsup_w = Tsup_w.block(0,0,3,3);
	Eigen::MatrixXf Tswg_w = kin->forwardKinematics(qCurr, swingFoot);
	Eigen::MatrixXf Rswg_w = Tswg_w.block(0,0,3,3);
	Eigen::VectorXf vswg_w = t2v(Tswg_w); 
	Eigen::MatrixXf Rz = rotz(theta);
	Eigen::MatrixXf Rswg_w_fin = Rz * Rswg_w;
	float b_f = atan2( Rswg_w_fin(0,2), sqrt(pow(Rswg_w_fin(0,0), 2) + pow(Rswg_w_fin(0,1), 2)) );
	float a_f = atan2( -Rswg_w_fin(1,2)/cos(b_f), Rswg_w_fin(2,2)/cos(b_f) );
	float g_f = atan2( -Rswg_w_fin(0,1)/cos(b_f), Rswg_w_fin(0,0)/cos(b_f) );
	float a_i = vswg_w(3);
	float b_i = vswg_w(4);
	float g_i = vswg_w(5);

	//std::cout << "Rswg_w:: " << Rswg_w << std::endl;
	//std::cout << "Rswg_w_fin:: " << Rswg_w_fin << std::endl;
	//std::cout << "init orient:: " << vswg_w(3) << " " << vswg_w(4) << " " << vswg_w(5) << std::endl;
	//std::cout << "fin orient:: " << a_f << " " << b_f << " " << g_f << std::endl;

	std::map<float, Eigen::Vector2f>::iterator it;
	float ti = tk;
	float tf = tk + duration;

	std::cout << "SWG:: " << ti << " " << tf << std::endl;

	// position
	if(!td.empty()){
		it = td.begin();
		while(it != td.end()){
			float t = it->first + tk;
			Eigen::Vector2f p2 = it->second;
			Eigen::VectorXf p_sup(4);
			p_sup << p2(0), p2(1), 0.0, 1.0;
			 
			Eigen::VectorXf p_w(4);
			p_w = Tsup_w * p_sup;	
			p_w(2) = (-4.0 * height) * (std::pow(((t - ti)/(tf - ti)), 2) - (((t - ti)/(tf - ti)))) + vswg_w(2);	

			Eigen::VectorXf o_w(3);
			float s = (t - ti)/(tf - ti);
			o_w(0) = s * a_f + (1.0 - s) * a_i;
			o_w(1) = s * b_f + (1.0 - s) * b_i;
			o_w(2) = s * g_f + (1.0 - s) * g_i; 
						

			Eigen::VectorXf p(6);			
			p << p_w(0), p_w(1), p_w(2), o_w(0), o_w(1), o_w(2);
			taskPos.insert(std::pair<float, Eigen::VectorXf>(t, p));			

			it++;
		}

		it = td.end();
		it--;
		Eigen::Vector2f p2 = it->second;
		Eigen::VectorXf p_sup(4);
		p_sup << p2(0), p2(1), 0.0, 1.0;
			 
		Eigen::VectorXf p_w(4);
		p_w = Tsup_w * p_sup;	
		p_w(2) = vswg_w(2);	

		Eigen::VectorXf p(6);
		p << p_w(0), p_w(1), p_w(2), a_f, b_f, g_f;
		taskPos.insert(std::pair<float, Eigen::VectorXf>(tf, p));	
	}
	else{
		Eigen::VectorXf p(6);
		p << vswg_w(0), vswg_w(1), vswg_w(2), vswg_w(3), vswg_w(4), vswg_w(5);
		taskPos.insert(std::pair<float, Eigen::VectorXf>(tf, p));	
	}
	
	//velocity
	if(!td_dot.empty()){
		it = td_dot.begin();
		while(it != td_dot.end()){
			float t = it->first + tk;
			Eigen::Vector2f v2 = it->second;
			Eigen::VectorXf v_sup(3);
			v_sup << v2(0), v2(1), 0.0;
			 
			Eigen::VectorXf v_w(3);
			v_w = Rsup_w * v_sup;	
			v_w(2) = (-4.0 * height / (tf - ti)) * ((2.0 * ((t - ti)/(tf - ti))) - 1.0);

			Eigen::VectorXf odot_w(3);
			odot_w(0) = (a_f - a_i) / (tf - ti);
			odot_w(1) = (b_f - b_i) / (tf - ti);
			odot_w(2) = (g_f - g_i) / (tf - ti); 
	
			Eigen::VectorXf v(6);
			v << v_w(0), v_w(1), v_w(2), odot_w(0), odot_w(1), odot_w(2);
			taskVel.insert(std::pair<float, Eigen::VectorXf>(t, v));

			it++;
		}

		it = td_dot.end();
		it--;
		Eigen::Vector2f v2 = it->second;
		Eigen::VectorXf v_sup(3);
		v_sup << v2(0), v2(1), 0.0;
			 
		Eigen::VectorXf v_w(3);
		v_w = Rsup_w * v_sup;	
		v_w(2) = 0.0;	
		Eigen::VectorXf v(6);
		//v << v_w(0), v_w(1), v_w(2), 0.0, 0.0, 0.0;
		v << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
		taskVel.insert(std::pair<float, Eigen::VectorXf>(tf, v));			
	}	
	else{
		Eigen::VectorXf v(6);
		v << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
		taskVel.insert(std::pair<float, Eigen::VectorXf>(tf, v));		
	}
	
}

Eigen::VectorXf TaskSwFoot::getPosition(float t){
	Eigen::VectorXf taskPosition(6);
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

Eigen::VectorXf TaskSwFoot::getVelocity(float t){
	Eigen::VectorXf taskVelocity(6);
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

std::map<float, Eigen::Vector2f> TaskSwFoot::loadFromFile(std::string path_to_file){

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


