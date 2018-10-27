/*
 * File:   TaskCoM.hpp
 * Author: Paolo Ferrari
 *
 * Created on March 2, 2016
 */

#ifndef TASKCOM_HPP
#define	TASKCOM_HPP

#include "constant_values.hpp"
#include "Eigen/Core"
#include "Eigen/Dense"
#include "Configuration.hpp"
#include "Robot.hpp"
//#include "Nao.hpp"
#include "Hrp4.hpp"
#include "Kinematics.hpp"
//#include "NaoKinematics.hpp"
#include "Hrp4Kinematics.hpp"
#include "matrix_tools.hpp"
#include <vector>
#include <map>
#include <fstream>

class TaskCoM{
	
	public:
	
		TaskCoM(int _type, float _duration, Configuration& qCurr, float _tk, Robot* robot, Kinematics* kin);
		virtual ~TaskCoM();
   		Eigen::VectorXf getPosition(float t);
    	Eigen::VectorXf getVelocity(float t);

		Eigen::VectorXf getConstrainedComponents();
		Eigen::VectorXf getGains();

	private:
		
		int type;
		float duration;
		float tk;
		Eigen::VectorXf constComp;
		Eigen::VectorXf K_CoM;
		
		std::map<float, Eigen::VectorXf> taskPos;
		std::map<float, Eigen::VectorXf> taskVel;
					
		std::map<float, Eigen::Vector2f> loadFromFile(std::string path_to_file);
   
};

#endif	/* TASKCOM_HPP */
