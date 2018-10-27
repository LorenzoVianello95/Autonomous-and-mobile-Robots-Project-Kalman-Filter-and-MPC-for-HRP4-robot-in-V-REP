/* 
 * File:   TaskSwFoot.hpp
 * Author: Poo
 *
 * Created on September 8, 2013, 8:04 PM
 */

#ifndef TASKSWFOOT_HPP
#define	TASKSWFOOT_HPP

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

class TaskSwFoot {
	
	public:
		TaskSwFoot(int _type, float _duration, Configuration& qCurr, float _tk, float _height, Robot* robot, Kinematics* kin);		
		virtual ~TaskSwFoot();

		Eigen::VectorXf getPosition(float t);
		Eigen::VectorXf getVelocity(float t);

		
	private:

		int type;
		float duration;
		float tk;
		float height;
		
		std::map<float, Eigen::VectorXf> taskPos;
		std::map<float, Eigen::VectorXf> taskVel;

		std::map<float, Eigen::Vector2f> loadFromFile(std::string path_to_file);
};

#endif	/* TASKSWFOOT_HPP */

