/* 
 * File:   Robot.hpp
 * Author: Paolo Ferrari
 *
 * Created on February 15, 2016
 */

#ifndef ROBOT_H
#define	ROBOT_H

#include "Joint.hpp"
#include "Configuration.hpp"
#include "Eigen/Core"
#include "Eigen/Dense"
#include <vector>
#include "Kinematics.hpp"
//#include "NaoKinematics.hpp"
#include "enum.h"
#include "matrix_tools.hpp"	
#include <string.h>

class Robot {

	public:
		Robot();
		virtual ~Robot() = 0;
		int getNdof();

		virtual Configuration getCurrentConfiguration() = 0;
		virtual void setConfigurationStat(Configuration& q) = 0;
		virtual void setConfigurationDyn(Configuration& q) = 0;
		virtual void setJointsConfiguration(Configuration& q) = 0;
		virtual std::vector<Joint> getJoints() = 0; 
		virtual std::vector<std::string> getJointNames() = 0;

	protected:	
		std::vector<Joint> joints;
		//std::vector<int> links;
		std::vector<std::string> joint_names;

};

#endif	/* ROBOT_H */

