/*
 * File:   Hrp4.hpp
 * Author: Paolo Ferrari
 *
 * Created on July 20, 2017
 */

#ifndef HRP4_H
#define	HRP4_H

#include "Robot.hpp"
#include "Joint.hpp"
#include "Configuration.hpp"
#include "Eigen/Core"
#include "Eigen/Dense"
#include "Kinematics.hpp"
//#include "Hrp4Kinematics.hpp"
#include "enum.h"
#include "matrix_tools.hpp"

class Hrp4 : public Robot {
	
	public:
		Hrp4();
		~Hrp4();
		
		void setConfigurationStat(Configuration& q);
		void setConfigurationDyn(Configuration& q);
		void setJointsConfiguration(Configuration& q);
		Configuration getCurrentConfiguration();
		std::vector<Joint> getJoints();  
		std::vector<std::string> getJointNames();
		
};

#endif	/* HRP4_H */

