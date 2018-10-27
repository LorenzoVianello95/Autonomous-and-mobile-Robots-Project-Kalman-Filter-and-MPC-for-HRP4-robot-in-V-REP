/*
 * File:   Robot.cpp
 * Author: Paolo Ferrari
 *
 * Created on February 15, 2016
 */

#include "Robot.hpp"
#include "Joint.hpp"
#include "Configuration.hpp"
#include "v_repLib.h"

Robot::Robot(){  
}

int Robot::getNdof(){
	return joints.size();
}




