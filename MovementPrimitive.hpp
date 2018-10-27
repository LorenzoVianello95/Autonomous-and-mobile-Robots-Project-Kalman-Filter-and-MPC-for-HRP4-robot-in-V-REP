/* 
 * File:   MovementPrimitive.hpp
 * Author: Paolo Ferrari
 *
 * Created on June 27, 2017 
 */

#ifndef MOVEMENTPRIMITIVE_H
#define MOVEMENTPRIMITIVE_H

#include <stdio.h>
#include <iostream>
#include "enum.h"
#include "Eigen/Dense"
#include "Configuration.hpp"
#include "Robot.hpp"
//#include "Nao.hpp"
#include "Hrp4.hpp"
#include "Kinematics.hpp"
//#include "NaoKinematics.hpp"
#include "Hrp4Kinematics.hpp"
#include "TaskSwFoot.hpp"
#include "TaskCoM.hpp"

class MovementPrimitive {

	private:
		Eigen::Matrix<float, PRIMITIVES_SET_SIZE, 5> PrimitivesTable;				

		int type;
		float tk;		
		float duration;
		Robot* robot;
		Kinematics* kin;
		TaskSwFoot* zswg;
		TaskCoM* zCoM;
	    
	public:
		MovementPrimitive(int _type, Configuration& qCurr, float _tk, Robot* _robot, Kinematics* _kin);
		~MovementPrimitive();

		float getDuration();
		void setDuration(float dur);
		TaskSwFoot* getzswg();
		TaskCoM* getzCoM();
    	
};

#endif
