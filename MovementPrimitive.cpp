/* 
 * File:   MovementPrimitive.hpp
 * Author: Paolo Ferrari
 *
 * Created on June 27, 2017 
 */

#include "MovementPrimitive.hpp"
#include <stdlib.h>	

MovementPrimitive::MovementPrimitive(int _type, Configuration& qCurr, float _tk, Robot* _robot, Kinematics* _kin){

	zswg = NULL;
	zCoM = NULL;

						//	type		height	dur 	taskSwg 	taskCoM
	PrimitivesTable <<	FREE_COM,		0,		0.5,		1, 			0,
						GAIT,		0.05,		0.6,		1, 			1;
						
	
	type = _type;
	tk = _tk;
	robot = _robot; 
	kin = _kin;

	float height = PrimitivesTable(type, 1);
	duration = PrimitivesTable(type, 2);

	if(PrimitivesTable(type, 3) == 1) zswg = new TaskSwFoot(type, duration, qCurr, tk, height, robot, kin);
	if(PrimitivesTable(type, 4) == 1) zCoM = new TaskCoM(type, duration, qCurr, tk, robot, kin);
	
	//std::cout << "Movement Primitive... " << type << std::endl;
	
}

MovementPrimitive::~MovementPrimitive(){ }

float MovementPrimitive::getDuration(){
	return duration;
}

void MovementPrimitive::setDuration(float dur){
	duration = dur;
}

TaskSwFoot* MovementPrimitive::getzswg(){
	return zswg;
}

TaskCoM* MovementPrimitive::getzCoM(){
	return zCoM;
}


