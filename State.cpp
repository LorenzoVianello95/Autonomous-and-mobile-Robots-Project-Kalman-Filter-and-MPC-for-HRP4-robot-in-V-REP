/* 
 * File:   State.hpp
 * Author: Paolo Ferrari
 *
 * Created on March 4, 2016 
 */

#include "State.hpp"
#include <stdlib.h>

State::State(){}

State::State(Configuration& _q, MovementPrimitiveType _type, float _t){ 
	int dof = _q.getqjnt().size();	
	Eigen::Vector3f pCoM = _q.getqCoMPosition();
	Eigen::Vector3f oCoM = _q.getqCoMOrientation();
	q.setqCoMPosition(pCoM);
	q.setqCoMOrientation(oCoM);
	q.setqjnt(_q.getqjnt()); 
	
	type = _type;

	t = _t;
}
	
State::~State(){}

Configuration State::getConfiguration(){
	return q;
}

MovementPrimitiveType State::getMovementPrimitiveType(){
	return type;
}

float State::getTime(){
	return t;
}






