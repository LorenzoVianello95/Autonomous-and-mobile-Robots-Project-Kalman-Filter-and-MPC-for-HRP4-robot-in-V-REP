/*
 * File:   Task.cpp
 * Author: Paolo Ferrari
 *
 * Created on January 31, 2017
 */

#include "Task.hpp"
#include "constant_values.hpp"
#include <fstream>
#include <sstream>
#include <boost/lexical_cast.hpp>

Task::Task(){  
}

TaskType Task::getTaskType(){
	return type;
}

void Task::addAttractor(Eigen::Vector3f _att){	
	atts.push_back(_att); 
}

int Task::getLastReachedAttractor(){
	return lastReached;
}

Eigen::Vector3f Task::getLastAttractor(){
	int last_index = atts.size() - 1;
	Eigen::Vector3f last_att = atts.at(last_index);	
	return last_att;
}

void Task::setLastReachedAttractor(){
	lastReached++;
}

Eigen::Vector3f Task::getAttractor(float i){
	Eigen::Vector3f att = atts.at(i);
	return att;	
} 


