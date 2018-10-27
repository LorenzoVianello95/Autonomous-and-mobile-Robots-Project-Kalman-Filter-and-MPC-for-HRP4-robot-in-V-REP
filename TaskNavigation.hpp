/*
 * File:   TaskNavigation.hpp
 * Author: Paolo Ferrari
 *
 * Created on January 31, 2017
 */

#ifndef TASKNAVIGATION_H
#define	TASKNAVIGATION_H

#include "Task.hpp"
#include "Eigen/Core"
#include "Eigen/Dense"
#include "enum.h"

class TaskNavigation : public Task {
	
	public:
		TaskNavigation();
		~TaskNavigation();

		Eigen::Vector3f getPosition(float k);
		Eigen::Vector3f getVelocity(float k);
		
		void defineTask(Eigen::Vector3f _init);
		void addAttractorTime(float _t_att);
		float getAttractorTime(float i);
		float getDuration();
		float computeClosestPoint(Eigen::VectorXf vr, float si);
		
};

#endif	/* TASKNAVIGATION_H */

