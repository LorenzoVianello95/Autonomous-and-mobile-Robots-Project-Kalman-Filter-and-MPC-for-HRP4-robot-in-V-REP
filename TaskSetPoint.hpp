/*
 * File:   TaskSetPoint.hpp
 * Author: Paolo Ferrari
 *
 * Created on January 31, 2017
 */

#ifndef TASKSETPOINT_H
#define	TASKSETPOINT_H

#include "Task.hpp"
#include "Eigen/Core"
#include "Eigen/Dense"
#include "enum.h"

class TaskSetPoint : public Task {
	
	public:
		TaskSetPoint();
		~TaskSetPoint();

		Eigen::Vector3f getPosition(float k);
		Eigen::Vector3f getVelocity(float k);
		
		void defineTask(Eigen::Vector3f _init);
		void addAttractorTime(float _t_att);
		float getAttractorTime(float i);
		float getDuration();
		float computeClosestPoint(Eigen::VectorXf vr, float si);
		
};

#endif	/* TASKSETPOINT_H */

