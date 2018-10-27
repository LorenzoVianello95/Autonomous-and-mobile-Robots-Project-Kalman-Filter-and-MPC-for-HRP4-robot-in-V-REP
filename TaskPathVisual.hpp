/*
 * File:   TaskPathVisual.hpp
 * Author: Paolo Ferrari
 *
 * Created on March 13, 2017
 */

#ifndef TASKPATHVISUAL_H
#define	TASKPATHVISUAL_H

#include "Task.hpp"
#include "Eigen/Core"
#include "Eigen/Dense"
#include "enum.h"

class TaskPathVisual : public Task {
	
	public:
		TaskPathVisual();
		~TaskPathVisual();
		
		Eigen::Vector3f getPosition(float k);
		Eigen::Vector3f getVelocity(float k);
		
		void defineTask(Eigen::Vector3f _init);
		void addAttractorTime(float _t_att);
		float getAttractorTime(float i);
		float getDuration();
		float computeClosestPoint(Eigen::VectorXf vr, float si);		
};

#endif	/* TASKPATHVISUAL_H */

