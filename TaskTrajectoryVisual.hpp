/*
 * File:   TaskTrajectoryVisual.hpp
 * Author: Paolo Ferrari
 *
 * Created on February 14, 2017
 */

#ifndef TASKTRAJECTORYVISUAL_H
#define	TASKTRAJECTORYVISUAL_H

#include "Task.hpp"
#include "Eigen/Core"
#include "Eigen/Dense"
#include "enum.h"

class TaskTrajectoryVisual : public Task {
	
	public:
		TaskTrajectoryVisual();
		~TaskTrajectoryVisual();
		
		Eigen::Vector3f getPosition(float k);
		Eigen::Vector3f getVelocity(float k);
		
		void defineTask(Eigen::Vector3f _init);
		void addAttractorTime(float _t_att);
		float getAttractorTime(float i);
		float getDuration();
		float computeClosestPoint(Eigen::VectorXf vr, float si);
		
};

#endif	/* TASKTRAJECTORYVISUAL_H */

