/* 
 * File:   Task.hpp
 * Author: Paolo Ferrari
 *
 * Created on January 31, 2017
 */

#ifndef TASK_H
#define	TASK_H

#include "constant_values.hpp"
#include "Eigen/Core"
#include "Eigen/Dense"
#include "Configuration.hpp"
#include "Robot.hpp"
//#include "Nao.hpp"
#include "Hrp4.hpp"
#include "Kinematics.hpp"
//#include "NaoKinematics.hpp"
#include "Hrp4Kinematics.hpp"
#include "matrix_tools.hpp"
#include <fstream>
#include "enum.h"
#include <vector>
#include <map>

class Task {

	public:
		Task();
		virtual ~Task() = 0;

		/**
			* Returns the specific type of the Task.
			*/
		TaskType getTaskType();
		
		/**
			* Defines the (current) final point of a Task. 
			* Repeated calls of this function allow to specify a Task as a composition of multiple subtasks.
			*/
		void addAttractor(Eigen::Vector3f _att);

		/**
			* Returns the index of the last attractor reached by the robot while accomplishing the Task. 
			*/
		int getLastReachedAttractor();

		/**
			* Returns the last attractor of the Task (i.e., the final point). 
			*/
		Eigen::Vector3f getLastAttractor();

		/**
			* Sets the index of the last attractor reached by the robot while accomplishing the Task. 
			*/
		void setLastReachedAttractor();

		/**
			* Returns the i-th attractor of the Task.
			*/
		Eigen::Vector3f getAttractor(float i);
		
		/**
			* Returns the Task position in correspondance of the parameter k.
			* Note that k represents:
			* - a time instant t if the task is specified as a trajectory
			* - a value of the parameter s if the task is specified as a path
			*/
		virtual Eigen::Vector3f getPosition(float k) = 0;

		/**
			* Returns the Task velocity in correspondance of the parameter k.
			* Note that k represents:
			* - a time instant t if the task is specified as a trajectory
			* - a value of the parameter s if the task is specified as a path
			*/
		virtual Eigen::Vector3f getVelocity(float k) = 0;

		/**
			* Creates the entire task (position and velocity) with _init as initial position.
			*/	
		virtual void defineTask(Eigen::Vector3f _init) = 0;

		/**
			* Sets the time instant of the (current) final point of a Task. 
			* Repeated calls of this function allow to specify a Task as a composition of multiple subtasks.
			*/
		virtual void addAttractorTime(float _t_att) = 0;

		/**
			* Returns the time instant associated to the i-th attractor of the Task.
			*/
		virtual float getAttractorTime(float i) = 0;

		/**
			* Returns the duration of the Task.
			*/
		virtual float getDuration() = 0;

		/**
			* Computes the point of the Task that is closest to the considered robot end-effector given its pose.
			*/
		virtual float computeClosestPoint(Eigen::VectorXf vr, float si) = 0;	

	protected:	
		TaskType type;
		std::map<float, Eigen::Vector3f> taskPos;
		std::map<float, Eigen::Vector3f> taskVel;
		std::vector<Eigen::Vector3f> atts;
		std::vector<float> t_atts;
		int lastReached;
		float dur;
};

#endif	/* TASK_H */

