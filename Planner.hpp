/* 
 * File:   Planner.hpp
 * Author: Paolo Ferrari
 *
 * Created on March 8, 2016 
 */

#ifndef PLANNER_H
#define PLANNER_H

#include "constant_values.hpp"
#include <stdio.h>
#include <vector>
#include <map>
#include <iostream>
#include "enum.h"
#include "Eigen/Dense"
#include "Configuration.hpp"
#include "State.hpp"
#include "Node.hpp"
#include "Robot.hpp"
//#include "Nao.hpp"
#include "Hrp4.hpp"
#include "Kinematics.hpp" 
//#include "NaoKinematics.hpp"
#include "Hrp4Kinematics.hpp"
#include "RRTtree.hpp"
#include "Task.hpp"
#include "TaskSetPoint.hpp"
#include "TaskTrajectory.hpp"
#include "TaskPath.hpp"
#include "TaskNavigation.hpp"
#include "MotionGenerator.hpp"
#include "matrix_tools.hpp"


class Planner {

	private:
		
		Configuration qInit;
		Task* task;	
		TaskType taskType;	
		Robot* robot;
		Kinematics* kin;
		RRTtree tree;
		MotionGenerator* mg;

		float time_budget; 
		//int node_root_index;
		std::vector<int> subtree_indexes;
		bool outside_known_area;
		int node_next_index;
		int node_prev_index;
		std::vector<int> current_subtree_indexes;

		int backtracking_phase_iters;

		Eigen::Vector3f pCoMRoot;
		Eigen::Vector3f goal;

		bool outside;
		int outside_count;
		bool complete_plan;
		
		//int findqNearIndex(Eigen::Vector3f trand);
		//int findqNearIndexExploitation(Eigen::Vector3f goal);
		//int findqNearIndexExploration(Eigen::Vector3f root);

		int findqNearIndexExploitation();
		int findqNearIndexExploration();

	
		Eigen::Vector2f computeMidPoint(Configuration& q);
		State extractNewState(std::map<float, Configuration> motion);
		
		int getNextNodeIndex();
		int findqNearIndexInSubtree(Eigen::Vector3f trand);
		int findqNearIndexInCompleteTree(Eigen::Vector3f trand);

		Node extractCurrentNearestNodeToGoal();
		bool isInArea(Configuration& q, Eigen::Vector3f pCoM);

		
		
	public:

		Planner(Configuration& _qInit, Task* _task, Robot* _robot, Kinematics* _kin);
		~Planner();
		//void Run();
		void Run(float _time_budget);
	
		//std::map<float, Configuration> extractPath();
		int getTreeSize();

		//Node extractCurrentNearestNodeToGoal();
		//Node extractCurrentNearestNodeToGoalWT(); // without time
		std::map<float, Configuration> extractCurrentPath();
		
		bool goalReached();

		//float getSimulationTime();

};

#endif
