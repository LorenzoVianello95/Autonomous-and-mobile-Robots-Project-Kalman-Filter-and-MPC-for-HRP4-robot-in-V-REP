/* 
 * File:   MotionGenerator.hpp
 * Author: Paolo Ferrari
 *
 * Created on February 26, 2016 
 */

#ifndef MOTIONGENERATOR_HPP
#define MOTIONGENERATOR_HPP

#include "constant_values.hpp"
#include "TaskSwFoot.hpp"
#include "TaskCoM.hpp"
#include "Task.hpp"
#include "TaskSetPoint.hpp"
#include "TaskTrajectory.hpp"
#include "TaskPath.hpp"
#include "TaskNavigation.hpp"
#include "Configuration.hpp"
#include "Robot.hpp"
//#include "Nao.hpp"
#include "Hrp4.hpp"
#include "Kinematics.hpp"
//#include "NaoKinematics.hpp"
#include "Hrp4Kinematics.hpp"
#include "Eigen/Core"
#include "Eigen/Dense"
#include "v_repLib.h"
#include "enum.h"
#include "Node.hpp"
#include "State.hpp"
#include "matrix_tools.hpp"
#include "prim_graph.hpp"
#include "MovementPrimitive.hpp"
#include <iostream>
#include <fstream>
#include <map>
#include <math.h> 
#include <vector>
//#include <fstream.h>

class MotionGenerator{

	public:
		MotionGenerator();
		MotionGenerator(Task* _thand, Robot* _robot, Kinematics* _kin);
		virtual ~MotionGenerator();
		//std::map<float, Configuration> generateForwardMotion(Node _n);
		//std::map<float, Configuration> generateForwardMotion(Node _n, float _planning_time);
		std::map<float, Configuration> generateForwardMotion(Node _n, float _planning_time, float _time_budget);

		
		MovementPrimitiveType getPrimitiveType();
		float getMotionDuration();

		bool getBacktrackingPhase();
		void setBacktrackingPhase(bool b);
	
		void setCurrentKnownArea(Eigen::Vector3f _pCoMRoot);
		void setPreviousKnownArea(Eigen::Vector3f _pCoMRootPrev);
		void setExplorationRef(Eigen::Vector3f _pCoMRef);

		void clearCoMPositions();
		void addCoMPosition(Eigen::Vector3f pCoM);

	private:
		Configuration qCurr; 
		float tk;	
		Robot* robot;
		Kinematics* kin;
		MovementPrimitive* prim;
		float intStep;  
		int dof;  
		EndEffector support;
		EndEffector swing;
		Eigen::Matrix4f init_tran;
		Eigen::Matrix4f Thand_w;
		Eigen::Matrix4f Tsup_w;
		Eigen::Matrix4f Tswg_w;

		Eigen::Vector3f pCoMRoot;
		Eigen::Vector3f pCoMRootPrev;
		std::vector<Eigen::Vector3f> CoMPositions;
		Eigen::Vector3f pCoMRef;

		Eigen::Vector3f phand_init;

		MovementPrimitiveType typeCurr;
		MovementPrimitiveType type;
		float duration;

		Task* task;
		TaskType taskType;
		float d_heuristic; // 

		float planning_time; //////////////
		float time_budget; //////////////

		bool backtracking_phase;
		int backtracking_phase_counter;

		Configuration FirstOrderIntegrator(Configuration& q, float t, float h);
		Configuration RungeKutta4(Configuration& q, float t, float h);
		Eigen::VectorXf f(float t, Configuration& q);
		Configuration update_q(Configuration& q, float h, Eigen::VectorXf k);

		Eigen::VectorXf computeGradHrange(Configuration& q);
		float computeHrange(Eigen::VectorXf q);

		Eigen::VectorXf computeRandomVelocities();
		Eigen::VectorXf normalize(Eigen::VectorXf qR_dot, float norm);
		Eigen::VectorXf computeBalanceVelocities(Configuration& q);

		Eigen::VectorXf computeJLA(Configuration& q, Eigen::VectorXf q1_dot, Eigen::MatrixXf P1);
		Eigen::VectorXf computeJLA_i(Eigen::VectorXf qjnt, int i, Eigen::VectorXf q1_dot, Eigen::MatrixXf P1);
		Eigen::VectorXf computeActivationSignFunction(Eigen::VectorXf qjnt, int i);
		float computeTuningFunction(Eigen::VectorXf qjnt, int i);
		float computeAdaptiveGainFunction(Eigen::VectorXf qjnt, int i, Eigen::VectorXf q1_dot, Eigen::MatrixXf P1, Eigen::VectorXf g_i);

		bool jointLimitsCheck(Configuration& q);
		bool collisionCheck(Configuration& q);	
		bool occlusionCheck(Configuration& q, float t);
		bool occlusionCheckPoint(Configuration& q, Eigen::Vector3f ps);

		Eigen::VectorXf sign(Eigen::VectorXf eM);

		MovementPrimitiveType chooseMovementPrimitive(std::vector<MovementPrimitiveType> available_set, Eigen::VectorXf weights_available_set);

		bool equilibriumCheck(Configuration& q);

		float getSimulationTime();

		
		bool isInArea(Configuration& q, Eigen::Vector3f pCoM);

		float angleSignedDistance(float a, float b);
		
};

#endif  /* MOTIONGENERATOR_HPP */
