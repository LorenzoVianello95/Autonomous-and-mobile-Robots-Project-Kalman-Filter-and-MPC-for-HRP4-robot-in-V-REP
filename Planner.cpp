/* 
 * File:   Planner.hpp
 * Author: Paolo Ferrari
 *
 * Created on March 8, 2016 
 */

#include "Planner.hpp"
#include <stdlib.h> 
#include <algorithm>

Planner::Planner(Configuration& _qInit, Task* _task, Robot* _robot, Kinematics* _kin){ 
	Eigen::Vector3f pCoM = _qInit.getqCoMPosition();
	Eigen::Vector3f oCoM = _qInit.getqCoMOrientation();
	qInit.setqCoMPosition(pCoM);
	qInit.setqCoMOrientation(oCoM);
	qInit.setqjnt(_qInit.getqjnt());
	task = _task;
	robot = _robot;
	kin = _kin;
	taskType = task->getTaskType();	

	mg = new MotionGenerator(task, robot, kin);    			 	
}

Planner::~Planner(){};

void Planner::Run(float _time_budget){

	//time_budget = _time_budget;

	time_budget = 5.0; ////////////////////////////////////

	int nFailsExploitation = 0;
	int nFailsExploration = 0;
	int nFailsExploitationMax = 5; 
	int nFailsExplorationMax = 5;
	
	float planning_time = 0.0;
	float t_start = simGetSimulationTime();	
	float t_current;
		
	if(tree.getSize() == 0){
		State s0(qInit, FREE_COM, 0);
		std::map<float, Configuration> motion0;  
		Eigen::Vector2f weights_available_set0;  
	 	weights_available_set0.setZero();    
		Node n0(s0, motion0, 0.0, -1, weights_available_set0, 0);	
		tree.addNode(n0);
		pCoMRoot = n0.getState().getConfiguration().getqCoMPosition();
		mg->setExplorationRef(pCoMRoot);
	}
	
	//Eigen::Vector3f pCoMRootPrev;	
	//Node root_prev = tree.getNodeAt(0);
	Node root_curr = extractCurrentNearestNodeToGoal();
	//pCoMRootPrev = root_prev.getState().getConfiguration().getqCoMPosition();

	tree.clearTree();
	State s0 = root_curr.getState();
	std::map<float, Configuration> motion0;  
	Eigen::VectorXf weights_available_set0 = root_curr.getWeightsAvailableSet();		
	int leaf_index0 = root_curr.getLeafIndex();
	int parent_index0 = root_curr.getParentIndex();
	Node n0(s0, motion0, 0.0, -1, weights_available_set0, 0);	
	tree.addNode(n0);
	
	int root_index = 0;		
	pCoMRoot = n0.getState().getConfiguration().getqCoMPosition();
	mg->setCurrentKnownArea(pCoMRoot);

	outside = false;
	//mg->setPreviousKnownArea(pCoMRootPrev);
	goal = task->getAttractor(task->getLastReachedAttractor()+1);
	
	int i = 0;
	//float t = 0.0;
	Eigen::Vector3f trand;

	float rs = 0.50;
	float alpha_rs = 0.25;

	while(!goalReached() && i < 1){
		i++;

		int index_qNear = tree.getSize() - 1; 
		//float r = static_cast<float>(rand()) / (static_cast<float>(RAND_MAX));
		//if(r < rs) index_qNear = findqNearIndexExploitation();
		//else index_qNear = findqNearIndexExploration();

		
		Node nNear = tree.getNodeAt(index_qNear);
		std::map<float, Configuration> motion = mg->generateForwardMotion(nNear, planning_time, time_budget);
		std::cout << "motion.size():: " << motion.size() << std::endl;
	
		if(motion.size() > 0){
			State sNew = extractNewState(motion);
			 
			Eigen::VectorXf weights_available_set;
			weights_available_set.resize(1);
			weights_available_set.setZero();
			for(int j = 0; j < tree.getSize(); j++){
				int j_parent = tree.getNodeAt(j).getParentIndex();
				MovementPrimitiveType j_type = tree.getNodeAt(j).getState().getMovementPrimitiveType();
				if(j_parent == index_qNear && j_type == mg->getPrimitiveType()){			
					weights_available_set.resize(tree.getNodeAt(j).getWeightsAvailableSet().size());
					weights_available_set = tree.getNodeAt(j).getWeightsAvailableSet();
					break;
				}
			}
		
			Node nNew(sNew, motion, tree.getNodeAt(index_qNear).getLeafIndex() + 1, index_qNear, weights_available_set, root_index);	 				
			tree.addNode(nNew);
			Configuration qNew = nNew.getState().getConfiguration();
			MovementPrimitiveType typeNew = mg->getPrimitiveType();
			Eigen::Vector3f pCoMNew = qNew.getqCoMPosition();
			
			MovementPrimitiveType typeNear = nNear.getState().getMovementPrimitiveType();
		
			//if(isInArea(qNew, pCoMRoot)) std::cout << "IN..." << "nNear:: " << typeNear << " nNew:: " << typeNew << std::endl; 
			//if(!isInArea(qNew, pCoMRoot) && typeNew != TRANSLATION) std::cout << "OUT..." << "nNear:: " << typeNear << " nNew:: " << typeNew << std::endl; 
			//if(!isInArea(qNew, pCoMRoot) && typeNew == TRANSLATION) std::cout << "TRANS..." << "nNear:: " << typeNear << " nNew:: " << typeNew << std::endl; 
			

			//if(!isInArea(qNew, pCoMRoot) && typeNew == TRANSLATION) outside = true;
			
			//std::cout << "fabs() " << typeNew << std::endl; 
			//tree.updateNode(index_qNear, mg->getPrimitiveType()); 
		}
		else{
			tree.updateNode(index_qNear, mg->getPrimitiveType()); 
			MovementPrimitiveType typeNew = mg->getPrimitiveType();	 
			std::cout << "EXPANSION FAILED " << typeNew << std::endl; 
			
			/* 
			if(r < rs){
				nFailsExploitation++;
				if(nFailsExploitation > nFailsExploitationMax){
					nFailsExploitation = 0;
					rs = std::max(0.0f, rs - alpha_rs); 
				}
			}	
			else{
				nFailsExploration++;
				if(nFailsExploration > nFailsExplorationMax){
					nFailsExploration = 0;
					rs = std::min(1.0f, rs + alpha_rs); 
				}
			}			
			*/ 
			
		}		
		t_current = simGetSimulationTime();	
		planning_time = t_current - t_start;

	}
	
	//if(goalReached()) std::cout << "PATH FOUND!!!" << std::endl; 
	//else std::cout << "PATH NOT FOUND!!!" << std::endl; 		

	std::cout << "# NODES: " << tree.getSize() << std::endl;
}

State Planner::extractNewState(std::map<float, Configuration> motion){
	std::map<float, Configuration>::iterator it;	
	it = motion.end();
	it--;
	float tk = it->first;
	Configuration q = it->second;
	MovementPrimitiveType type = mg->getPrimitiveType();
	State s(q, type, tk);

	return s;	
}
	
int Planner::findqNearIndexExploitation(){
	int n = tree.getSize();  
	
	Eigen::Vector2f trand;
	trand << goal(0), goal(1);


	// find the nearest to trand
	Eigen::Vector3f pCoM;
	State s = tree.getStateAt(0);
	Configuration q = s.getConfiguration();
	pCoM = q.getqCoMPosition();
	Eigen::Vector2f mf = computeMidPoint(q);
	
	int imin = 0;
	//float min_dist = std::sqrt(std::pow(trand(0)-pCoM(0),2)+std::pow(trand(1)-pCoM(1),2)); 	
	float min_dist = std::sqrt(std::pow(trand(0)-mf(0),2)+std::pow(trand(1)-mf(1),2)); 	
	
	int i = 1;	
	float dist = 0.0;
	while(i < tree.getSize()){
		s = tree.getStateAt(i);	
		q = s.getConfiguration();
		pCoM = q.getqCoMPosition();
		mf = computeMidPoint(q);
		//dist = std::sqrt(std::pow(trand(0)-pCoM(0),2)+std::pow(trand(1)-pCoM(1),2)); 
		dist = std::sqrt(std::pow(trand(0)-mf(0),2)+std::pow(trand(1)-mf(1),2)); 			
		if(dist < min_dist){
			min_dist = dist;
			imin = i;
		}

		i++;
	}

	return imin;
	
}

int Planner::findqNearIndexExploration(){
	int n = tree.getSize();  
	
	Eigen::Vector2f trand;
	// randomly generate trand
	float thetaRand = (0.0) + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX/(6.28)));
	//float thetaRand = 3.14/2.0;
	float l = 3.0;
	trand(0) = pCoMRoot(0) + l * std::sin(thetaRand);
	trand(1) = pCoMRoot(1) + l * std::cos(thetaRand);	
			
	// find the nearest to trand
	Eigen::Vector3f pCoM;
	State s = tree.getStateAt(0);
	Configuration q = s.getConfiguration();
	pCoM = q.getqCoMPosition();
	Eigen::Vector2f mf = computeMidPoint(q);
	
	int imin = 0;
	//float min_dist = std::sqrt(std::pow(trand(0)-pCoM(0),2)+std::pow(trand(1)-pCoM(1),2)); 	
	float min_dist = std::sqrt(std::pow(trand(0)-mf(0),2)+std::pow(trand(1)-mf(1),2)); 	

	int i = 1;	
	float dist = 0.0;
	while(i < tree.getSize()){
		s = tree.getStateAt(i);	
		q = s.getConfiguration();
		pCoM = q.getqCoMPosition();
		mf = computeMidPoint(q);
		//dist = std::sqrt(std::pow(trand(0)-pCoM(0),2)+std::pow(trand(1)-pCoM(1),2)); 
		dist = std::sqrt(std::pow(trand(0)-mf(0),2)+std::pow(trand(1)-mf(1),2)); 			
		if(dist < min_dist){
			min_dist = dist;
			imin = i;
		}

		i++;
	}

	return imin;
	
}
  
bool Planner::goalReached(){
	bool reached = false;
	Eigen::Vector3f att_to_reach = task->getAttractor(task->getLastReachedAttractor() + 1);
	Eigen::Vector3f goal = task->getLastAttractor();
	if(goal != att_to_reach) return false; 
	
	// get the last added node
	int last_index = tree.getSize() - 1;
	State s = tree.getStateAt(last_index);
	Configuration q = s.getConfiguration();
	// get position of the ee in q
	Eigen::Matrix4f Tee_w = kin->forwardKinematics(q, R_HAND);
	Eigen::VectorXf vee = t2v(Tee_w);
	Eigen::Vector2f midfeet = computeMidPoint(q);	
	Eigen::MatrixXf Tc_w = kin->forwardKinematics(q, TOP_CAMERA);
	Eigen::VectorXf vc_w = t2v(Tc_w);	

	Eigen::MatrixXf Tw_c = Tc_w.inverse();
	Eigen::VectorXf p_w_hom(4);
	p_w_hom << goal(0), goal(1), goal(2), 1.0;	
	Eigen::VectorXf p_c_hom(4);
	p_c_hom << Tw_c * p_w_hom;
	float lambda = 0.2;
	float u = lambda * p_c_hom(0)/p_c_hom(2);
	float v = lambda * p_c_hom(1)/p_c_hom(2);
	float t_task = task->getDuration();
	float t_current = s.getTime();
	
	float dist = 0.0;
	if(taskType == TASK_TRAJECTORY_VISUAL) dist = t_task - t_current;
	else if(taskType == TASK_PATH_VISUAL) dist = 1.0 - task->computeClosestPoint(vc_w, 0.0);	
	else if(taskType == TASK_NAVIGATION) dist = std::sqrt(std::pow(midfeet(0) - goal(0), 2) + std::pow(midfeet(1) - goal(1), 2));
	else dist = std::sqrt(std::pow(vee(0) - goal(0), 2) + std::pow(vee(1) - goal(1), 2) + std::pow(vee(2) - goal(2), 2));

	if(taskType == TASK_NAVIGATION && dist < 5.0*TOLERANCE_FROM_GOAL) reached = true;
	else if(dist < TOLERANCE_FROM_GOAL) reached = true;

	return reached;		
}


int Planner::getTreeSize(){
	return tree.getSize();
}

int Planner::getNextNodeIndex(){
	return node_next_index;
}

 
/*
std::map<float, Configuration> Planner::extractCurrentPath(){

	std::map<float, Configuration> path;
	std::map<float, Configuration> pathAux;
	std::vector<Node> pathNodes;

	return path;

}
*/

std::map<float, Configuration> Planner::extractCurrentPath(){

	std::map<float, Configuration> path;
	std::map<float, Configuration> pathAux;
	std::vector<Node> pathNodes;

	Eigen::Vector3f goal = task->getAttractor(0);

	int n; // index of the next subgoal	
	n = tree.getSize() - 1;
		
	if(n > 0){
		Node v = tree.getNodeAt(n);
		while(v.getParentIndex() != -1){
			Configuration qv = v.getState().getConfiguration();
			if(isInArea(qv, pCoMRoot)) pathNodes.push_back(v);
			v = tree.getNodeAt(v.getParentIndex());
		}
		pathNodes.push_back(v);
		std::cout << "---------------- pathNodes.size():: " << pathNodes.size() << std::endl;
	}
	else{
		Node v = tree.getNodeAt(n);
		pathNodes.push_back(v);
	}
	std::reverse(pathNodes.begin(), pathNodes.end());

	for(int i = 0; i < pathNodes.size(); i++){
		Node v = pathNodes.at(i);
		std::map<float, Configuration> subpath = v.getSubpath();
		pathAux.insert(subpath.begin(), subpath.end());	
	} 		

	std::map<float, Configuration>::iterator itA;
	itA = pathAux.begin();
	float t_init = 0.0;
	if(itA != pathAux.end()) t_init = itA->first; 
	while(itA != pathAux.end()){
		float t = itA->first - t_init; 				
		Configuration q = itA->second;		
		path.insert(std::pair<float, Configuration>(t, q)); 
		itA++;
	}
	
	return path;

}

Node Planner::extractCurrentNearestNodeToGoal(){

	Node res = tree.getNodeAt(0);

	return res;

}
 


bool Planner::isInArea(Configuration& q, Eigen::Vector3f pCoM){
	// computing the control points on the robot

	Eigen::VectorXf v1_h(6), v2_lh(6), v3_rh(6), v4_lf(6), v5_lf(6), v6_rf(6), v7_rf(6);
	v1_h << 0.09, 0.0, 0.0, 0.0, 0.0, 0.0; 
	v2_lh << 0.05, -0.007, 0.06, 0.0, 0.0, 0.0; 
	v3_rh << 0.06, 0.0, -0.01, 0.0, 0.0, 0.0; 
	v4_lf << 0.01, 0.0, 0.0, 0.0, 0.0, 0.0; 
	v5_lf << -0.065, 0.0, 0.0, 0.0, 0.0, 0.0; 
	v6_rf << 0.01, 0.0, 0.0, 0.0, 0.0, 0.0; 
	v7_rf << -0.065, 0.0, 0.0, 0.0, 0.0, 0.0; 
	Eigen::MatrixXf T1_h = v2t(v1_h);
	Eigen::MatrixXf T2_lh = v2t(v2_lh);
	Eigen::MatrixXf T3_rh = v2t(v3_rh);
	Eigen::MatrixXf T4_lf = v2t(v4_lf);
	Eigen::MatrixXf T5_lf = v2t(v5_lf);
	Eigen::MatrixXf T6_rf = v2t(v6_rf);
	Eigen::MatrixXf T7_rf = v2t(v7_rf);

	Eigen::MatrixXf Th_w = kin->forwardKinematics(q, HEAD);
	Eigen::MatrixXf Tlh_w = kin->forwardKinematics(q, L_HAND);
	Eigen::MatrixXf Trh_w = kin->forwardKinematics(q, R_HAND);
	Eigen::MatrixXf Tlf_w = kin->forwardKinematics(q, L_FOOT);			
	Eigen::MatrixXf Trf_w = kin->forwardKinematics(q, R_FOOT);			
	
	Eigen::MatrixXf T1 = Th_w * T1_h;
	Eigen::MatrixXf T2 = Tlh_w * T2_lh;
	Eigen::MatrixXf T3 = Trh_w * T3_rh;
	Eigen::MatrixXf T4 = Tlf_w * T4_lf;
	Eigen::MatrixXf T5 = Tlf_w * T5_lf;
	Eigen::MatrixXf T6 = Trf_w * T6_rf;
	Eigen::MatrixXf T7 = Trf_w * T7_rf;

	Eigen::VectorXf v1 = t2v(T1);
	Eigen::VectorXf v2 = t2v(T2);
	Eigen::VectorXf v3 = t2v(T3);
	Eigen::VectorXf v4 = t2v(T4);
	Eigen::VectorXf v5 = t2v(T5);
	Eigen::VectorXf v6 = t2v(T6);
	Eigen::VectorXf v7 = t2v(T7);

	float d1 = sqrt(std::pow(pCoM(0) - v1(0), 2) + std::pow(pCoM(1) - v1(1), 2) + std::pow(pCoM(2) - v1(2), 2));
	float d2 = sqrt(std::pow(pCoM(0) - v2(0), 2) + std::pow(pCoM(1) - v2(1), 2) + std::pow(pCoM(2) - v2(2), 2));
	float d3 = sqrt(std::pow(pCoM(0) - v3(0), 2) + std::pow(pCoM(1) - v3(1), 2) + std::pow(pCoM(2) - v3(2), 2));
	float d4 = sqrt(std::pow(pCoM(0) - v4(0), 2) + std::pow(pCoM(1) - v4(1), 2) + std::pow(pCoM(2) - v4(2), 2));
	float d5 = sqrt(std::pow(pCoM(0) - v5(0), 2) + std::pow(pCoM(1) - v5(1), 2) + std::pow(pCoM(2) - v5(2), 2));
	float d6 = sqrt(std::pow(pCoM(0) - v6(0), 2) + std::pow(pCoM(1) - v6(1), 2) + std::pow(pCoM(2) - v6(2), 2));
	float d7 = sqrt(std::pow(pCoM(0) - v7(0), 2) + std::pow(pCoM(1) - v7(1), 2) + std::pow(pCoM(2) - v7(2), 2));  

	//std::cout << "fabs(t2v(Tlf_w)(1) - pCoM(1)):: " << fabs(t2v(Tlf_w)(1) - pCoM(1)) << std::endl;
	//float dlf = fabs(t2v(Tlf_w)(1) - pCoM(1));
	//float drf = fabs(t2v(Trf_w)(1) - pCoM(1));
	//if(dlf > 0.1 || drf > 0.1) return false;

	float r = 5.0;
	if(d1 < r && d2 < r && d3 < r && d4 < r && d5 < r && d6 < r && d7 < r) return true;
	return false; 
}

Eigen::Vector2f Planner::computeMidPoint(Configuration& q){
	Eigen::Matrix4f Trf_w = kin->forwardKinematics(q, R_FOOT);
	Eigen::Matrix4f Tlf_w = kin->forwardKinematics(q, L_FOOT);
	Eigen::VectorXf vrf = t2v(Trf_w);
	Eigen::VectorXf vlf = t2v(Tlf_w);
	Eigen::Vector2f mid_feet;
	mid_feet(0) = (vrf(0) + vlf(0)) / 2;
	mid_feet(1) = (vrf(1) + vlf(1)) / 2;

	return mid_feet;		
}


