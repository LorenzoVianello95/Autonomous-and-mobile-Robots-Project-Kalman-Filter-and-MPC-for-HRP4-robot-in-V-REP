/*
 * File:   prim_graph.hpp
 * Author: Paolo Ferrari
 *
 * Created on Saturday June 27, 2017 
 */


#ifndef PRIM_GRAPH_HPP
#define PRIM_GRAPH_HPP

#include <iostream>
#include "Eigen/SVD"
#include "Eigen/Core"
#include "Eigen/Dense"
#include "enum.h"
#include "v_repLib.h"
#include <vector>
#include <fstream>
#include <sstream>
#include <boost/lexical_cast.hpp>

inline Eigen::MatrixXf getPrimGraph(){
	
	Eigen::MatrixXf primGraph((int)PRIMITIVES_SET_SIZE, (int)PRIMITIVES_SET_SIZE);
	primGraph.setZero();

	// 	FREE_COM	// 0 			
	primGraph(0,1) = 1.0; 
	// 	GAIT	// 1 			
	primGraph(1,1) = 1.0; 
	
	/*
	// PESI UGUALI
	// 	FREE_COM	// 0 			
	primGraph(0,1) = 1.0; 
	// 	DYN_F_S_L 	// 1
	primGraph(1,3) = 1.0; 
	
	//	DYN_F_C_LRLm	//18
	primGraph(18,19) = 1.0; 
	primGraph(18,20) = 1.0; 
	primGraph(18,23) = 1.0; 
	primGraph(18,25) = 1.0; 
	//primGraph(18,26) = 1.0; 
	//	DYN_F_C_RLRm	//19
	primGraph(19,18) = 1.0; 
	primGraph(19,21) = 1.0; 
	primGraph(19,22) = 1.0; 
	primGraph(19,24) = 1.0; 
	//primGraph(19,27) = 1.0; 
	//	DYN_B_C_LRLm	//20
	primGraph(20,18) = 1.0; 
	primGraph(20,21) = 1.0; 
	primGraph(20,23) = 1.0; 
	primGraph(20,24) = 1.0; 
	//primGraph(20,27) = 1.0; 
	//	DYN_B_C_RLRm	//21
	primGraph(21,19) = 1.0; 
	primGraph(21,20) = 1.0; 
	primGraph(21,22) = 1.0; 
	primGraph(21,25) = 1.0; 
	//primGraph(21,26) = 1.0; 	
	//	DYN_LL_C_LRm	//22
	primGraph(22,18) = 1.0; 
	primGraph(22,20) = 1.0; 
	primGraph(22,22) = 1.0; 
	primGraph(22,24) = 1.0; 
	//primGraph(22,26) = 1.0; 
	//	DYN_LR_C_RLm	//23
	primGraph(23,19) = 1.0; 
	primGraph(23,21) = 1.0; 
	primGraph(23,23) = 1.0; 
	primGraph(23,25) = 1.0; 
	//primGraph(23,27) = 1.0; 
	//	DYN_FDL_C_LRm	//24
	primGraph(24,19) = 1.0; 
	primGraph(24,20) = 1.0; 
	primGraph(24,23) = 1.0; 
	primGraph(24,25) = 1.0; 
	//	DYN_FDR_C_RLm	//25
	primGraph(25,18) = 1.0; 
	primGraph(25,21) = 1.0; 
	primGraph(25,22) = 1.0; 
	primGraph(25,24) = 1.0; 
	//	DYN_BDL_C_LRm	//26
	//	DYN_BDR_C_RLm	//27

	primGraph(TRANSLATION,TRANSLATION) = 1.0; 
	*/

	return primGraph;
}
#endif /* PRIM_GRAPH_HPP */
