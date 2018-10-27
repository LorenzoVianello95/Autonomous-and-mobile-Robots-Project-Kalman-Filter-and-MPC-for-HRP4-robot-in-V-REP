/* 
 * File:   Node.hpp
 * Author: Paolo Ferrari
 *
 * Created on March 4, 2016 
 */

#ifndef NODE_H
#define NODE_H

#include <stdio.h>
#include <map>
#include <iostream>
#include "enum.h"
#include "Eigen/Dense"
#include "Configuration.hpp"
#include "State.hpp"
#include <vector>
#include "prim_graph.hpp"

class Node {

	private:
		
		State s;
		std::map<float, Configuration> subpath;
		int leaf_index;
		int parent_index;
		
		std::vector<MovementPrimitiveType> available_set;
		Eigen::VectorXf weights_available_set;

		int subtree_root_index;
			    
	public:

		//Node(State _s, std::map<float, Configuration> _subpath, int _leaf_index, int _parent_index, Eigen::VectorXf _weights_available_set);
		Node(State _s, std::map<float, Configuration> _subpath, int _leaf_index, int _parent_index, Eigen::VectorXf _weights_available_set, int _subtree_root_index);
		~Node();
		State getState();
		std::map<float, Configuration> getSubpath();
		int getLeafIndex();
		int getParentIndex();	
		void setParentIndex(int i);

		int getSubtreeRootIndex();	
		void setSubtreeRootIndex(int _subtree_root_index);

		std::vector<MovementPrimitiveType> getAvailableSet();
		Eigen::VectorXf getWeightsAvailableSet();
		void deleteFromAvailableSet(MovementPrimitiveType _type);
		bool hasNonEmptyAvailableSet();
		bool hasEmptyAvailableSet();
		void updateWeightsAvailableSet(MovementPrimitiveType _type);

};

#endif
