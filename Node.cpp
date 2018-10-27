/* 
 * File:   State.hpp
 * Author: Paolo Ferrari
 *
 * Created on March 4, 2016 
 */

#include "Node.hpp"
#include <stdlib.h>

Node::Node(State _s, std::map<float, Configuration> _subpath, int _leaf_index, int _parent_index, Eigen::VectorXf _weights_available_set, int _subtree_root_index){
	s = _s;	
	subpath = _subpath;
	leaf_index = _leaf_index;
	parent_index = _parent_index;

	subtree_root_index = _subtree_root_index; //

	MovementPrimitiveType type = s.getMovementPrimitiveType();

	Eigen::MatrixXf primGraph = getPrimGraph();
	// construct the Available Set
	for(int i = 0; i < PRIMITIVES_SET_SIZE; i++){
		if(primGraph(type, i) > 0.0) available_set.push_back((MovementPrimitiveType)i);
	}
	
	if(_weights_available_set(0) > 0.0) weights_available_set = _weights_available_set; 
	else{
		weights_available_set.resize(available_set.size());
		weights_available_set.setZero();
		float val;
		int j = 0;	
		for(int i = 0; i < PRIMITIVES_SET_SIZE; i++){
			if(primGraph(type, i) > 0.0){
				val = primGraph(type, i);
				//val = primGraph(type, i) / 100.0;
				//val = 1.0/(float)available_set.size();
				weights_available_set(j) = val;
				j++;
			}
		}
	}
}

Node::~Node(){ }

State Node::getState(){
	return s;
}

std::map<float, Configuration> Node::getSubpath(){
	return subpath;
}

int Node::getLeafIndex(){
	return leaf_index;
}

int Node::getParentIndex(){
	return parent_index;
}

int Node::getSubtreeRootIndex(){
	return subtree_root_index;
}

void Node::setSubtreeRootIndex(int _subtree_root_index){
	subtree_root_index = _subtree_root_index;
}

void Node::setParentIndex(int i){
	parent_index = i;
}

std::vector<MovementPrimitiveType> Node::getAvailableSet(){
	return available_set;
}

Eigen::VectorXf Node::getWeightsAvailableSet(){
	return weights_available_set;
}

void Node::deleteFromAvailableSet(MovementPrimitiveType _type){
	float add = 0.0;
	float del = 0.0;	
	for(int i = 0; i < available_set.size(); i++){
		if(available_set.at(i) == _type){
			del = weights_available_set(i);
			weights_available_set(i) = 0.0;
		} 
	}
	int nZero = 0;
	for(int i = 0; i < available_set.size(); i++){
		if(weights_available_set(i) == 0) nZero++;
	}
	add = del / (float)(available_set.size() - nZero);
			
	for(int i = 0; i < available_set.size(); i++){
		if(weights_available_set(i) > 0) weights_available_set(i) = weights_available_set(i) + add; 
	}	
}

void Node::updateWeightsAvailableSet(MovementPrimitiveType _type){
	float b;
	//float alpha = 2.0;
	float alpha = 100.0;
	for(int i = 0; i < available_set.size(); i++){
		if(available_set.at(i) == _type){
			//b = weights_available_set(i) / alpha;
			//weights_available_set(i) = weights_available_set(i) - b;
			weights_available_set(i) = weights_available_set(i) / alpha;
		} 
	}
}

bool Node::hasNonEmptyAvailableSet(){
	bool res = true;
	int nZeros = 0;

	for(int i = 0; i < available_set.size(); i++){
		if(weights_available_set(i) < 0.01) nZeros++;
	}

	if(nZeros == available_set.size()) res = false;
	
	return res;
}

bool Node::hasEmptyAvailableSet(){
	bool res = false;
	int n = 0;	

	for(int i = 0; i < available_set.size(); i++){
		if(weights_available_set(i) < 0.01) n++;
	}
	
	if(n == available_set.size()) res = true;  
	return res;
}








