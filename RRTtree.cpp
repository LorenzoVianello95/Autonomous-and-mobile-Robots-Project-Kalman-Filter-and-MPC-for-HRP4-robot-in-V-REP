/* 
 * File:   RRTtree.hpp
 * Author: Paolo Ferrari
 *
 * Created on March 8, 2016 
 */

#include "RRTtree.hpp"
#include <stdlib.h>
#include <algorithm>

RRTtree::RRTtree(){}

RRTtree::~RRTtree(){}

State RRTtree::getStateAt(int i){
	Node n = tree.at(i);
	State s = n.getState();
	return s;
}

void RRTtree::addNode(Node n){
	tree.push_back(n);
}

std::map<float, Configuration> RRTtree::extractPath(){
	std::map<float, Configuration> path;
	std::vector<Node> pathNodes;
	int n = tree.size();
	if(n > 0){
		Node v = tree.at(n-1);
		while(v.getParentIndex() != -1){
			pathNodes.push_back(v);
			v = tree.at(v.getParentIndex());
		}
		pathNodes.push_back(v);
	}
	std::reverse(pathNodes.begin(), pathNodes.end());

	for(int i = 0; i < pathNodes.size(); i++){
		Node v = pathNodes.at(i);
		std::map<float, Configuration> subpath = v.getSubpath();
		path.insert(subpath.begin(), subpath.end());	
	} 		
	
	return path;
}

int RRTtree::getSize(){
	return tree.size();
}

Node RRTtree::getNodeAt(int i){
	Node n = tree.at(i);
	return n;
}

void RRTtree::updateNode(int i, MovementPrimitiveType type){
	Node n = tree.at(i);
	int index_parent = n.getParentIndex();
	MovementPrimitiveType type_child = n.getState().getMovementPrimitiveType();
	for(int j = 0; j < tree.size(); j++){
		n = tree.at(j);
		int j_parent = n.getParentIndex();
		MovementPrimitiveType j_type = n.getState().getMovementPrimitiveType();
		if(j_parent == index_parent && j_type == type_child){
			n.updateWeightsAvailableSet(type);
			tree.at(j) = n;
		}
	}	 
}

void RRTtree::clearTree(){
	tree.clear();
}

void RRTtree::updateSubtreeIndex(int i, int root){
	Node n = tree.at(i);
	n.setSubtreeRootIndex(root);
	tree.at(i) = n;
}

