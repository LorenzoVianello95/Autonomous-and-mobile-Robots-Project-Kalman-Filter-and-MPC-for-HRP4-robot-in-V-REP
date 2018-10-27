/* 
 * File:   RRTtree.hpp
 * Author: Paolo Ferrari
 *
 * Created on March 8, 2016 
 */

#ifndef RRTTREE_H
#define RRTTREE_H

#include <stdio.h>
#include <vector>
#include <map>
#include <iostream>
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
#include "enum.h"

class RRTtree {

	private:		
		std::vector<Node> tree;
	   
	public:
		RRTtree();
		~RRTtree();

		State getStateAt(int i);
		void addNode(Node n);
		std::map<float, Configuration> extractPath();
		int getSize();
		Node getNodeAt(int i);
		void updateNode(int i, MovementPrimitiveType _type);

		void clearTree(); //

		void updateSubtreeIndex(int i, int root);
		
};

#endif
