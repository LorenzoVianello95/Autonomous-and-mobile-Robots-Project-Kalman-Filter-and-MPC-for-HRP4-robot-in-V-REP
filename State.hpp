/* 
 * File:   State.hpp
 * Author: Paolo Ferrari
 *
 * Created on March 4, 2016 
 */

#ifndef STATE_H
#define STATE_H

#include <stdio.h>
#include <iostream>
#include "enum.h"
#include "Eigen/Dense"
#include "Configuration.hpp"
#include <vector>
#include "prim_graph.hpp"

class State {

	private:	
		Configuration q;
		MovementPrimitiveType type;
		float t;
	    
	public:
		State();
		State(Configuration& _q, MovementPrimitiveType _type, float _t);
		~State();

		Configuration getConfiguration();
		MovementPrimitiveType getMovementPrimitiveType();
		float getTime();
		
};

#endif
