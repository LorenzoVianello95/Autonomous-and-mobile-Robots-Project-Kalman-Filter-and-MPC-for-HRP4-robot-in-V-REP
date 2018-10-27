/*
 * File:   constant_values.h
 * Author: Paolo Ferrari
 *
 * Created on March 14, 2016 
 */

#ifndef CONSTANT_VALUES_HPP
#define	CONSTANT_VALUES_HPP

const float INTEGRATION_STEP = 0.025;

const int MAX_ITERATION = 200; 
const float EXPLORATION_RATE = 0.3;
const float TOLERANCE_FROM_GOAL = 0.01;

// Threshoulds for the three "zones" (manipulation, loco-manipulation, locomotion)
const float D0 = 0.15;
const float D1 = 0.45;
const float D2 = 0.6;

const float E = 2.718281828459045;

#endif	/* CONSTANT_VALUES_HPP */
