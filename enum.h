#pragma once

enum JointIndexes {
	NECK_Y,			// 0
	NECK_P,			// 1
	R_SHOULDER_P,	// 2
	R_SHOULDER_R,	// 3
	R_SHOULDER_Y,	// 4
	R_ELBOW_P,		// 5
	R_WRIST_Y,		// 6
	R_WRIST_P,		// 7
	R_WRIST_R,		// 8
	L_SHOULDER_P,	// 9
	L_SHOULDER_R,	// 10
	L_SHOULDER_Y,	// 11
	L_ELBOW_P,		// 12
	L_WRIST_Y,		// 13
	L_WRIST_P,		// 14
	L_WRIST_R,		// 15
	CHEST_P,		// 16
	CHEST_Y,		// 17
	R_HIP_Y,		// 18
	R_HIP_R,		// 19
	R_HIP_P,		// 20
	R_KNEE_P,		// 21
	R_ANKLE_P,		// 22
	R_ANKLE_R,		// 23
	L_HIP_Y,		// 24
	L_HIP_R,		// 25
	L_HIP_P,		// 26
	L_KNEE_P,		// 27
	L_ANKLE_P,		// 28
	L_ANKLE_R		// 29
};

enum EndEffector {
	L_FOOT,
	R_FOOT,
	L_HAND,
	R_HAND,
	HEAD,
	COM,
	TOP_CAMERA,
	TORSO
};


enum MovementPrimitiveType {
	FREE_COM, 	// 0	
	GAIT,		// 1 		
	PRIMITIVES_SET_SIZE			
};

enum TaskType {
	TASK_SETPOINT,
	TASK_TRAJECTORY,
	TASK_PATH,
	TASK_NAVIGATION,
	TASK_TRAJECTORY_VISUAL,
	TASK_PATH_VISUAL
};

