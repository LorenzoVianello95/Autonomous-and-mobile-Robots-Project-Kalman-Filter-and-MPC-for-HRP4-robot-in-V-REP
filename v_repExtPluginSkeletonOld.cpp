// Copyright 2006-2013 Dr. Marc Andreas Freese. All rights reserved.
// marc@coppeliarobotics.com
// www.coppeliarobotics.com
//
// -------------------------------------------------------------------
// This file is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//
// You are free to use/modify/distribute this file for whatever purpose!
// -------------------------------------------------------------------
//
// This file was automatically created for V-REP release V3.0.3 on April 29th 2013

#include <unistd.h>
#include "v_repExtPluginSkeleton.h"
#include "v_repLib.h"
#include <iostream>
#include <fstream>
#include "constant_values.hpp"
#include "Configuration.hpp"
#include "Joint.hpp"
#include "Robot.hpp"
//#include "Nao.hpp"
#include "Hrp4.hpp"
#include "Kinematics.hpp"
//#include "NaoKinematics.hpp"
#include "Hrp4Kinematics.hpp"
#include "TaskSwFoot.hpp"
#include "TaskCoM.hpp"
#include "Task.hpp"
#include "TaskSetPoint.hpp"
#include "TaskTrajectory.hpp"
#include "TaskPath.hpp"
#include "TaskNavigation.hpp"
#include "TaskTrajectoryVisual.hpp"
#include "TaskPathVisual.hpp"
#include "MotionGenerator.hpp"
#include "matrix_tools.hpp"
#include "MovementPrimitive.hpp"
#include <map>
#include "enum.h"
#include "State.hpp"
#include "Node.hpp"
#include "RRTtree.hpp"
#include "Planner.hpp"
#include "MPCSolver.hpp"
#include <ctime>
#include <fenv.h>
#include <time.h>
#include <fstream>
#include "termcolor.hpp"
#include "imgui.h"
#include "imgui_impl_glfw_gl3.h"
#include <stdio.h>
#include <GL/gl3w.h>    // This example is using gl3w to access OpenGL functions (because it is small). You may use glew/glad/glLoadGen/etc. whatever already works for you.
#include <GLFW/glfw3.h>
#include <algorithm>
#include <vector>

#ifdef _WIN32
    #include <shlwapi.h>
    #pragma comment(lib, "Shlwapi.lib")
#endif /* _WIN32 */

#if defined (__linux) || defined (__APPLE__)
    #include <string.h>
    #include <sys/time.h>
    #define _stricmp(x,y) strcasecmp(x,y)
#endif

#define PLUGIN_VERSION 1
#define EIGEN_INITIALIZE_MATRICES_BY_ZERO

#define LEFT_SUPPORT false
#define RIGHT_SUPPORT true

LIBRARY vrepLib; // the V-REP library that we will dynamically load and bind

simInt * btnAux;
simInt uiHndl, btnStat;
Robot* robot;
Kinematics* kin;
Configuration currentConfiguration;
Eigen::VectorXf actualConfiguration(30);
std::map<float, Configuration> motion;
std::map<float, Configuration>::iterator it;
simInt sp, sp0, sp1;
simInt spKnob, spBall, spMF;
Eigen::Vector3f goal;
float t1, t2, t3;
Task* task;
float dg;
simFloat d_ball[3];
int timeIndex=0;
std::vector<simInt> joints;
Eigen::VectorXf initialFootPosition(6);
Eigen::VectorXf currentFootPosition(6);
Eigen::VectorXf currentCoMPosition(3);
Eigen::VectorXf currentTorsoPosition(3);
Eigen::VectorXf currentPosition(12);
simFloat* poseShift = NULL;
simFloat* initialPoseShift = NULL;
bool supportFoot=LEFT_SUPPORT;
Eigen::VectorXf initialFootPositionLeftSupport(6);
Eigen::VectorXf initialCoMPositionLeftSupport(3);
Eigen::VectorXf initialTorsoPositionLeftSupport(6);

Eigen::VectorXf initialFootPositionRightSupport(6);
Eigen::VectorXf initialCoMPositionRightSupport(3);
Eigen::VectorXf initialTorsoPositionRightSupport(6);

/////////JACOBIANS/////////
Eigen::MatrixXf jacobianFoot(6,30);
Eigen::MatrixXf jacobianFootTranspose(30,6);
Eigen::VectorXf taskFootError(6);
Eigen::VectorXf taskFootPosition(6);
Eigen::VectorXf taskFootVelocity(6);
Eigen::MatrixXf swingFootTask(6,2);
Eigen::MatrixXf pseudoInverseFootJacobian(30,6);

Eigen::MatrixXf jacobianCoM(3,30);
Eigen::MatrixXf jacobianCoMTranspose(30,3);
Eigen::VectorXf taskCoMError(3);
Eigen::VectorXf taskCoMPosition(3);
Eigen::VectorXf taskCoMVelocity(3);
Eigen::MatrixXf pseudoInverseCoMJacobian(30,3);

Eigen::MatrixXf jacobianTorso(3,30);
Eigen::MatrixXf jacobianTorsoTranspose(30,3);
Eigen::MatrixXf jacobianTorsoFull(6,30);
Eigen::MatrixXf jacobianTorsoFullTranspose(30,6);
Eigen::VectorXf taskTorsoError(3);
Eigen::VectorXf taskTorsoPosition(3);
Eigen::VectorXf taskTorsoVelocity(3);
Eigen::MatrixXf pseudoInverseTorsoJacobian(30,6);

Eigen::MatrixXf jacobian(12,30);
Eigen::MatrixXf jacobianTranspose(30,12);
Eigen::VectorXf taskError(12);
Eigen::VectorXf taskPosition(12);
Eigen::VectorXf taskVelocity(12);
Eigen::MatrixXf pseudoInverseJacobian(30,12);
Eigen::VectorXf optimalCoMPosition(3);
Eigen::VectorXf optimalCoMVelocity(3);
Eigen::VectorXf optimalFootstepPosition(3);

Eigen::VectorXf qdot(30);
float swingFootXGain = 150;
float swingFootYGain = 150;
float swingFootZGain = 150;
float swingFootRollGain = 150;
float swingFootPitchGain = 150;
float swingFootYawGain = 150;

float comXGain = 150;
float comYGain = 150;
float comZGain = 150;

float torsoRollGain = 150;
float torsoPitchGain = 150;
float torsoYawGain = 150;

/*float swingFootXGain = 50;
float swingFootYGain = 50;
float swingFootZGain = 50;
float swingFootRollGain = 50;
float swingFootPitchGain = 50;
float swingFootYawGain = 50;

float comXGain = 50;
float comYGain = 50;
float comZGain = 50;

float torsoRollGain = 50;
float torsoPitchGain = 50;
float torsoYawGain = 50;*/

int simulationType = 0;

Eigen::MatrixXf gains(12,12);
int ind=0;
int indInitial=0;
bool dynamicsEnabled=true;
mpcSolver::MPCSolver* solver;
int S=0;
int D=0;
int stepDuration=0;
float stepHeight = 0.035;
float simulationTimeStep =0;
float mpcTimeStep = 0.1;
float controlTimeStep = 0;
float singleSupportDuration = simulationType ? 0.5 : 0.5;
float doubleSupportDuration = 0.4;
float predictionTime = 2*(singleSupportDuration+doubleSupportDuration);
float thetaMax = M_PI/8;
float footContraintSquareWidth = 0.01;//0.08;
float deltaXMax = 0.2;
float deltaYIn = 0.18;
float deltaYOut = 0.25;
float measuredComWeight = 0.15;
float vRefX = 0.15;

float vRefY = 0;
float omegaRef = 0;
float simulationTime=0;
int graphHandleCoMX=0;
int graphHandleCoMY=0;
int boundingBoxHandle=0;
bool stopWalking = true;
float desiredCoMHeight = 0.8;
GLFWwindow* window;
ImVec4 clear_color;
char *stateString;

enum robotStates{ Idle, Evasion, Walk, Stop, Emergency, VisualTracking };
robotStates currentRobotState=Idle;
robotStates previousRobotState=Idle;

bool considerObstacle=false;

int objectHandle=0;

Eigen::VectorXf qToPlayBack(34);
Eigen::Vector3f zmpToPlayback;

std::ofstream timeLog ("/home/nickstu/time.txt");
std::ofstream comTrajectory ("/home/nickstu/comTrajectory.txt");
std::ofstream zmpTrajectory ("/home/nickstu/zmpTrajectory.txt");
std::ofstream leftFootTrajectory ("/home/nickstu/leftFootTrajectory.txt");
std::ofstream rightFootTrajectory ("/home/nickstu/rightFootTrajectory.txt");
std::ofstream supportFootTrajectory ("/home/nickstu/supportFootTrajectory.txt");
std::ofstream vRef_profile ("/home/nickstu/vRef_profile.txt");
std::ofstream pos_profile ("/home/nickstu/pos_profile.txt");
std::ofstream footstepPlan ("/home/nickstu/footstep_plan.txt");

// vRef profile and footstep sequence
int total_samples = simulationType ? 2000 : 1000;
int previewed_footsteps = 2;
int profile_i = 0;
Eigen::VectorXf vRef_profile_sagittal = Eigen::VectorXf::Zero(total_samples);
Eigen::VectorXf vRef_profile_coronal = Eigen::VectorXf::Zero(total_samples);
Eigen::VectorXf vRef_profile_omega = Eigen::VectorXf::Zero(total_samples);
Eigen::VectorXf pos_profile_x = Eigen::VectorXf::Zero(total_samples);
Eigen::VectorXf pos_profile_y = Eigen::VectorXf::Zero(total_samples);
Eigen::VectorXf pos_profile_angle = Eigen::VectorXf::Zero(total_samples);
Eigen::VectorXf footstep_plan_x = Eigen::VectorXf::Zero(previewed_footsteps);
Eigen::VectorXf footstep_plan_y = Eigen::VectorXf::Zero(previewed_footsteps);
Eigen::VectorXf footstep_plan_theta = Eigen::VectorXf::Zero(previewed_footsteps);


static void error_callback(int error, const char* description)
{
    fprintf(stderr, "Error %d: %s\n", error, description);
}

void StaticPlayback(){
	float t = simGetSimulationTime();
	Configuration qCurr;
	if(it != motion.end()){
		if(it->first < t){
			it++;
			if(t <= it->first){
				qCurr = it->second;
				// Disable the dynamic engine
	      //simSetBooleanParameter(sim_boolparam_dynamics_handling_enabled,false);
				robot->setConfigurationStat(qCurr);
				it++;
			}
		}
	}
}

void initializeJoints(){
  joints.push_back(simGetObjectHandle("NECK_Y"));
  joints.push_back(simGetObjectHandle("NECK_P"));
  joints.push_back(simGetObjectHandle("R_SHOULDER_P"));
  joints.push_back(simGetObjectHandle("R_SHOULDER_R"));
  joints.push_back(simGetObjectHandle("R_SHOULDER_Y"));
  joints.push_back(simGetObjectHandle("R_ELBOW_P"));
  joints.push_back(simGetObjectHandle("R_WRIST_Y"));
  joints.push_back(simGetObjectHandle("R_WRIST_P"));
  joints.push_back(simGetObjectHandle("R_WRIST_R"));
  joints.push_back(simGetObjectHandle("L_SHOULDER_P"));
  joints.push_back(simGetObjectHandle("L_SHOULDER_R"));
  joints.push_back(simGetObjectHandle("L_SHOULDER_Y"));
  joints.push_back(simGetObjectHandle("L_ELBOW_P"));
  joints.push_back(simGetObjectHandle("L_WRIST_Y"));
  joints.push_back(simGetObjectHandle("L_WRIST_P"));
  joints.push_back(simGetObjectHandle("L_WRIST_R"));
  joints.push_back(simGetObjectHandle("CHEST_P"));
  joints.push_back(simGetObjectHandle("CHEST_Y"));
  joints.push_back(simGetObjectHandle("R_HIP_Y"));
  joints.push_back(simGetObjectHandle("R_HIP_R"));
  joints.push_back(simGetObjectHandle("R_HIP_P"));
  joints.push_back(simGetObjectHandle("R_KNEE_P"));
  joints.push_back(simGetObjectHandle("R_ANKLE_P"));
  joints.push_back(simGetObjectHandle("R_ANKLE_R"));
  joints.push_back(simGetObjectHandle("L_HIP_Y"));
  joints.push_back(simGetObjectHandle("L_HIP_R"));
  joints.push_back(simGetObjectHandle("L_HIP_P"));
  joints.push_back(simGetObjectHandle("L_KNEE_P"));
  joints.push_back(simGetObjectHandle("L_ANKLE_P"));
  joints.push_back(simGetObjectHandle("L_ANKLE_R"));
}

void setConfiguration(Eigen::VectorXf desiredConfiguration){
  for (int i = 0; i<desiredConfiguration.rows(); i++){
    simSetJointPosition(joints.at(i),desiredConfiguration(i));
  }
}

void setConfigurationKinematics(Eigen::VectorXf desiredConfiguration){
    simFloat posePos[3] = {0,0,0};      // Translation of torso w.r.t. the world frame
    simFloat posePosOrig[3] = {0,0,0};  // Just a simple variable used for computation
    simFloat poseOri[3] = {0,0,0};      // Orientation of support foot w.r.t. the world frame
    simFloat poseTorsoPos[3] = {0,0,0}; // Relative translation of torso w.r.t. support fo
    simFloat poseTorsoOri[3] = {0,0,0}; // Relative orientation of torso w.r.t. support foot
    if (poseShift == NULL) {
        poseShift = new simFloat[6];
        poseShift[0] = 0.0;
        poseShift[1] = 0.0;
        poseShift[2] = 0.0;
        poseShift[3] = 0.0;
        poseShift[4] = 0.0;
        poseShift[5] = 0.0;
    }
    simInt torsoHandle = simGetObjectHandle("body_respondable");
    simInt soleHandle;
    if(supportFoot==RIGHT_SUPPORT){
        soleHandle = simGetObjectHandle("r_ankle_respondable");
    }else{
        soleHandle = simGetObjectHandle("l_ankle_respondable");
    }
    setConfiguration(desiredConfiguration);
    simFloat roto[12]; simFloat R_WSF[12];

    // Get the relative transformation of torso w.r.t. support foot
    simGetObjectPosition(torsoHandle,soleHandle,poseTorsoPos);
    simGetObjectOrientation(torsoHandle,soleHandle,poseTorsoOri);

    // Here poseOri contains the orientation of the support foot expressed in the world frame
    for(int i=0; i<3; ++i){
        poseOri[i] = poseShift[i+3];
    }

    // Build R_SFW
    // We have only the rotation part because posePosOrig is just the Null Vector
    simBuildMatrix(posePosOrig,poseOri,R_WSF);
    // We are transforming the relative translation from Support Foot to Torso expressed in the support foot frame
    // in the world frame
    // R_SFW * p_TSF
    simTransformVector(R_WSF, poseTorsoPos);
    // p_SFW + R_SFW * p_TSF
    for(int i=0; i<3;++i){
        posePos[i] = poseShift[i]+poseTorsoPos[i];
    }
    simGetObjectFloatParameter(soleHandle, 20, poseOri);
    //posePos[2] += poseOri[0];
    // Set the position in absolute frame
    simSetObjectPosition(torsoHandle,-1,posePos);

    // R_WT = R_SFW * R_TSF
    simBuildMatrix(posePosOrig, poseTorsoOri,roto);
    simMultiplyMatrices(R_WSF,roto,R_WSF);
    simGetEulerAnglesFromMatrix(R_WSF,poseTorsoOri);
    simSetObjectOrientation(torsoHandle,-1,poseTorsoOri);
}

void setConfigurationDynamics(Eigen::VectorXf desiredConfiguration){
  for (int i = 0; i<desiredConfiguration.rows(); i++){
    simSetJointTargetPosition(joints.at(i),desiredConfiguration(i));
  }
}

Eigen::VectorXf getCurrentConfiguration(){
  Eigen::VectorXf ret(30);
  simFloat value=0;
  for (int i = 0; i<30; i++){
    simGetJointPosition(joints.at(i),&value);
    ret(i)=value;
  }
  return ret;
}

Eigen::MatrixXf getSwingFootTrajectory(Eigen::VectorXf optimalFootstep, const Eigen::VectorXf& swingFootInitialPosition, int ind, float stepHeight, int S, int D, int indInitial, float delta){

    float m_exp=1;
    float scale = 0.7694;
    float tf = (indInitial + S)*delta;
    float k=1;
    float timeSim=ind*delta;
    float ti=indInitial*delta;

    Eigen::VectorXf swingFootPosition(6);
    Eigen::VectorXf swingFootVelocity(6);
    Eigen::MatrixXf ret(6,2);
    // std::cout<<tf-time<<std::endl;


    if (ind<indInitial){
        //std::cout<<"VEDO SECCI ENTRA"<<std::endl;
        swingFootPosition(0) = swingFootInitialPosition(0);
        swingFootPosition(1) = swingFootInitialPosition(1);
        swingFootPosition(2) = 0*swingFootInitialPosition(2);
        swingFootPosition(3) = 0*swingFootInitialPosition(3);
        swingFootPosition(4) = 0*swingFootInitialPosition(4);
        swingFootPosition(5) = 0*swingFootInitialPosition(5);
        swingFootVelocity(0) = 0;
        swingFootVelocity(1) = 0;
        swingFootVelocity(2) = 0;
        swingFootVelocity(3) = 0;
        swingFootVelocity(4) = 0;
        swingFootVelocity(5) = 0;
    }

    else if (ind>=indInitial+S){
        //            std::cout<<"VEDO SECCI ENTRA"<<std::endl;
        swingFootPosition(0) = optimalFootstep(0);
        swingFootPosition(1) = optimalFootstep(1);
        swingFootPosition(2) = 0;//optimalFootstep(3);
        swingFootPosition(3) = 0*swingFootInitialPosition(3);
        swingFootPosition(4) = 0*swingFootInitialPosition(4);
        swingFootPosition(5) = optimalFootstep(2);
        swingFootVelocity(0) = 0;
        swingFootVelocity(1) = 0;
        swingFootVelocity(2) = 0;
        swingFootVelocity(3) = 0;
        swingFootVelocity(4) = 0;
        swingFootVelocity(5) = 0;
    }

    else{
        swingFootPosition(0) = swingFootInitialPosition(0) + (timeSim-ti)/(tf-ti) * (optimalFootstep(0)-swingFootInitialPosition(0));
        swingFootPosition(1) = swingFootInitialPosition(1) + (timeSim-ti)/(tf-ti) * (optimalFootstep(1)-swingFootInitialPosition(1));
        swingFootPosition(2) = ((sin(M_PI-(timeSim-ti)/(tf-ti)*M_PI)*pow(sin(0.5*(M_PI-(timeSim-ti)/(tf-ti)*M_PI)),m_exp)/scale))*stepHeight;
  //swingFootPosition(0) = optimalFootstep(0);
  //swingFootPosition(1) = optimalFootstep(1);
  //swingFootPosition(2) = optimalFootstep(3);
        swingFootPosition(3) = 0*swingFootInitialPosition(3);
        swingFootPosition(4) = 0*swingFootInitialPosition(4);
        swingFootPosition(5) = swingFootInitialPosition(5) + (timeSim-ti)/(tf-ti) * (optimalFootstep(2)-swingFootInitialPosition(5));

        // std::cout<<swingFootPosition(5)<<std::endl;

        swingFootVelocity(0) = 0;//k*(optimalFootstep(0)-swingFootInitialPosition(0))/(tf-timeSim);
        swingFootVelocity(1) = 0;//k*(optimalFootstep(1)-swingFootInitialPosition(1))/(tf-timeSim);
        swingFootVelocity(2) = (M_PI/(tf-ti))*(stepHeight/scale)*
                (-cos(M_PI-(timeSim-ti)/(tf-ti)*M_PI)*pow(sin(0.5*(M_PI-(timeSim-ti)/(tf-ti)*M_PI)),m_exp)
                -sin(M_PI-(timeSim-ti)/(tf-ti)*M_PI)*0.5*m_exp*pow(sin(0.5*(M_PI-(timeSim-ti)/(tf-ti)*M_PI)),(m_exp-1))*cos(0.5*(M_PI-(timeSim-ti)/(tf-ti)*M_PI)));
  //swingFootVelocity(0) = 0;
  //swingFootVelocity(1) = 0;
  //swingFootVelocity(2) = 0;
        swingFootVelocity(3) = 0;
        swingFootVelocity(4) = 0;
        swingFootVelocity(5) = 0;//k*(optimalFootstep(2)-swingFootInitialPosition(5)/(tf-timeSim));
    }

    ret.col(0)=swingFootPosition;
    ret.col(1)=swingFootVelocity;

    return ret;
}

/*Eigen::MatrixXf getSwingFootTrajectory(Eigen::VectorXf optimalFootstep, const Eigen::VectorXf& swingFootInitialPosition, int ind, float stepHeight, int S, int D, int indInitial, float delta){

    float m_exp=1;
    float scale = 0.7694;
    float tf = (indInitial + S)*delta;
    float k=1;
    float timeSim=ind*delta;
    float ti=indInitial*delta;

    Eigen::VectorXf swingFootPosition(6);
    Eigen::VectorXf swingFootVelocity(6);
    Eigen::MatrixXf ret(6,2);
    // std::cout<<tf-time<<std::endl;


    if (ind<indInitial){
        //std::cout<<"VEDO SECCI ENTRA"<<std::endl;
        swingFootPosition(0) = swingFootInitialPosition(0);
        swingFootPosition(1) = swingFootInitialPosition(1);
        swingFootPosition(2) = 0*swingFootInitialPosition(2);
        swingFootPosition(3) = 0*swingFootInitialPosition(3);
        swingFootPosition(4) = 0*swingFootInitialPosition(4);
        swingFootPosition(5) = 0*swingFootInitialPosition(5);
        swingFootVelocity(0) = 0;
        swingFootVelocity(1) = 0;
        swingFootVelocity(2) = 0;
        swingFootVelocity(3) = 0;
        swingFootVelocity(4) = 0;
        swingFootVelocity(5) = 0;
    }

    else if (ind>=indInitial+S){
        //            std::cout<<"VEDO SECCI ENTRA"<<std::endl;
        swingFootPosition(0) = optimalFootstep(0);
        swingFootPosition(1) = optimalFootstep(1);
        swingFootPosition(2) = 0;//optimalFootstep(3);
        swingFootPosition(3) = 0*swingFootInitialPosition(3);
        swingFootPosition(4) = 0*swingFootInitialPosition(4);
        swingFootPosition(5) = optimalFootstep(2);
        swingFootVelocity(0) = 0;
        swingFootVelocity(1) = 0;
        swingFootVelocity(2) = 0;
        swingFootVelocity(3) = 0;
        swingFootVelocity(4) = 0;
        swingFootVelocity(5) = 0;
    }

    else{
        swingFootPosition(0) = swingFootInitialPosition(0) + (delta)/(tf-timeSim) * k *(optimalFootstep(0)-swingFootInitialPosition(0));
        swingFootPosition(1) = swingFootInitialPosition(1) + (delta)/(tf-timeSim) * k *(optimalFootstep(1)-swingFootInitialPosition(1));
        swingFootPosition(2) = ((sin(M_PI-(timeSim-ti)/(tf-ti)*M_PI)*pow(sin(0.5*(M_PI-(timeSim-ti)/(tf-ti)*M_PI)),m_exp)/scale))*stepHeight;
  //swingFootPosition(0) = optimalFootstep(0);
  //swingFootPosition(1) = optimalFootstep(1);
  //swingFootPosition(2) = optimalFootstep(3);
        swingFootPosition(3) = 0*swingFootInitialPosition(3);
        swingFootPosition(4) = 0*swingFootInitialPosition(4);
        swingFootPosition(5) = swingFootInitialPosition(5) + (delta)/(tf-timeSim) * k*(optimalFootstep(2)-swingFootInitialPosition(5));

        // std::cout<<swingFootPosition(5)<<std::endl;

        swingFootVelocity(0) = 0;//k*(optimalFootstep(0)-swingFootInitialPosition(0))/(tf-timeSim);
        swingFootVelocity(1) = 0;//k*(optimalFootstep(1)-swingFootInitialPosition(1))/(tf-timeSim);
        swingFootVelocity(2) = (M_PI/(tf-ti))*(stepHeight/scale)*
                (-cos(M_PI-(timeSim-ti)/(tf-ti)*M_PI)*pow(sin(0.5*(M_PI-(timeSim-ti)/(tf-ti)*M_PI)),m_exp)
                -sin(M_PI-(timeSim-ti)/(tf-ti)*M_PI)*0.5*m_exp*pow(sin(0.5*(M_PI-(timeSim-ti)/(tf-ti)*M_PI)),(m_exp-1))*cos(0.5*(M_PI-(timeSim-ti)/(tf-ti)*M_PI)));
  //swingFootVelocity(0) = 0;
  //swingFootVelocity(1) = 0;
  //swingFootVelocity(2) = 0;
        swingFootVelocity(3) = 0;
        swingFootVelocity(4) = 0;
        swingFootVelocity(5) = 0;//k*(optimalFootstep(2)-swingFootInitialPosition(5)/(tf-timeSim));
    }

    ret.col(0)=swingFootPosition;
    ret.col(1)=swingFootVelocity;

    return ret;


}*/

Eigen::MatrixXf computePseudoinverse(Eigen::MatrixXf matrixToBePseudoinverted, float threshold = 1E-6){
       int rows = (int)(matrixToBePseudoinverted.rows());
       return matrixToBePseudoinverted.jacobiSvd(Eigen::ComputeThinU|Eigen::ComputeThinV).setThreshold(threshold).solve(Eigen::MatrixXf::Identity(rows,rows));
   }
////////////////////////////////////////////////////////////////////////////////

float angleSignedDistance(float a, float b){
	//float d = fabs(a - b) % 2.0*M_PI;
	float d = fabs(a - b);
	while(d > 2.0*M_PI) d = d - 2.0*M_PI;

	float r = 0.0;
	if(d > M_PI) r = 2.0*M_PI - d;
	else r = d;
	float sign = 0.0;
	if( (a-b>=0.0 && a-b<=M_PI) || (a-b<=-M_PI && a-b>=-2.0*M_PI) ) sign = +1.0;
	else sign = -1.0;

	r = sign * r;
	return r;
}

void generateTask(){

  jacobianFoot = (supportFoot==LEFT_SUPPORT) ? kin->getJacobian(currentConfiguration,L_FOOT,R_FOOT) : kin->getJacobian(currentConfiguration,R_FOOT,L_FOOT);
  jacobianFootTranspose = jacobianFoot.transpose();

  jacobianCoM = (supportFoot==LEFT_SUPPORT) ? kin->getJacobian(currentConfiguration,L_FOOT,COM) : kin->getJacobian(currentConfiguration,R_FOOT,COM);;
  jacobianCoMTranspose = jacobianCoM.transpose();

  jacobianTorsoFull = (supportFoot==LEFT_SUPPORT) ? kin->getJacobian(currentConfiguration,L_FOOT,TORSO) : kin->getJacobian(currentConfiguration,R_FOOT,TORSO);;

  jacobianTorso.setZero();
  jacobianTorso.row(0) = jacobianTorsoFull.row(3);
  jacobianTorso.row(1) = jacobianTorsoFull.row(4);

  jacobianTorsoTranspose.setZero();
  jacobianTorsoTranspose=jacobianTorso.transpose();

  currentFootPosition = (supportFoot==LEFT_SUPPORT) ? kin->forwardKinematicsWrtSupportFoot(currentConfiguration, R_FOOT, supportFoot) : kin->forwardKinematicsWrtSupportFoot(currentConfiguration, L_FOOT, supportFoot);
  currentCoMPosition = (kin->forwardKinematicsWrtSupportFoot(currentConfiguration, COM, supportFoot)).segment(0,3);
  currentTorsoPosition = (kin->forwardKinematicsWrtSupportFoot(currentConfiguration, TORSO, supportFoot));

  currentPosition <<currentCoMPosition, currentFootPosition, currentTorsoPosition(3), currentTorsoPosition(4);

  taskFootPosition=(supportFoot==LEFT_SUPPORT) ? initialFootPositionLeftSupport : initialFootPositionRightSupport;
  taskFootPosition(2)=0.05;
  //taskFootPosition(4)=0.5*sin(simGetSimulationTime());

  taskCoMPosition=(supportFoot==LEFT_SUPPORT) ? initialCoMPositionLeftSupport : initialCoMPositionRightSupport;
  //taskCoMPosition=(supportFoot==LEFT_SUPPORT) ? initialCoMPositionLeftSupport : initialCoMPositionRightSupport;
  taskCoMPosition(1)=0;


  taskTorsoPosition.setZero();
  taskTorsoPosition(0)=(supportFoot==LEFT_SUPPORT) ? initialTorsoPositionLeftSupport(3) : initialTorsoPositionRightSupport(3);
  taskTorsoPosition(1)=(supportFoot==LEFT_SUPPORT) ? initialTorsoPositionLeftSupport(4) : initialTorsoPositionRightSupport(4);

  //taskTorsoPosition(0)=0.5*sin(simGetSimulationTime());
  // std::cout<<"desired position: "<<taskTorsoPosition.transpose()<<std::endl;
  // std::cout<<"current position: "<<currentTorsoPosition(3) << " "<< currentTorsoPosition(4) <<std::endl;


  //taskTorsoPosition(0)=M_PI/2;
  taskVelocity.setZero();


  jacobian << jacobianCoM, jacobianFoot, jacobianTorso;
  jacobianTranspose << jacobianCoMTranspose,jacobianFootTranspose, jacobianTorsoTranspose;
  taskPosition << taskCoMPosition, taskFootPosition, taskTorsoPosition;

  //pseudoInverseFootJacobian=jacobianLeftFoot2RightFootTranspose*(jacobianLeftFoot2RightFoot*jacobianLeftFoot2RightFootTranspose).inverse();
  pseudoInverseJacobian=jacobianTranspose*(jacobian*jacobianTranspose).inverse();
  taskError = taskPosition-currentPosition;
  taskError(9) =angleSignedDistance(taskPosition(9),currentPosition(9));
  taskError(10) =angleSignedDistance(taskPosition(10),currentPosition(10));

  std::cout<<"torso orientation: "<<currentTorsoPosition.transpose()<<std::endl;
  std::cout<<"comOrientation: "<<currentConfiguration.getqCoMOrientation().transpose()<<std::endl;

}

void generateMPCTask(){

  solver->setCoMTargetHeight(desiredCoMHeight);

  if(solver->getFootstepCounter()==0 && solver->getControlIteration()==0){
    S=solver->getS();
    D=solver->getD();
    stepDuration=S+D;

    initialFootPosition = (supportFoot==LEFT_SUPPORT) ? kin->forwardKinematicsWrtSupportFoot(currentConfiguration, R_FOOT, supportFoot) : kin->forwardKinematicsWrtSupportFoot(currentConfiguration, L_FOOT, supportFoot);
  }

  if(solver->supportFootHasChanged()){

    supportFoot=!supportFoot;
    indInitial=ind;

    S=solver->getS();//round(singleSupportDuration/simulationTimeStep);
    D=solver->getD();//round(doubleSupportDuration/simulationTimeStep);
    stepDuration=S+D;

    initialFootPosition = (supportFoot==LEFT_SUPPORT) ? kin->forwardKinematicsWrtSupportFoot(currentConfiguration, R_FOOT, supportFoot) : kin->forwardKinematicsWrtSupportFoot(currentConfiguration, L_FOOT, supportFoot);
  }

  if(solver->supportFootHasChanged() || (solver->getFootstepCounter()==0 && solver->getControlIteration()==0)){

    pos_profile_x.setZero();
    pos_profile_y.setZero();
    pos_profile_angle.setZero();

    //pos_profile_y(0) = -(supportFoot ? 1 : -1)*0.18/2;

    // Integrate reference velocities to generate footstep sequence
    for(int i=1; i<(previewed_footsteps+1)*(S+D); ++i){

      if (i<=S+D & profile_i==0) {
	pos_profile_angle(i) = pos_profile_angle(i-1);
      	pos_profile_x(i) = pos_profile_x(i-1);
      	pos_profile_y(i) = pos_profile_y(i-1);
      } else {

        pos_profile_angle(i) = pos_profile_angle(i-1) + vRef_profile_omega(profile_i+i-1)*mpcTimeStep;
        pos_profile_x(i) = pos_profile_x(i-1) + cos(pos_profile_angle(i))*vRef_profile_sagittal(profile_i+i-1)*mpcTimeStep
		         + sin(pos_profile_angle(i))*vRef_profile_coronal(profile_i+i-1)*mpcTimeStep;
        pos_profile_y(i) = pos_profile_y(i-1) - sin(pos_profile_angle(i))*vRef_profile_sagittal(profile_i+i-1)*mpcTimeStep
		         + cos(pos_profile_angle(i))*vRef_profile_coronal(profile_i+i-1)*mpcTimeStep;
      }
    }

    int sign = 0;//supportFoot;// ? 1 : -1;
    float displacement = (supportFoot ? 1 : -1)*0.18/2;

    for(int i=0; i<previewed_footsteps; ++i){
      footstep_plan_x(i) = pos_profile_x((S+D)*(i+1)) + sin(pos_profile_angle((S+D)*(i+1)))*displacement;
      footstep_plan_y(i) = pos_profile_y((S+D)*(i+1)) + cos(pos_profile_angle((S+D)*(i+1)))*displacement - (supportFoot ? -1 : 1)*0.18/2;
      footstep_plan_theta(i) = pos_profile_angle((S+D)*(i+1));

      displacement = -displacement;
      //sign = !sign;
    }

    std::cout << "x " << footstep_plan_x.transpose() << std::endl;
    std::cout << "y " << footstep_plan_y.transpose() << std::endl;

    profile_i += S+D;
  }

  jacobianFoot = (supportFoot==LEFT_SUPPORT) ? kin->getJacobian(currentConfiguration,L_FOOT,R_FOOT) : kin->getJacobian(currentConfiguration,R_FOOT,L_FOOT);
  jacobianFootTranspose = jacobianFoot.transpose();

  jacobianCoM = (supportFoot==LEFT_SUPPORT) ? kin->getJacobian(currentConfiguration,L_FOOT,COM) : kin->getJacobian(currentConfiguration,R_FOOT,COM);;
  jacobianCoMTranspose = jacobianCoM.transpose();

  jacobianTorsoFull = (supportFoot==LEFT_SUPPORT) ? kin->getJacobian(currentConfiguration,L_FOOT,TORSO) : kin->getJacobian(currentConfiguration,R_FOOT,TORSO);;
  jacobianTorsoFullTranspose = jacobianTorsoFull.transpose();

  jacobianTorso = jacobianTorsoFull.block(3,0,3,30);
  jacobianTorsoTranspose=jacobianTorso.transpose();

  currentFootPosition = (supportFoot==LEFT_SUPPORT) ? kin->forwardKinematicsWrtSupportFoot(currentConfiguration, R_FOOT, supportFoot) : kin->forwardKinematicsWrtSupportFoot(currentConfiguration, L_FOOT, supportFoot);
  currentCoMPosition = (kin->forwardKinematicsWrtSupportFoot(currentConfiguration, COM, supportFoot)).segment(0,3);
  currentTorsoPosition = (kin->forwardKinematicsWrtSupportFoot(currentConfiguration, TORSO, supportFoot)).segment(3,3);

  currentPosition <<currentCoMPosition, currentFootPosition, currentTorsoPosition;

  taskFootPosition=(supportFoot==LEFT_SUPPORT) ? initialFootPositionLeftSupport : initialFootPositionRightSupport;
  taskCoMPosition=(supportFoot==LEFT_SUPPORT) ? initialCoMPositionLeftSupport : initialCoMPositionRightSupport;
  taskTorsoPosition=(supportFoot==LEFT_SUPPORT) ? initialTorsoPositionLeftSupport.segment(3,3) : initialTorsoPositionRightSupport.segment(3,3);
  //taskCoMPosition(1)=0.05 + 0.05*sin(simGetSimulationTime());
  Eigen::Vector3f measuredZmpPos;
  measuredZmpPos.setZero();

  Eigen::Affine3f swingFootTransform;
  swingFootTransform.matrix() = v2t(currentFootPosition);
  simulationTime=simGetSimulationTime();

  //0 is straight walk, varying velocity
  //1 is cusp
  
  double simulationStart = 0;
  double simulationEnd;
  if (simulationType == 0) simulationEnd = 1+8.4;
  else simulationEnd = 19;

  if (simulationType == 0) {
    vRefX = 0;

    if (simGetSimulationTime()>simulationStart) {
      stopWalking=false;
      stateString="Walk";
      vRefX = 0.1;
    }

    //if (simGetSimulationTime()>simulationStart) vRefX = 0.15;

    if (simGetSimulationTime()>1+4.1) vRefX = 0.15;
  }
/*
  if (simulationType == 1) {
    vRefX = 0;

    if (simGetSimulationTime()>simulationStart) {
      stopWalking=false;
      stateString="Walk";
      vRefX = 0.15;
      omegaRef = 0.2;
    }

    if (simGetSimulationTime()>9) {
      vRefX = -0.15;
      omegaRef = 0.2;
    }

    if (simGetSimulationTime()>18) {
      vRefX = -0.15;
      omegaRef = 0;
    }
  }*/

  vRefX = vRef_profile_sagittal(profile_i);
  vRefY = 0;
  omegaRef = 0;

  solver->solve(currentCoMPosition, measuredZmpPos, swingFootTransform, supportFoot, simulationTime, vRefX, vRefY, omegaRef, stopWalking, footstep_plan_x, footstep_plan_y, footstep_plan_theta);

  //std::cout << solver->getOptimalFootsteps().transpose() << std::endl;

  optimalCoMPosition = solver->getOptimalCoMPosition();
  optimalCoMVelocity = solver->getOptimalCoMVelocity();
  optimalFootstepPosition = solver->getOptimalFootsteps();

  std::cout << optimalCoMPosition.transpose() << std::endl;

  taskCoMPosition(0)=optimalCoMPosition(0)+0.0025;
  taskCoMPosition(1)=optimalCoMPosition(1);
  taskCoMPosition(2)=optimalCoMPosition(2);

  taskCoMVelocity.setZero();
  taskCoMVelocity(0) = optimalCoMVelocity(0);
  taskCoMVelocity(1) = optimalCoMVelocity(1);
  taskCoMVelocity(2) = optimalCoMVelocity(2);

  swingFootTask = getSwingFootTrajectory(optimalFootstepPosition, initialFootPosition, ind, stepHeight, round(S*mpcTimeStep/controlTimeStep), round(D*mpcTimeStep/controlTimeStep), indInitial, simGetSimulationTimeStep());

  taskFootPosition = swingFootTask.col(0);
  taskFootVelocity = swingFootTask.col(1);
  taskTorsoVelocity.setZero();

  /*if(solver->isRealEmergency()){
    taskFootPosition(2)=0;
    taskFootVelocity(2)=0;
  }*/

  simSetGraphUserData(graphHandleCoMX,"ActualCoMX",currentCoMPosition(0));
  simSetGraphUserData(graphHandleCoMX,"DesiredCoMX",taskCoMPosition(0));

  simSetGraphUserData(graphHandleCoMY,"ActualCoMY",currentCoMPosition(1));
  simSetGraphUserData(graphHandleCoMY,"DesiredCoMY",taskCoMPosition(1));

  jacobian << jacobianCoM, jacobianFoot, jacobianTorso;
  jacobian.col(2).setZero();
  jacobian.col(9).setZero();

  jacobianTranspose << jacobianCoMTranspose,jacobianFootTranspose, jacobianTorsoTranspose;
  taskPosition << taskCoMPosition, taskFootPosition, taskTorsoPosition;
  taskVelocity << taskCoMVelocity, taskFootVelocity, taskTorsoVelocity;
  //pseudoInverseFootJacobian=jacobianLeftFoot2RightFootTranspose*(jacobianLeftFoot2RightFoot*jacobianLeftFoot2RightFootTranspose).inverse();
  //pseudoInverseJacobian=jacobianTranspose*(jacobian*jacobianTranspose).inverse();
  jacobian.col(16).setZero();
  jacobian.col(17).setZero();
  pseudoInverseJacobian=computePseudoinverse(jacobian);
  taskError = taskPosition-currentPosition;

  taskError(9) = angleSignedDistance(taskTorsoPosition(0),currentTorsoPosition(0));
  taskError(10) = angleSignedDistance(taskTorsoPosition(1),currentTorsoPosition(1));
  taskError(11) = angleSignedDistance(taskTorsoPosition(2),currentTorsoPosition(2));

  simFloat positionSupportFoot[3] = {0,0,0};
  simFloat orientationSupportFoot[3] = {0,0,0};
  initialPoseShift = new simFloat[6];
  simInt soleHandle;
  if(supportFoot==RIGHT_SUPPORT){
      soleHandle = simGetObjectHandle("r_ankle_respondable");
  }else{
      soleHandle = simGetObjectHandle("l_ankle_respondable");
  }
  simGetObjectPosition(soleHandle,-1, positionSupportFoot);
  simGetObjectOrientation(soleHandle,-1, orientationSupportFoot);

  initialPoseShift[0] = positionSupportFoot[0];
  initialPoseShift[1] = positionSupportFoot[1];
  initialPoseShift[2] = 0.02;
  initialPoseShift[3] = orientationSupportFoot[0];
  initialPoseShift[4] = orientationSupportFoot[1];
  initialPoseShift[5] = orientationSupportFoot[2];
  //std::cout<<"CoM position is: "<<currentCoMPosition.transpose()<<" and support foot is: "<<supportFoot <<std::endl;;

  Eigen::VectorXd supportFootAbsPose = Eigen::VectorXd(7);
  supportFootAbsPose << positionSupportFoot[0], positionSupportFoot[1], positionSupportFoot[2], orientationSupportFoot[0], orientationSupportFoot[1], orientationSupportFoot[2], supportFoot;

  simFloat leftFootAbsPositionVrep[3] = {0,0,0};
  simGetObjectPosition(simGetObjectHandle("l_ankle_respondable"),-1, leftFootAbsPositionVrep);
  simFloat leftFootAbsOrientationVrep[3] = {0,0,0};
  simGetObjectOrientation(simGetObjectHandle("l_ankle_respondable"),-1, leftFootAbsOrientationVrep);
  Eigen::VectorXd leftFootAbsPose = Eigen::VectorXd(6);
  leftFootAbsPose << leftFootAbsPositionVrep[0], leftFootAbsPositionVrep[1], leftFootAbsPositionVrep[2], leftFootAbsOrientationVrep[0], leftFootAbsOrientationVrep[1], leftFootAbsOrientationVrep[2];

  simFloat rightFootAbsPositionVrep[3] = {0,0,0};
  simGetObjectPosition(simGetObjectHandle("r_ankle_respondable"),-1, rightFootAbsPositionVrep);
  simFloat rightFootAbsOrientationVrep[3] = {0,0,0};
  simGetObjectOrientation(simGetObjectHandle("r_ankle_respondable"),-1, rightFootAbsOrientationVrep);
  Eigen::VectorXd rightFootAbsPose = Eigen::VectorXd(6);
  rightFootAbsPose << rightFootAbsPositionVrep[0], rightFootAbsPositionVrep[1], rightFootAbsPositionVrep[2], rightFootAbsOrientationVrep[0], rightFootAbsOrientationVrep[1], rightFootAbsOrientationVrep[2];

  if (simGetSimulationTime()>simulationStart && simGetSimulationTime()<=simulationEnd) {
	  timeLog << simGetSimulationTime()-simulationStart << std::endl;
	  comTrajectory << currentCoMPosition.transpose() << std::endl;
	  zmpTrajectory << solver->getOptimalZMPPosition().transpose() << std::endl;
	  leftFootTrajectory << leftFootAbsPose.transpose() << std::endl;
	  rightFootTrajectory << rightFootAbsPose.transpose() << std::endl;
	  supportFootTrajectory << supportFootAbsPose.transpose() << std::endl;
  }

  if (simGetSimulationTime()>simulationEnd) std::cout << "Simulation Ended" << std::endl;

}

void guiLoop(){
  glfwPollEvents();
  ImGui_ImplGlfwGL3_NewFrame();
  // 1. Show a simple window.
  // Tip: if we don't call ImGui::Begin()/ImGui::End() the widgets appears in a window automatically called "Debug".
  ImGui::Begin("Gait Parameters", nullptr, ImVec2(400,400));
  //ImGui::SliderFloat("vRefX", &vRefX, -0.25f, 0.25f, "%.2f");
  //ImGui::SliderFloat("vRefY", &vRefY, -0.1f, 0.1f, "%.2f");
  //ImGui::SliderFloat("omegaRef", &omegaRef, -0.5f, 0.5f, "%.2f");
  ImGui::SliderFloat("CoM Height", &desiredCoMHeight, 0.67f, 0.85f, "%.2f");

  if (ImGui::Button("Idle")){
      currentRobotState=Idle;
      stateString="Idle";
  }
  if (ImGui::Button("Evasion")){
      currentRobotState=Evasion;
      stateString="Evasion";
  }
  if (ImGui::Button("Walk")){
      currentRobotState=Walk;
      stopWalking=false;
      stateString="Walk";
  }
  if (ImGui::Button("Stop")){
      currentRobotState=Stop;
      stopWalking=true;
      stateString="Stop";
  }
  if (ImGui::Button("Emergency")){
      currentRobotState=Emergency;
      stateString="Emergency";
  }
  if (ImGui::Button("Visual Tracking")){
      currentRobotState=VisualTracking;
      stateString="Visual Tracking";
  }

  if(ImGui::Button("Consider Obstacle")){
      considerObstacle=!considerObstacle;
  }

  switch(currentRobotState){
    case Idle:
      stateString="Idle";
    break;

    case Evasion:
      stateString="Evasion";
    break;

    case Walk:
      stateString="Walk";
    break;

    case Stop:
      stateString="Stop";
    break;

    case Emergency:
      stateString="Emergency";
    break;

    case VisualTracking:
      stateString="Visual Tracking";
    break;

  }

  ImGui::Text("Current robot state: %s", stateString);
  ImGui::End();

   // Rendering
  int display_w, display_h;
  glfwGetFramebufferSize(window, &display_w, &display_h);
  glViewport(0, 0, display_w, display_h);
  glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
  glClear(GL_COLOR_BUFFER_BIT);
  ImGui::Render();
  glfwSwapBuffers(window);
  ////
}

int sign(float number){
  if (number>0)
    return 1;
  else if(number<0)
    return -1;
  return 0;
}

Eigen::VectorXf computeJointVelocitiesQP(){
	int nVariables = 30;

	qpOASES::real_t xOpt[nVariables];

	qpOASES::Options options;
	options.setToMPC();
	options.printLevel=qpOASES::PL_NONE;
	qpOASES::int_t nWSR = 300;
	qpOASES::QProblemB qp = qpOASES::QProblemB(nVariables);
	qp.setOptions(options);

	Eigen::MatrixXf costFunctionH = simulationTimeStep*simulationTimeStep*jacobianTranspose*jacobian + 1E-7*Eigen::MatrixXf::Identity(nVariables,nVariables);

	Eigen::VectorXf costFunctionF = simulationTimeStep*jacobianTranspose*(-taskError);

        qpOASES::real_t H[nVariables*nVariables];
	qpOASES::real_t g[nVariables];

	for(int i=0;i<nVariables;++i){
		for(int j=0;j<nVariables;++j){
			H[i*nVariables+j] = costFunctionH(i,j);
		}
		g[i] = costFunctionF(i);
	}

	qp.init(H,g,0,0,nWSR,NULL);

	qp.getPrimalSolution(xOpt);

	Eigen::VectorXf decisionVariables(nVariables);
	for(int i=0;i<nVariables;++i){
		decisionVariables(i) = xOpt[i];
	}
        std::cout<<decisionVariables.transpose()<<std::endl;
	return decisionVariables;
}

Eigen::VectorXf computeJointVelocitiesKI(){
  return pseudoInverseJacobian*(taskVelocity + (gains*taskError));
}

void mainLoop(){

  // simFloat* objectPosition=new simFloat[3];
  // simGetObjectPosition(objectHandle,boundingBoxHandle, objectPosition);

  float distance=0;//sqrt(pow(objectPosition[0],2)+pow(objectPosition[1],2));
  float angle=0;//atan2(objectPosition[1],objectPosition[0]);

  actualConfiguration=getCurrentConfiguration();
  currentConfiguration.setqjnt(actualConfiguration);

  generateMPCTask();

  simFloat* pos = new simFloat[3];
  simFloat* ori = new simFloat[3];
  poseShift = new simFloat[6];

  simInt soleHandle;
  if(supportFoot==RIGHT_SUPPORT){
      soleHandle = simGetObjectHandle("r_ankle_respondable");
  }else{
      soleHandle = simGetObjectHandle("l_ankle_respondable");
  }
  simGetObjectPosition(soleHandle,-1,pos);
  simGetObjectOrientation(soleHandle,-1,ori);

  poseShift[0] = pos[0];
  poseShift[1] = pos[1];
  poseShift[2] = 0.02;
  poseShift[3] = ori[0];
  poseShift[4] = ori[1];
  poseShift[5] = ori[2];

  if(considerObstacle){
    if(distance<7 && distance>=5){
      previousRobotState=currentRobotState;
      currentRobotState=VisualTracking;
    }
    else if(distance>=2 && distance<5){
      currentRobotState=Evasion;
      vRefX=-0.15;
      if (angle>0) {
        omegaRef=0.5*(angle-(M_PI/2));
      }
      else{
        omegaRef=0.5*(angle+(M_PI/2));
      }
      stopWalking=false;
    }
    else if(distance<2){
      currentRobotState=Emergency;
      omegaRef=0;
      vRefX=0;
      stopWalking=true;
    }
    else{
      currentRobotState=Idle;
      vRefX=0;
      omegaRef=0;
      stopWalking=true;
    }
  }

  qdot = computeJointVelocitiesKI();
  // // qdot = computeJointVelocitiesQP();
  // if(currentRobotState==Idle){
  //   qdot(0) = cos(simulationTime);
  // }
  // else if(currentRobotState==VisualTracking){
  //   qdot(0) = 1.5*(angle-actualConfiguration(0)); // Tracking of the object in the safe tracking zone
  // }
  // else{
  //   qdot(0)=(-actualConfiguration(0));
  // }

  Eigen::VectorXf q = actualConfiguration + qdot*simGetSimulationTimeStep();
  q(2)=3*q(26)/4 - 3*q(20)/4;
  q(9)=3*q(20)/4 - 3*q(26)/4;

  /*float balanceGain = 0.2;
  simFloat torsoOrientation[3] = {0,0,0};
  int torsoHandle = simGetObjectHandle("torso_respondable");//("waist");
  simGetObjectOrientation(torsoHandle,-1,torsoOrientation);
  std::cout << torsoOrientation[0] << " " << torsoOrientation[1] << " " << torsoOrientation[2] << std::endl;*/
  //q(23) -= balanceGain*torsoOrientation[2];
  //q(29) -= balanceGain*torsoOrientation[2];
  //q(22) += balanceGain*torsoOrientation[1];
  //q(28) += balanceGain*torsoOrientation[1];

  if(!dynamicsEnabled)
    setConfigurationKinematics(q);
  else
    setConfigurationDynamics(q);
  ++ind;

  //GUI LOOP
  // You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell if dear imgui wants to use your inputs.
  // - When io.WantCaptureMouse is true, do not dispatch mouse input data to your main application.
  // - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data to your main application.
  // Generally you may always pass all inputs to dear imgui, and hide them from your application based on those two flags.
  guiLoop();

}

void initializeSimulation(){
  simSetBooleanParameter(sim_boolparam_dynamics_handling_enabled,dynamicsEnabled);
  std::cout<<"Initializing Simulation"<<std::endl;
  robot = new Hrp4();
  kin = new Hrp4Kinematics();
  initializeJoints();
  graphHandleCoMX = simGetObjectHandle("CoMDesiredTrajectoryX");
  graphHandleCoMY = simGetObjectHandle("CoMDesiredTrajectoryY");
  boundingBoxHandle = simGetObjectHandle("torso_respondable");
  currentConfiguration = robot->getCurrentConfiguration();

  initialFootPositionLeftSupport = kin->forwardKinematicsWrtSupportFoot(currentConfiguration, R_FOOT, LEFT_SUPPORT);
  initialCoMPositionLeftSupport = (kin->forwardKinematicsWrtSupportFoot(currentConfiguration, COM, LEFT_SUPPORT)).segment(0,3);
  initialTorsoPositionLeftSupport = kin->forwardKinematicsWrtSupportFoot(currentConfiguration, TORSO, LEFT_SUPPORT);

  initialFootPositionRightSupport = kin->forwardKinematicsWrtSupportFoot(currentConfiguration, L_FOOT, RIGHT_SUPPORT);
  initialCoMPositionRightSupport = (kin->forwardKinematicsWrtSupportFoot(currentConfiguration, COM, RIGHT_SUPPORT)).segment(0,3);
  initialTorsoPositionRightSupport = kin->forwardKinematicsWrtSupportFoot(currentConfiguration, TORSO, RIGHT_SUPPORT);

  initialCoMPositionLeftSupport(2) = desiredCoMHeight;
  initialCoMPositionRightSupport(2) = desiredCoMHeight;

  simFloat positionSupportFoot[3] = {0,0,0};
  simFloat orientationSupportFoot[3] = {0,0,0};
  initialPoseShift = new simFloat[6];
  simInt soleHandle;
  if(supportFoot==RIGHT_SUPPORT){
      soleHandle = simGetObjectHandle("r_ankle_respondable");
  }else{
      soleHandle = simGetObjectHandle("l_ankle_respondable");
  }
  simGetObjectPosition(soleHandle,-1, positionSupportFoot);
  simGetObjectOrientation(soleHandle,-1, orientationSupportFoot);

  initialPoseShift[0] = positionSupportFoot[0];
  initialPoseShift[1] = positionSupportFoot[1];
  initialPoseShift[2] = 0.02;
  initialPoseShift[3] = orientationSupportFoot[0];
  initialPoseShift[4] = orientationSupportFoot[1];
  initialPoseShift[5] = orientationSupportFoot[2];

  taskPosition.setZero();

  simulationTimeStep=simGetSimulationTimeStep();
  controlTimeStep=simulationTimeStep;
  S=round(singleSupportDuration/simulationTimeStep);
  D=round(doubleSupportDuration/simulationTimeStep);
  stepDuration=round((singleSupportDuration+doubleSupportDuration)/controlTimeStep);
  indInitial=stepDuration;

  // Write velocity profile

  if (simulationType==0) {
  
    for(int i=0; i<total_samples; ++i){
      vRef_profile_sagittal(i) = 0.1;
    }

    for(int i=60; i<total_samples; ++i){
      vRef_profile_sagittal(i) = 0.2;
    }
  }

  if (simulationType==1) {
    for(int i=0; i<total_samples; ++i){
      vRef_profile_sagittal(i) = 0.15;
      vRef_profile_omega(i) = 0.2;
    }

    for(int i=85; i<total_samples; ++i){
      vRef_profile_sagittal(i) = -0.15;
    }

    for(int i=162; i<total_samples; ++i){
      vRef_profile_omega(i) = 0;
    }
  }
  
/*
  vRef_profile << vRef_profile_sagittal.transpose() << std::endl;
  vRef_profile << vRef_profile_coronal.transpose() << std::endl;
  vRef_profile << vRef_profile_omega.transpose() << std::endl;
  pos_profile << pos_profile_x.transpose() << std::endl;
  pos_profile << pos_profile_y.transpose() << std::endl;
  pos_profile << pos_profile_angle.transpose() << std::endl;
  footstepPlan << footstep_plan_x.transpose() << std::endl;
  footstepPlan << footstep_plan_y.transpose() << std::endl;
  footstepPlan << footstep_plan_theta.transpose() << std::endl;*/

  solver = new mpcSolver::MPCSolver(mpcTimeStep, controlTimeStep, predictionTime, initialCoMPositionLeftSupport,
                       singleSupportDuration, doubleSupportDuration, thetaMax,
  					 footContraintSquareWidth, deltaXMax, deltaYIn, deltaYOut, measuredComWeight);

  gains=Eigen::MatrixXf::Identity(12,12);

  gains(0,0) = comXGain;
  gains(1,1) = comYGain;
  gains(2,2) = comZGain;
  gains(3,3) = swingFootXGain;
  gains(4,4) = swingFootYGain;
  gains(5,5) = swingFootZGain;
  gains(6,6) = swingFootRollGain;
  gains(7,7) = swingFootPitchGain;
  gains(8,8) = swingFootYawGain;
  gains(9,9) = torsoRollGain;
  gains(10,10) = torsoPitchGain;
  gains(11,11) = torsoYawGain;

  //GUI

  glfwSetErrorCallback(error_callback);
  if (!glfwInit())
      std::cout<<"Errore"<<std::endl;
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  window = glfwCreateWindow(400, 400, "MPC Walking Pattern Generator", NULL, NULL);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1); // Enable vsync
  gl3wInit();

  // Setup ImGui binding
  ImGui_ImplGlfwGL3_Init(window, true);
  clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

  std::cout<<"GUI initialization completed"<<std::endl;

  objectHandle=simGetObjectHandle("Cuboid");

  qToPlayBack.setZero();
  zmpToPlayback.setZero();

}

void resetSimulation(){
  supportFoot=LEFT_SUPPORT;
  ind = 0;
  indInitial=0;
  vRefX=vRefY=omegaRef=0;
  stopWalking=true;
  currentRobotState=Idle;
}
// This is the plugin start routine (called just once, just after the plugin was loaded):

VREP_DLLEXPORT unsigned char v_repStart(void* reservedPointer, int reservedInt) {
    // Dynamically load and bind V-REP functions:
    // ******************************************
    // 1. Figure out this plugin's directory:
    char curDirAndFile[1024];
    #ifdef _WIN32
        GetModuleFileName(NULL, curDirAndFile, 1023);
        PathRemoveFileSpec(curDirAndFile);
    #elif defined (__linux) || defined (__APPLE__)
        getcwd(curDirAndFile, sizeof (curDirAndFile));
    #endif
        std::string currentDirAndPath(curDirAndFile);
        // 2. Append the V-REP library's name:
        std::string temp(currentDirAndPath);
    #ifdef _WIN32
        temp += "\\v_rep.dll";
    #elif defined (__linux)
        temp += "/libv_rep.so";
    #elif defined (__APPLE__)
        temp += "/libv_rep.dylib";
    #endif /* __linux || __APPLE__ */
        // 3. Load the V-REP library:
        vrepLib = loadVrepLibrary(temp.c_str());
        if (vrepLib == NULL) {
            std::cout << "Error, could not find or correctly load the V-REP library. Cannot start 'PluginSkeleton' plugin.\n";
            return (0); // Means error, V-REP will unload this plugin
        }
        if (getVrepProcAddresses(vrepLib) == 0) {
            std::cout << "Error, could not find all required functions in the V-REP library. Cannot start 'PluginSkeleton' plugin.\n";
            unloadVrepLibrary(vrepLib);
            return (0); // Means error, V-REP will unload this plugin
        }
        // ******************************************

        // Check the version of V-REP:
        // ******************************************
        int vrepVer;
        simGetIntegerParameter(sim_intparam_program_version, &vrepVer);
        if (vrepVer < 20604) // if V-REP version is smaller than 2.06.04
        {
            std::cout << "Sorry, your V-REP copy is somewhat old. Cannot start 'PluginSkeleton' plugin.\n";
            unloadVrepLibrary(vrepLib);
            return (0); // Means error, V-REP will unload this plugin
        }
        // ******************************************

        simLockInterface(1);

        // Here you could handle various initializations
        // Here you could also register custom Lua functions or custom Lua constants
        // etc.

        return (PLUGIN_VERSION); // initialization went fine, we return the version number of this plugin (can be queried with simGetModuleName)
    }

// This is the plugin end routine (called just once, when V-REP is ending, i.e. releasing this plugin):

VREP_DLLEXPORT void v_repEnd() {
    // Here you could handle various clean-up tasks

    unloadVrepLibrary(vrepLib); // release the library
}

// This is the plugin messaging routine (i.e. V-REP calls this function very often, with various messages):

VREP_DLLEXPORT void* v_repMessage(int message, int* auxiliaryData, void* customData, int* replyData) { // This is called quite often. Just watch out for messages/events you want to handle
  // Keep following 6 lines at the beginning and unchanged:
  simLockInterface(1);
  static bool refreshDlgFlag = true;
  int errorModeSaved;
  simGetIntegerParameter(sim_intparam_error_report_mode, &errorModeSaved);
  simSetIntegerParameter(sim_intparam_error_report_mode, sim_api_errormessage_ignore);
  void* retVal = NULL;

  // Here we can intercept many messages from V-REP (actually callbacks). Only the most important messages are listed here.
  // For a complete list of messages that you can intercept/react with, search for "sim_message_eventcallback"-type constants
  // in the V-REP user manual.

  if (message == sim_message_eventcallback_refreshdialogs)
      refreshDlgFlag = true; // V-REP dialogs were refreshed. Maybe a good idea to refresh this plugin's dialog too

  if (message == sim_message_eventcallback_menuitemselected) { // A custom menu bar entry was selected..
      // here you could make a plugin's main dialog visible/invisible
  }

  if (message == sim_message_eventcallback_instancepass) { // This message is sent each time the scene was rendered (well, shortly after) (very often)
      // It is important to always correctly react to events in V-REP. This message is the most convenient way to do so:

      int flags = auxiliaryData[0];
      bool sceneContentChanged = ((flags & (1 + 2 + 4 + 8 + 16 + 32 + 64 + 256)) != 0); // object erased, created, model or scene loaded, und/redo called, instance switched, or object scaled since last sim_message_eventcallback_instancepass message
      bool instanceSwitched = ((flags & 64) != 0);

      if (instanceSwitched) {
          // React to an instance switch here!!
      }

      if (sceneContentChanged) { // we actualize plugin objects for changes in the scene

          //...

          refreshDlgFlag = true; // always a good idea to trigger a refresh of this plugin's dialog here
      }
  }

  if (message == sim_message_eventcallback_mainscriptabouttobecalled) { // The main script is about to be run (only called while a simulation is running (and not paused!))

  }

  if (message == sim_message_eventcallback_simulationabouttostart) { // Simulation is about to start
    initializeSimulation();
  }

  if (message == sim_message_eventcallback_simulationended) { // Simulation just ended
    resetSimulation();
  }

  if (message == sim_message_eventcallback_moduleopen) { // A script called simOpenModule (by default the main script). Is only called during simulation.
      if ((customData == NULL) || (_stricmp("PluginSkeleton", (char*) customData) == 0)) // is the command also meant for this plugin?
      {
          // we arrive here only at the beginning of a simulation
      }
  }

  if (message == sim_message_eventcallback_modulehandle) { // A script called simHandleModule (by default the main script). Is only called during simulation.
      mainLoop();
      if ((customData == NULL) || (_stricmp("PluginSkeleton", (char*) customData) == 0)) // is the command also meant for this plugin?
      {
          // we arrive here only while a simulation is running
      }
  }

  if (message == sim_message_eventcallback_moduleclose) { // A script called simCloseModule (by default the main script). Is only called during simulation.
      if ((customData == NULL) || (_stricmp("PluginSkeleton", (char*) customData) == 0)) // is the command also meant for this plugin?
      {
          // we arrive here only at the end of a simulation
      }
  }

  if (message == sim_message_eventcallback_instanceswitch) { // Here the user switched the scene. React to this message in a similar way as you would react to a full
      // scene content change. In this plugin example, we react to an instance switch by reacting to the
      // sim_message_eventcallback_instancepass message and checking the bit 6 (64) of the auxiliaryData[0]
      // (see here above)

  }

  if (message == sim_message_eventcallback_broadcast) { // Here we have a plugin that is broadcasting data (the broadcaster will also receive this data!)

  }

  if (message == sim_message_eventcallback_scenesave) { // The scene is about to be saved. If required do some processing here (e.g. add custom scene data to be serialized with the scene)

  }

  // You can add many more messages to handle here
  if ((message == sim_message_eventcallback_guipass) && refreshDlgFlag) { // handle refresh of the plugin's dialogs
      // ...
      refreshDlgFlag = false;
  }

    // Keep following unchanged:
  simSetIntegerParameter(sim_intparam_error_report_mode, errorModeSaved); // restore previous settings
  simLockInterface(0);
  return retVal;
}
