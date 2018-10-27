#ifndef MPCSOLVER_HPP
#define MPCSOLVER_HPP
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <sys/time.h>
#include "qpOASES/qpOASES.hpp"

namespace mpcSolver{

    class MPCSolver{
	public:
        MPCSolver(float, float, float, Eigen::Vector3f, float, float, float, float, float, float, float, float);

        // Main method
        void solve(Eigen::Vector3f, Eigen::Vector3f, Eigen::Affine3f, bool, float, float, float, float, bool, Eigen::VectorXf, Eigen::VectorXf, Eigen::VectorXf);

        // Get stuff
        Eigen::VectorXf getOptimalCoMPosition();
        Eigen::VectorXf getOptimalCoMVelocity();
        Eigen::VectorXf getOptimalZMPPosition();
        Eigen::VectorXf getOptimalFootsteps();
        bool isRealEmergency();
        bool supportFootHasChanged();
	int getS();
	int getD();
	int getControlIteration();
	int getFootstepCounter();

	// Variable footstep timing
	void computeFootstepTiming();
	void generatePredictionMatrices();

        // Generate matrices
        void genCostFunction(Eigen::Affine3f);
	void genCostFunctionFootsteps(Eigen::Affine3f);
        void genStabilityConstraint();
        void genBalanceConstraint();
        void genBalanceConstraintEmergency(Eigen::Affine3f);
        void genFeasibilityConstraint();
        void genSwingFootConstraint(Eigen::Affine3f);

        // Solve
        void computeOrientations();
	void computeOrientationsFootsteps();
        Eigen::VectorXf solveQP();

        // Update the state
        Eigen::Vector3f updateState(float,int,float);
        void changeReferenceFrame(Eigen::Affine3f);

        void setCoMTargetHeight(float height);

        // Log
        void logToFile();

	private:

        // Constant parameters
        int N,S,D,M, footstepCounter;
        float singleSupportDuration, doubleSupportDuration, thetaMax;
        float footContraintSquareWidth;
        float deltaXMax;
        float deltaYIn;
        float deltaYOut;
        float mpcTimeStep;
        float controlTimeStep;
        float omega;
        float measuredComWeight = 0;
        float comTargetHeight=0;
	float predictionTime;

        // Parameters for the current iteration
        bool supportFoot;
        float simulationTime;
        float vRefX=0.1;
        float vRefY=0;
        float omegaRef=0;
        int mpcIter,controlIter;
        Eigen::VectorXf footstep_plan_x;
	Eigen::VectorXf footstep_plan_y;
	Eigen::VectorXf footstep_plan_theta;

        // Matrices for prediction
        Eigen::VectorXf p;
        Eigen::MatrixXf P;
        Eigen::MatrixXf Vu;
        Eigen::MatrixXf Vs;

        // Matrices for cost function
        Eigen::MatrixXf costFunctionH;
        Eigen::VectorXf costFunctionF;

        // Matrices for stability constraint
        Eigen::MatrixXf Aeq;
        Eigen::VectorXf beq;

        //Matrices for balance constraint
        Eigen::MatrixXf AZmp;
        Eigen::MatrixXf AZmpEmergency;
        Eigen::MatrixXf ASwingFoot;
        Eigen::VectorXf bZmpMax;
        Eigen::VectorXf bZmpMin;
        Eigen::VectorXf bZmpEmergency;
        Eigen::VectorXf bSwingFoot;

        // Matrices for feasibility constraints
        Eigen::MatrixXf AFootsteps;
        Eigen::VectorXf bFootstepsMax;
        Eigen::VectorXf bFootstepsMin;

        // Matrices for the stacked constraints
        Eigen::MatrixXf AConstraint;
        Eigen::VectorXf bConstraintLeft;
        Eigen::VectorXf bConstraintRight;

        // Solution of the QP for determining orientations
        Eigen::VectorXf predictedRotations;

        // Cost function weights
        float qZd = 1;
        float qVx = 10;
        float qVy = 10;
        float qZx = 1000;
        float qZy = 1000;
	float qF = 10000000;

        // State
        Eigen::Vector3f comPos;
        Eigen::Vector3f comVel;
        Eigen::Vector3f zmpPos;
        Eigen::Vector3f optimalFootsteps;

        //Emergency
        bool realEmergency=false;
        bool lastSupportEmergency=false;

	// Quadratic problem
	qpOASES::QProblem qp;
   };

}

#endif
