#include "MPCSolver.hpp"
#include "utils.cpp"
#include "qpOASES/qpOASES.hpp"
#include  <stdlib.h>
#include <iostream>
#include <fstream>
#include <Eigen/Cholesky>


using namespace mpcSolver;

struct Point
{
    float x, y;
};

// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are colinear
// 1 --> Clockwise
// 2 --> Counterclockwise
float orientation(Point p, Point q, Point r)
{
    float val = (q.y - p.y) * (r.x - q.x) -
              (q.x - p.x) * (r.y - q.y);

    if (val == 0) return 0;  // colinear
    return (val > 0)? 1: 2; // clock or counterclock wise
}

// Prints convex hull of a set of n points.
Eigen::MatrixXf convexHull(std::vector<Point> points)
{
    // There must be at least 3 points

    int n = points.size();

    if (n < 3) std::cout<<"ERROR: THERE MUST BE AT LEAST 3 POINTS"<<std::endl;

    // Initialize Result
    std::vector<Point> hull;

    // Find the leftmost point
    int l = 0;
    for (int i = 1; i < n; i++)
        if (points[i].x < points[l].x)
            l = i;

    // Start from leftmost point, keep moving counterclockwise
    // until reach the start point again.  This loop runs O(h)
    // times where h is number of points in result or output.
    int p = l, q;
    do
    {
        // Add current point to result
        hull.push_back(points[p]);

        // Search for a point 'q' such that orientation(p, x,
        // q) is counterclockwise for all points 'x'. The idea
        // is to keep track of last visited most counterclock-
        // wise point in q. If any point 'i' is more counterclock-
        // wise than q, then update q.
        q = (p+1)%n;
        for (int i = 0; i < n; i++)
        {
           // If i is more counterclockwise than current q, then
           // update q
           if (orientation(points[p], points[i], points[q]) == 2)
               q = i;
        }

        // Now q is the most counterclockwise with respect to p
        // Set p as q for next iteration, so that q is added to
        // result 'hull'
        p = q;

    } while (p != l);  // While we don't come to first point

    // Print Result
    Eigen::MatrixXf hullPoints(2,hull.size());
    for (int i = 0; i < hull.size(); i++){
        hullPoints.col(i)<<  hull[i].x,  hull[i].y;
    }

    return hullPoints;
}

MPCSolver::MPCSolver(float mpcTimeStep, float controlTimeStep, float predictionTime, Eigen::Vector3f initialComPosition,
                     float singleSupportDuration, float doubleSupportDuration, float thetaMax,
					 float footContraintSquareWidth, float deltaXMax, float deltaYIn, float deltaYOut, float measuredComWeight ){

	// Set up parameters
    this->mpcTimeStep = mpcTimeStep;
    this->controlTimeStep = controlTimeStep;
    this->predictionTime = predictionTime;

    this->footContraintSquareWidth = footContraintSquareWidth;
    this->deltaXMax = deltaXMax;
    this->deltaYIn = deltaYIn;
    this->deltaYOut = deltaYOut;
    this->thetaMax = thetaMax;
    this->comTargetHeight = initialComPosition(2);

    this->omega=sqrt(9.81/comTargetHeight);

    this->singleSupportDuration=singleSupportDuration;
    this->doubleSupportDuration=doubleSupportDuration;

    this->measuredComWeight = measuredComWeight;

    mpcIter = 0;
    controlIter = 0;

    generatePredictionMatrices();

    // Initialize CoM, ZMP and predicted footstep
    comPos = initialComPosition;
    comVel = Eigen::Vector3f::Zero(3); //CHANGE TO Vector3f
    zmpPos = Eigen::Vector3f(comPos(0),comPos(1),0.0);
    optimalFootsteps = Eigen::Vector3f::Zero();

    // Vector that contains predicted rotations
    predictedRotations = Eigen::VectorXf::Zero(M+1);

    // Initialize footstep counter
    footstepCounter=0;
}

void MPCSolver::solve(Eigen::Vector3f measuredComPos, Eigen::Vector3f measuredZmpPos,
					  Eigen::Affine3f swingFootTransform, bool supportFoot, float simulationTime, float vRefX, float vRefY, float omegaRef, bool emergencyStop,
					  Eigen::VectorXf footstep_plan_x, Eigen::VectorXf footstep_plan_y, Eigen::VectorXf footstep_plan_theta){

    // Save iteration parameters
    this->vRefX = vRefX;
    this->vRefY = vRefY;
    this->omegaRef = omegaRef;
    this->supportFoot = supportFoot;
    this->simulationTime = simulationTime;
    this->footstep_plan_x = footstep_plan_x;
    this->footstep_plan_y = footstep_plan_y;
    this->footstep_plan_theta = footstep_plan_theta;

    // If new footstep is starting, change reference frame
    if(supportFootHasChanged()) changeReferenceFrame(swingFootTransform);

    // Adjust the state based on measures
    comPos(0) = (1-measuredComWeight)*comPos(0) + measuredComWeight*measuredComPos(0);
    comPos(1) = (1-measuredComWeight)*comPos(1) + measuredComWeight*measuredComPos(1);

    // Compute footstep timing and generate matrices for new timing
    if(supportFootHasChanged()) {
	computeFootstepTiming();
	generatePredictionMatrices();
    }

    // Compute footsteps orientations
    computeOrientationsFootsteps();

    // Compute the cost function
    genCostFunctionFootsteps(swingFootTransform);

    // Compute the matrices for the constraints and stack them
    genStabilityConstraint();
    genBalanceConstraint(); 
    genFeasibilityConstraint();
    genSwingFootConstraint(swingFootTransform);


    if(emergencyStop && mpcIter>S && realEmergency==false){
        realEmergency=true;
        lastSupportEmergency=supportFoot;
    }

    if(!emergencyStop && mpcIter>S && supportFoot==lastSupportEmergency)
        realEmergency=false;

    /*if ((footstepCounter == 0 || mpcIter >= S) && !realEmergency) {*/
        int nConstraints = Aeq.rows() + AFootsteps.rows() + AZmp.rows();// + ASwingFoot.rows();
        AConstraint.resize(nConstraints, 2*(N+M));
        bConstraintLeft.resize(nConstraints);
        bConstraintRight.resize(nConstraints);

        AConstraint 	 << Aeq, 0*AFootsteps, AZmp;//, ASwingFoot;
        bConstraintLeft  << beq, -0*bFootstepsMin, -bZmpMin;//, bSwingFoot;
        bConstraintRight << beq, 0*bFootstepsMax, bZmpMax;//, bSwingFoot;
    /*}
    else if(!realEmergency) {
        int nConstraints = Aeq.rows() + AFootsteps.rows() + AZmp.rows();
        AConstraint.resize(nConstraints, 2*(N+M));
        bConstraintLeft.resize(nConstraints);
        bConstraintRight.resize(nConstraints);

        AConstraint 	 << Aeq, AFootsteps, AZmp;
        bConstraintLeft  << beq, -bFootstepsMin, -bZmpMin;
        bConstraintRight << beq, bFootstepsMax, bZmpMax;
    }
    else{
        genBalanceConstraintEmergency(swingFootTransform);
        int nConstraints = Aeq.rows() + ASwingFoot.rows() + AZmpEmergency.rows() ;
        AConstraint.resize(nConstraints, 2*(N+M));
        bConstraintLeft.resize(nConstraints);
        bConstraintRight.resize(nConstraints);

        AConstraint 	 << Aeq,ASwingFoot,AZmpEmergency;
        bConstraintLeft  << beq, bSwingFoot,-Eigen::VectorXf::Ones(AZmpEmergency.rows())*1E06;
        bConstraintRight << beq, bSwingFoot, bZmpEmergency;

    }*/

    // Solve QP
    Eigen::VectorXf decisionVariables = solveQP();

    //std::cout<<decisionVariables.transpose()<<std::endl;

    // Split the QP solution in ZMP dot and footsteps
    Eigen::VectorXf zDotOptimalX(N);
    Eigen::VectorXf zDotOptimalY(N);
    Eigen::VectorXf footstepsOptimalX(M);
    Eigen::VectorXf footstepsOptimalY(M);

    zDotOptimalX = (decisionVariables.head(N));
    zDotOptimalY = (decisionVariables.segment(N+M,N));
    footstepsOptimalX = decisionVariables.segment(N,M);
    footstepsOptimalY = decisionVariables.segment(2*N+M,M);

    // Update the state based on the result of the QP
    Eigen::Vector3f nextStateX = updateState(zDotOptimalX(0),0,controlTimeStep);
    Eigen::Vector3f nextStateY = updateState(zDotOptimalY(0),1,controlTimeStep);

    comPos << nextStateX(0),nextStateY(0),comTargetHeight;
    comVel << nextStateX(1),nextStateY(1),0.0;
    zmpPos << nextStateX(2),nextStateY(2),0.0;
    optimalFootsteps << footstepsOptimalX(0),footstepsOptimalY(0),predictedRotations(1);

    ++controlIter;
    mpcIter = floor(controlIter*controlTimeStep/mpcTimeStep);
    if(mpcIter>=S+D){
        controlIter = 0;
    	mpcIter = 0;
        footstepCounter++;
    }

    // std::cout << "Iteration " << controlIter << " Footstep " << footstepCounter << std::endl;
}

void MPCSolver::computeFootstepTiming() {

    double alpha = 0.1;
    double averageVel = 0.15;
    double averageTime = 0.8;

    //double stepDuration = std::max(1.2 - 0.6*abs(vRefX)/0.3, 0.6);
    //double stepDuration = std::max(averageTime - averageTime*abs(vRefX)/(alpha + abs(vRefX)) + averageTime*averageVel/(alpha + abs(vRefX)), 0.6);
    double stepDuration = std::max(averageTime*(1 - (abs(vRefX) - averageVel)/(alpha + abs(vRefX))), 0.6);

    doubleSupportDuration = round(4*stepDuration)/10;
    singleSupportDuration = round(10*stepDuration)/10 - doubleSupportDuration;

    predictionTime = 2*(singleSupportDuration+doubleSupportDuration);

    std::cout << "Single support duration: " << singleSupportDuration << std::endl;
    std::cout << "Double support duration: " << doubleSupportDuration << std::endl;
}

void MPCSolver::generatePredictionMatrices() {
    N = round(predictionTime/mpcTimeStep);
    S = round(singleSupportDuration/mpcTimeStep);
    D = round(doubleSupportDuration/mpcTimeStep);
    M = ceil(N/(S+D));
    
    // Matrices for cost function
    costFunctionH = Eigen::MatrixXf::Zero(2*(N+M),2*(N+M));
    costFunctionF = Eigen::VectorXf::Zero(2*(N+M));

    // Matrices for stability constraint
    Aeq = Eigen::MatrixXf::Zero(2,(N*2)+(M*2));
    beq = Eigen::VectorXf::Zero(2);

    // Matrices for swing foot constraint
    ASwingFoot = Eigen::MatrixXf::Zero(2,(N*2)+(M*2));
    bSwingFoot = Eigen::VectorXf::Zero(2);

    // Matrices for ZMP constraint
    AZmp = Eigen::MatrixXf::Zero(2*N,2*(N+M));
    bZmpMax = Eigen::VectorXf::Zero(2*N);
    bZmpMin = Eigen::VectorXf::Zero(2*N);

    // Matrices for feasibility constraint
    AFootsteps = Eigen::MatrixXf::Zero(2*M,2*(M+N));
    bFootstepsMax = Eigen::VectorXf::Zero(2*M);
    bFootstepsMin = Eigen::VectorXf::Zero(2*M);

    // Matrices for all constraints stacked
    AConstraint = Eigen::MatrixXf::Zero(2*(N+M)+2,2*(N+M));
    bConstraintLeft = Eigen::VectorXf::Zero(2*(N+M)+2);
    bConstraintRight = Eigen::VectorXf::Zero(2*(N+M)+2);

    // Matrices for ZMP prediction
    p =  Eigen::VectorXf::Ones(N);
    P =  Eigen::MatrixXf::Ones(N,N)*mpcTimeStep;

    for(int i=0; i<N;++i){
    	for(int j=0;j<N;++j){
            if (j>i) P(i,j)=0;
        }
    }

    // Matrices for CoM velocity prediction
    Vu = Eigen::MatrixXf::Zero(N,N);
    Vs = Eigen::MatrixXf::Zero(N,3);

    float ch = cosh(omega*mpcTimeStep);
    float sh = sinh(omega*mpcTimeStep);

    Eigen::MatrixXf A_upd = Eigen::MatrixXf::Zero(3,3);
    Eigen::VectorXf B_upd = Eigen::VectorXf::Zero(3);
    A_upd<<ch,sh/omega,1-ch,omega*sh,ch,-omega*sh,0,0,1;
    B_upd<<mpcTimeStep-sh/omega,1-ch,mpcTimeStep;

    Eigen::RowVectorXf Vu_newline(N);
    Eigen::RowVectorXf Vs_newline(3);
    Eigen::RowVectorXf A_midLine(3);

    A_midLine<<omega*sh,ch,-omega*sh;

    for(int i=1;i<=N;++i) {
        Vu_newline.setZero();
        Vs_newline.setZero();
        Vu_newline(i-1) = 1-ch;
        if(i>1) {
            for(int j=0;j<=i-2;++j) {
                Vu_newline.segment(j,1) = A_midLine*(matrixPower(A_upd,(i-j-2)))*B_upd;
            }
        }
        Vs_newline = A_midLine*(matrixPower(A_upd,(i-1)));
        Vu.row(i-1) = Vu_newline;
        Vs.row(i-1) = Vs_newline;
    }
}

void MPCSolver::changeReferenceFrame(Eigen::Affine3f swingFootTransform) {

	Eigen::Vector3f swingFootPos = swingFootTransform.translation();

        comPos=swingFootTransform.rotation()*comPos;
	comVel=swingFootTransform.rotation()*comVel;
	zmpPos=swingFootTransform.rotation()*zmpPos;

	comPos(0)-= -swingFootPos(0);
	comPos(1)-= -swingFootPos(1);
	zmpPos(0)-= -swingFootPos(0);
	zmpPos(1)-= -swingFootPos(1);

	// WARNING: Check if this rotation matrix should be transposed

}

void MPCSolver::genStabilityConstraint() {
	Eigen::VectorXf b(N);
  	Aeq.setZero();

	for(int i=0;i<N;++i){
		b(i) = pow(exp(-omega*mpcTimeStep),i);
	}

	//Aeq.block(0,0,1,N)   = ((1/omega)*((1-exp(-omega*mpcTimeStep))/(1-pow(exp(-omega*mpcTimeStep),N))))*b.transpose();
	//Aeq.block(1,N+M,1,N) = ((1/omega)*((1-exp(-omega*mpcTimeStep))/(1-pow(exp(-omega*mpcTimeStep),N))))*b.transpose();
	Aeq.block(0,0,1,N)   = (1/omega)*(1-exp(-omega*mpcTimeStep))*b.transpose();
	Aeq.block(1,N+M,1,N) = (1/omega)*(1-exp(-omega*mpcTimeStep))*b.transpose();

	beq<<comPos(0) + comVel(0)/omega - zmpPos(0), comPos(1) + comVel(1)/omega - zmpPos(1);
}

void MPCSolver::genCostFunctionFootsteps(Eigen::Affine3f swingFootTransform) {

	costFunctionH.block(0,0,N,N) = qZd*Eigen::MatrixXf::Identity(N,N);
  	costFunctionH.block(N+M,N+M,N,N) = qZd*Eigen::MatrixXf::Identity(N,N);
	costFunctionH.block(N,N,M,M) = qF*Eigen::MatrixXf::Identity(M,M);
  	costFunctionH.block(2*N+M,2*N+M,M,M) = qF*Eigen::MatrixXf::Identity(M,M);

  	Eigen::VectorXf costFunctionF1 = Eigen::VectorXf::Zero(N+M);
  	Eigen::VectorXf costFunctionF2 = Eigen::VectorXf::Zero(N+M);

        costFunctionF1.block(N,0,M,1) = -qF*footstep_plan_x;
        costFunctionF2.block(N,0,M,1) = -qF*footstep_plan_y;

        costFunctionF<<costFunctionF1,costFunctionF2;
}

void MPCSolver::genCostFunction(Eigen::Affine3f swingFootTransform) {

  if(!realEmergency){
  	costFunctionH.block(0,0,N,N) = qZd*Eigen::MatrixXf::Identity(N,N) + qVx*Vu.transpose()*Vu;
  	costFunctionH.block(N+M,N+M,N,N) = qZd*Eigen::MatrixXf::Identity(N,N) + qVy*Vu.transpose()*Vu;

  	Eigen::VectorXf vArcX = Eigen::VectorXf::Zero(N);
  	Eigen::VectorXf vArcY = Eigen::VectorXf::Zero(N);

  	Eigen::VectorXf costFunctionF1 = Eigen::VectorXf::Zero(N+M);
  	Eigen::VectorXf costFunctionF2 = Eigen::VectorXf::Zero(N+M);

  	for(int i=0;i<N;++i){
  		vArcX(i) = vRefX*cos(i*omegaRef*mpcTimeStep) - vRefY*sin(i*omegaRef*mpcTimeStep);
  		vArcY(i) = vRefX*sin(i*omegaRef*mpcTimeStep) + vRefY*cos(i*omegaRef*mpcTimeStep);
  	}

      Eigen::Vector3f stateX = Eigen::Vector3f(comPos(0),comVel(0),zmpPos(0));
      Eigen::Vector3f stateY = Eigen::Vector3f(comPos(1),comVel(1),zmpPos(1));

      costFunctionF1.block(0,0,N,1) = (qVx*Vu.transpose())*((Vs*stateX)-vArcX);
      costFunctionF2.block(0,0,N,1) = (qVy*Vu.transpose())*((Vs*stateY)-vArcY);

      costFunctionF<<costFunctionF1,costFunctionF2;
    }
  else{

    Eigen::Vector3f footPosition=swingFootTransform.translation();
    Eigen::MatrixXf rotMatrix = swingFootTransform.rotation();

    Eigen::Vector3f RPY;
    RPY << atan2(rotMatrix(2,1),rotMatrix(2,2)),
            atan2(-rotMatrix(2,0),sqrt(rotMatrix(2,1)*rotMatrix(2,1)+rotMatrix(2,2)*rotMatrix(2,2))),
            atan2(rotMatrix(1,0),rotMatrix(0,0));

    float footx=footPosition(0);
    float footy=footPosition(1);

    costFunctionH.block(0,0,N,N) = qZd*Eigen::MatrixXf::Identity(N,N) + qZx * (P.transpose()*P);
    costFunctionH.block(N+M,N+M,N,N) = qZd*Eigen::MatrixXf::Identity(N,N) + qZy * (P.transpose()*P);

    Eigen::VectorXf costFunctionF1 = Eigen::VectorXf::Zero(N+M);
    Eigen::VectorXf costFunctionF2 = Eigen::VectorXf::Zero(N+M);

    float x_hull=footx/2;
    float y_hull = footy/2;

    costFunctionF1.block(0,0,N,1) = qZx * P.transpose()* p * (zmpPos(0)-x_hull);
    costFunctionF2.block(0,0,N,1) = qZy * P.transpose()* p * (zmpPos(1)-y_hull);

    costFunctionF<<costFunctionF1,costFunctionF2;
  }
}

void MPCSolver::computeOrientations() {
	Eigen::MatrixXf Iprev = Eigen::MatrixXf::Identity(M,M);
	for (int i=0; i<M-1; ++i){
		Iprev(i+1,i) = -1;
	}
	Eigen::MatrixXf footstepsH = Eigen::MatrixXf::Zero(M,M);
	Eigen::VectorXf footstepsF = Eigen::VectorXf::Zero(M);

	Eigen::MatrixXf footstepsIdentity = Eigen::MatrixXf::Identity(M,M);
	footstepsH = (Iprev.transpose()*Iprev);

	footstepsF = -omegaRef*(singleSupportDuration+doubleSupportDuration)*Iprev.transpose()*Eigen::VectorXf::Ones(M);

	qpOASES::real_t thetaOpt[M];

	qpOASES::Options optionsRotations;
	optionsRotations.setToMPC();
	optionsRotations.printLevel=qpOASES::PL_NONE;
	qpOASES::QProblemB qpRotations(M);
	qpRotations.setOptions(optionsRotations);

	qpOASES::int_t nWSRRotations = 300;

	qpOASES::real_t footstepsHqpOASES[M*M];
	qpOASES::real_t footstepsFqpOASES[M];

	for(int i=0;i<M;++i){
		for(int j=0;j<M;++j){
			footstepsHqpOASES[i*M+j] = footstepsH(i,j);
		}
		footstepsFqpOASES[i] = footstepsF(i);
	}

	qpRotations.init(footstepsHqpOASES,footstepsFqpOASES,0,0,nWSRRotations);
	qpRotations.getPrimalSolution(thetaOpt);

	// FIX SATURATION WITH CONSTRAINTS
	predictedRotations(0)=0;
	if(footstepCounter==0){
		for(int i=1;i<M;++i){
			predictedRotations(1) = 0;
			predictedRotations(i+1) = thetaOpt[i-1];
		}
	}
	else{
		for(int i=1;i<M+1;++i){
			predictedRotations(i) = thetaOpt[i-1];
		}
	}

	for(int i=0;i<M;++i){
		if(angDiff(predictedRotations(i+1),predictedRotations(i))>=thetaMax){
			predictedRotations(i+1)=wrapToPi(predictedRotations(i)+thetaMax);
		}
		else if(angDiff(predictedRotations(i+1),predictedRotations(i))<=-thetaMax){
			predictedRotations(i+1)=wrapToPi(predictedRotations(i)-thetaMax);
		}
	}
}

void MPCSolver::computeOrientationsFootsteps() {

	Eigen::MatrixXf footstepsH = Eigen::MatrixXf::Identity(M,M);
	Eigen::VectorXf footstepsF = -footstep_plan_theta;

	qpOASES::real_t thetaOpt[M];

	qpOASES::Options optionsRotations;
	optionsRotations.setToMPC();
	optionsRotations.printLevel=qpOASES::PL_NONE;
	qpOASES::QProblemB qpRotations(M);
	qpRotations.setOptions(optionsRotations);

	qpOASES::int_t nWSRRotations = 300;

	qpOASES::real_t footstepsHqpOASES[M*M];
	qpOASES::real_t footstepsFqpOASES[M];

	for(int i=0;i<M;++i){
		for(int j=0;j<M;++j){
			footstepsHqpOASES[i*M+j] = footstepsH(i,j);
		}
		footstepsFqpOASES[i] = footstepsF(i);
	}

	qpRotations.init(footstepsHqpOASES,footstepsFqpOASES,0,0,nWSRRotations);
	qpRotations.getPrimalSolution(thetaOpt);

	// FIX SATURATION WITH CONSTRAINTS
	predictedRotations(0)=0;
	if(footstepCounter==0){
		for(int i=1;i<M;++i){
			predictedRotations(1) = 0;
			predictedRotations(i+1) = thetaOpt[i-1];
		}
	}
	else{
		for(int i=1;i<M+1;++i){
			predictedRotations(i) = thetaOpt[i-1];
		}
	}

	for(int i=0;i<M;++i){
		if(angDiff(predictedRotations(i+1),predictedRotations(i))>=thetaMax){
			predictedRotations(i+1)=wrapToPi(predictedRotations(i)+thetaMax);
		}
		else if(angDiff(predictedRotations(i+1),predictedRotations(i))<=-thetaMax){
			predictedRotations(i+1)=wrapToPi(predictedRotations(i)-thetaMax);
		}
	}

	//std::cout << predictedRotations.transpose() << std::endl;
}

void MPCSolver::genBalanceConstraint(){


  AZmp.setZero();
	Eigen::MatrixXf Icf = Eigen::MatrixXf::Zero(S+D,S+D);
	Eigen::MatrixXf Ic = Eigen::MatrixXf::Zero(N,N);
	Eigen::MatrixXf Cc = Eigen::MatrixXf::Zero(N,M);
	Eigen::VectorXf Ccf = Eigen::VectorXf::Ones(S+D);

	for(int i=0;i<S;++i){
		Icf(i,i)=1;
	}

	if((int)simulationTime/mpcTimeStep<S+D){
		Ic.block(0,0,S+D-mpcIter,S+D-mpcIter) = Eigen::MatrixXf::Zero(S+D-mpcIter,S+D-mpcIter);
	}
	else{
		Ic.block(0,0,S+D-mpcIter,S+D-mpcIter) = Icf.block(mpcIter,mpcIter,S+D-mpcIter,S+D-mpcIter);
	}

	for(int i=0; i<M-1; ++i){
		Ic.block(S+D-mpcIter+(i*(S+D)),S+D-mpcIter+(i*(S+D)),S+D,S+D) = Icf;
	}

	Ic.block(S+D-mpcIter+((M-1)*(S+D)),S+D-mpcIter+((M-1)*(S+D)),mpcIter,mpcIter) = Icf.block(0,0,mpcIter,mpcIter);

	for(int i=0; i<M-1; ++i){
		Cc.block(S+D-mpcIter+(i*(S+D)),i,S+D,1) = Ccf;
	}

	Cc.block(S+D-mpcIter+((M-1)*(S+D)),M-1,mpcIter,1) = Ccf.block(0,0,mpcIter,1);

	Eigen::MatrixXf rCosZmp = Eigen::MatrixXf::Zero(N,N);
	Eigen::MatrixXf rSinZmp = Eigen::MatrixXf::Zero(N,N);

	rCosZmp.block(0,0,S+D-mpcIter,S+D-mpcIter) = Eigen::MatrixXf::Identity(S+D-mpcIter,S+D-mpcIter);
	rSinZmp.block(0,0,S+D-mpcIter,S+D-mpcIter) = Eigen::MatrixXf::Zero(S+D-mpcIter,S+D-mpcIter);

	for(int i=0; i<M-1; ++i){
		rCosZmp.block(S+D-mpcIter+(i*(S+D)),S+D-mpcIter+(i*(S+D)),S+D,S+D) = Eigen::MatrixXf::Identity(S+D,S+D)*cos(predictedRotations(i+1));
		rSinZmp.block(S+D-mpcIter+(i*(S+D)),S+D-mpcIter+(i*(S+D)),S+D,S+D) = Eigen::MatrixXf::Identity(S+D,S+D)*sin(predictedRotations(i+1));
	}

	rCosZmp.block(S+D-mpcIter+((M-1)*(S+D)),S+D-mpcIter+((M-1)*(S+D)),mpcIter,mpcIter) = Eigen::MatrixXf::Identity(S+D,S+D).block(0,0,mpcIter,mpcIter)*cos(predictedRotations(M));
	rSinZmp.block(S+D-mpcIter+((M-1)*(S+D)),S+D-mpcIter+((M-1)*(S+D)),mpcIter,mpcIter) = Eigen::MatrixXf::Identity(S+D,S+D).block(0,0,mpcIter,mpcIter)*sin(predictedRotations(M));

	Eigen::MatrixXf zmpRotationMatrix(2*N,2*N);
	zmpRotationMatrix << rCosZmp,rSinZmp,
						-rSinZmp,rCosZmp;

	AZmp.block(0,0,N,N) = Ic*P;
	AZmp.block(0,N,N,M) = -Ic*Cc;
	AZmp.block(N,N+M,N,N) = Ic*P;
	AZmp.block(N,2*N+M,N,M) = -Ic*Cc;

	AZmp = zmpRotationMatrix*AZmp;

	Eigen::VectorXf bZmpLeft = Eigen::VectorXf::Zero(2*N);
	Eigen::VectorXf bZmpRight = Eigen::VectorXf::Zero(2*N);

	bZmpLeft<<Ic*p*footContraintSquareWidth/2,Ic*p*footContraintSquareWidth/2;

	bZmpRight<<Ic*p*zmpPos(0),Ic*p*zmpPos(1);
	bZmpRight=zmpRotationMatrix*bZmpRight;

	bZmpMax=bZmpLeft-bZmpRight;
	bZmpMin=bZmpLeft+bZmpRight;
}

void MPCSolver::genBalanceConstraintEmergency(Eigen::Affine3f swingFootTransform){

  float footx=0;
  float footy=0;
  float foot_angle=0;

  Eigen::Vector3f footPosition=swingFootTransform.translation();
  Eigen::MatrixXf rotMatrix = swingFootTransform.rotation();

  Eigen::Vector3f RPY;
  RPY << atan2(rotMatrix(2,1),rotMatrix(2,2)),
          atan2(-rotMatrix(2,0),sqrt(rotMatrix(2,1)*rotMatrix(2,1)+rotMatrix(2,2)*rotMatrix(2,2))),
          atan2(rotMatrix(1,0),rotMatrix(0,0));

  footx=footPosition(0);
  footy=footPosition(1);

  foot_angle=RPY(2);

  Eigen::Matrix2f footRotation;
  footRotation << cos(foot_angle), -sin(foot_angle), sin(foot_angle), cos(foot_angle);

  Eigen::MatrixXf footVertices(2,4);
  footVertices << footContraintSquareWidth/2,-footContraintSquareWidth/2,-footContraintSquareWidth/2,footContraintSquareWidth/2,
                  -footContraintSquareWidth/2,-footContraintSquareWidth/2,footContraintSquareWidth/2,footContraintSquareWidth/2;

  footVertices=footRotation*footVertices;

  //std::cout<<"footVertices: "<<footVertices<<std::endl;

  std::vector<Point> points = {{footContraintSquareWidth/2, -footContraintSquareWidth/2},
                               {-footContraintSquareWidth/2, -footContraintSquareWidth/2},
                               {-footContraintSquareWidth/2, footContraintSquareWidth/2},
                               {footContraintSquareWidth/2, footContraintSquareWidth/2},
                               {footx + footVertices(0,0),footy + footVertices(1,0)},
                               {footx + footVertices(0,1),footy + footVertices(1,1)},
                               {footx + footVertices(0,2),footy + footVertices(1,2)},
                               {footx + footVertices(0,3),footy + footVertices(1,3)}};

  Eigen::MatrixXf hullPoints = convexHull(points);

  //std::cout<<"hullPoints: "<<hullPoints<<std::endl;

  hullPoints.conservativeResize(hullPoints.rows(), hullPoints.cols()+1);
  hullPoints.col(hullPoints.cols()-1)=hullPoints.col(0);

  Eigen::Matrix2f R;
  R<<0,1,-1,0;

  Eigen::MatrixXf AQ(N*(hullPoints.cols()-1),2*(N+M));
  Eigen::VectorXf bQ(N*(hullPoints.cols()-1));

  for(int i=0;i<hullPoints.cols()-1;++i){
      for(int j=0; j<N;++j){
          Eigen::MatrixXf Phull(2, 2*(N+M));

          Phull<<Eigen::MatrixXf::Ones(1,j+1)*mpcTimeStep,Eigen::MatrixXf::Zero(1,N-j-1), Eigen::MatrixXf::Zero(1,N), Eigen::MatrixXf::Zero(1,2*M),
                 Eigen::MatrixXf::Zero(1,N),Eigen::MatrixXf::Ones(1,j+1)*mpcTimeStep, Eigen::MatrixXf::Zero(1,N-j-1), Eigen::MatrixXf::Zero(1,2*M);

          AQ.block(j+(N*i),0,1,2*(N+M)) = (R*(hullPoints.col(i+1)-hullPoints.col(i))).transpose()*Phull;

          bQ.segment(j+(N*i),1) = -(R*(hullPoints.col(i+1)-hullPoints.col(i))).transpose()*zmpPos.segment(0,2) + (R*(hullPoints.col(i+1)-hullPoints.col(i))).transpose()*hullPoints.col(i);
      }
  }

  AZmpEmergency=AQ;
  bZmpEmergency=bQ;

}

void MPCSolver::genFeasibilityConstraint(){
  AFootsteps.setZero();
  Eigen::MatrixXf Iprev = Eigen::MatrixXf::Identity(M,M);
	for (int i=0; i<M-1; ++i){
		Iprev(i+1,i) = -1;
	}
	Eigen::VectorXf pFr = Eigen::VectorXf::Zero(M);
	Eigen::VectorXf pFl = Eigen::VectorXf::Zero(M);
	Eigen::VectorXf pF = Eigen::VectorXf::Ones(M);
	Eigen::MatrixXf footstepsRotationMatrix(2*M,2*M);
	Eigen::MatrixXf rCosFootsteps = Eigen::MatrixXf::Identity(M,M);
	Eigen::MatrixXf rSinFootsteps = Eigen::MatrixXf::Zero(M,M);

	AFootsteps.block(0,N,M,M) = Iprev;
	AFootsteps.block(M,2*N+M,M,M) = Iprev;

	rCosFootsteps(0,0) = 1;
	rSinFootsteps(0,0) = 0;

	for(int i=1; i<M; ++i){
		rCosFootsteps(i,i) = cos(predictedRotations(i));
		rSinFootsteps(i,i) = sin(predictedRotations(i));
	}

	footstepsRotationMatrix <<  rCosFootsteps, rSinFootsteps,
							   -rSinFootsteps, rCosFootsteps;

	AFootsteps=footstepsRotationMatrix*AFootsteps;

	if(supportFoot==true){
		for(int i=0;i<M;++i){
			if(i%2==0) pFr(i) = 1;
			else pFl(i) = 1;
		}
	}
	else{
		for(int i=0;i<M;++i){
			if(i%2==0) pFl(i) = 1;
			else pFr(i) = 1;
		}
	}

	bFootstepsMax<<pF*deltaXMax,-pFl*deltaYIn+pFr*deltaYOut;
	bFootstepsMin<<pF*deltaXMax,pFl*deltaYOut-pFr*deltaYIn;
}

void MPCSolver::genSwingFootConstraint(Eigen::Affine3f swingFootTransform) {

        Eigen::Vector3f swingFootPos = swingFootTransform.translation();

        ASwingFoot(0,N) = 1;
        ASwingFoot(1,2*N+M) = 1;

        bSwingFoot(0) = swingFootPos(0);
        bSwingFoot(1) = swingFootPos(1);
}

Eigen::VectorXf MPCSolver::solveQP() {
	int nVariables = costFunctionH.rows();
	int nConstraints = AConstraint.rows();

	qpOASES::real_t H[nVariables*nVariables];
	qpOASES::real_t g[nVariables];

	qpOASES::real_t A[nConstraints*nVariables];
	qpOASES::real_t lb[nConstraints];
	qpOASES::real_t ub[nConstraints];

	for(int i=0;i<nVariables;++i){
		for(int j=0;j<nVariables;++j){
			H[i*nVariables+j] = costFunctionH(i,j);
		}
		g[i] = costFunctionF(i);
	}

	for(int i=0;i<nConstraints;++i){
		for(int j=0;j<nVariables;++j){
			A[i*nVariables+j] = AConstraint(i,j);
		}
		lb[i] = bConstraintLeft(i);
		ub[i] = bConstraintRight(i);
	}

	qpOASES::real_t xOpt[nVariables];

	qpOASES::Options options;
	options.setToMPC();
	options.printLevel=qpOASES::PL_NONE;
	qpOASES::int_t nWSR = 300;

	qp = qpOASES::QProblem(nVariables, nConstraints);
	qp.setOptions(options);
	qp.init(H,g,A,0,0,lb,ub,nWSR,NULL,NULL,NULL,NULL,NULL,NULL);

	qp.getPrimalSolution(xOpt);

	Eigen::VectorXf decisionVariables(2*(N+M));

	for(int i=0;i<2*(N+M);++i){
		decisionVariables(i) = xOpt[i];
	}

	return decisionVariables;

}

Eigen::Vector3f MPCSolver::updateState(float zmpDot, int dim, float timeStep) {
	// Update the state along the dim-th direction (0,1,2) = (x,y,z)

  float ch = cosh(omega*timeStep);
  float sh = sinh(omega*timeStep);

	Eigen::Matrix3f A_upd = Eigen::MatrixXf::Zero(3,3);
	Eigen::Vector3f B_upd = Eigen::VectorXf::Zero(3);
	A_upd<<ch,sh/omega,1-ch,omega*sh,ch,-omega*sh,0,0,1;
	B_upd<<timeStep-sh/omega,1-ch,timeStep;

  Eigen::Vector3f currentState = Eigen::Vector3f(comPos(dim),comVel(dim),zmpPos(dim));

  return A_upd*currentState + B_upd*zmpDot;
}

bool MPCSolver::supportFootHasChanged(){
    if (controlIter==0 && footstepCounter>0) return true;
    else return false;
}

Eigen::VectorXf MPCSolver::getOptimalCoMPosition(){
    return comPos;
}

Eigen::VectorXf MPCSolver::getOptimalCoMVelocity(){
    return comVel;
}

Eigen::VectorXf MPCSolver::getOptimalFootsteps(){
    return optimalFootsteps;
}

Eigen::VectorXf MPCSolver::getOptimalZMPPosition(){
    return zmpPos;
}

void MPCSolver::setCoMTargetHeight(float height){
    this->comTargetHeight=height;
}

bool MPCSolver::isRealEmergency(){
  return realEmergency;
}

int MPCSolver::getS(){
  return S;
}

int MPCSolver::getD(){
  return D;
}

int MPCSolver::getControlIteration(){
  return this->controlIter;
}

int MPCSolver::getFootstepCounter(){
  return this->footstepCounter;
}

