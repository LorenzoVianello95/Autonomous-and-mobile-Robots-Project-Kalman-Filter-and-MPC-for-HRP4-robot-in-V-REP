/* 
 * File:   MotionGenerator.cpp
 * Author: Paolo Ferrari
 *
 * Created on February 26, 2016 
 */

#include "MotionGenerator.hpp"
#include <stdlib.h>

MotionGenerator::MotionGenerator(){}

MotionGenerator::MotionGenerator(Task* _task, Robot* _robot, Kinematics* _kin){	
	task = _task;
	robot = _robot; 
	kin = _kin;
	intStep = INTEGRATION_STEP; 
	taskType = task->getTaskType();	
}

MotionGenerator::~MotionGenerator(){}

std::map<float, Configuration> MotionGenerator::generateForwardMotion(Node _n, float _planning_time, float _time_budget){

	///std::cout << "Generating Motion..." << std::endl; 
	State _s = _n.getState();

	planning_time = _planning_time; //////////////
	time_budget = _time_budget;	
	
	std::map<float, Configuration> motion;

	Configuration _qCurr = _s.getConfiguration();
	typeCurr = _s.getMovementPrimitiveType();
	tk = _s.getTime();
	dof = _qCurr.getqjnt().size();
	Eigen::Vector3f pCoM = _qCurr.getqCoMPosition();
	Eigen::Vector3f oCoM = _qCurr.getqCoMOrientation(); 
	qCurr.setqCoMPosition(pCoM);
	qCurr.setqCoMOrientation(oCoM);
	qCurr.setqjnt(_qCurr.getqjnt());
	
	std::vector<MovementPrimitiveType> available_set;
	Eigen::VectorXf weights_available_set;
	available_set = _n.getAvailableSet();	
	weights_available_set = _n.getWeightsAvailableSet();

	////////////////////////////////////////////////////////////////////////////////////
	if(!isInArea(qCurr, pCoMRoot)){
		Eigen::Vector3f goal = task->getAttractor(task->getLastReachedAttractor()+1);
		Configuration qFin(dof);
		Eigen::Vector3f pCoMFin;
		Eigen::Vector3f pCoMIni = qCurr.getqCoMPosition();
		pCoMFin(0) = goal(0);
		pCoMFin(1) = goal(1);
		pCoMFin(2) = pCoMIni(2); 
		qFin.setqCoMPosition(pCoMFin);
		qFin.setqCoMOrientation(qCurr.getqCoMOrientation());
		qFin.setqjnt(qCurr.getqjnt());
		motion.insert(std::pair<float, Configuration>(tk + intStep, qFin));
		type = TRANSLATION;
		duration = intStep;

		float l = sqrt(std::pow(pCoMFin(0)-pCoMIni(0),2) + std::pow(pCoMFin(1)-pCoMIni(1),2) + std::pow(pCoMFin(2)-pCoMIni(2),2));	
		float s = 0.0;
		float delta_s = 0.05;
		Eigen::Vector3f pCoM_s;
		while(s < l){
			pCoM_s = pCoMIni + (s/l)*(pCoMFin - pCoMIni);	
			Configuration q_s(dof);
			q_s.setqCoMPosition(pCoM_s);
			q_s.setqCoMOrientation(qCurr.getqCoMOrientation());
			q_s.setqjnt(qCurr.getqjnt());

			if(collisionCheck(q_s)){
				motion.clear();
				break;	
			}
			
			s = s + delta_s;
		}
 	
		return motion;
	}
	////////////////////////////////////////////////////////////////////////////////////

	//type = chooseMovementPrimitive(available_set, weights_available_set);
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////  
	int typeFromInput;
	std::cout << "Enter a Primitive Type: " << std::endl;   
	std::cin >> typeFromInput;  
	if(typeFromInput == 77){
		motion.clear();
		return motion;
	}
	type = (MovementPrimitiveType)typeFromInput;
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////	 

	prim = new MovementPrimitive((int)type, qCurr, tk, robot, kin);	
	//Eigen::VectorXf pCoMFinal = prim->getzCoM()->getPosition(prim->getDuration());
	//Eigen::VectorXf pSwgFinal = prim->getzswg()->getPosition(prim->getDuration());
	
	//std::cout << "pCoMFinal :: " << pCoMFinal.transpose() << std::endl;
	//std::cout << "pSwgFinal :: " << pSwgFinal.transpose() << std::endl;
	
	// set the duration of the FREE_COM
	if(type == FREE_COM){
		float dur;
		Eigen::Matrix4f Th = kin->forwardKinematics(qCurr, R_HAND);
		Eigen::VectorXf ph = t2v(Th);
		int iAttNear = task->getLastReachedAttractor()+1;
		Eigen::Vector3f pg = task->getAttractor(iAttNear);
		float dhg = sqrt( std::pow(ph(0) - pg(0),2) + std::pow(ph(1) - pg(1),2) + std::pow(ph(2) - pg(2),2) );		
		dur = dhg / 0.04;
		prim->setDuration(dur); 
	}

	
	//////////////////////////////////////////////////////----------------------------------------------
	std::vector<MovementPrimitiveType> primSeq;

	if(type == DYN_F_C_LRLm){
		int n = 2;
		for(int i = 0; i < n; i++){
			primSeq.push_back(DYN_F_C_L);
			primSeq.push_back(DYN_F_C_R);
		}
		primSeq.push_back(DYN_F_C_L);
	}  
	else if(type == DYN_F_C_RLRm){
		int n = 2;
		for(int i = 0; i < n; i++){
			primSeq.push_back(DYN_F_C_R);
			primSeq.push_back(DYN_F_C_L);
		}
		primSeq.push_back(DYN_F_C_R);
	} 
	else if(type == DYN_B_C_LRLm){
		int n = 2;
		for(int i = 0; i < n; i++){
			primSeq.push_back(DYN_B_C_L);
			primSeq.push_back(DYN_B_C_R);
		}
		primSeq.push_back(DYN_B_C_L);
	}
	else if(type == DYN_B_C_RLRm){
		int n = 2;
		for(int i = 0; i < n; i++){
			primSeq.push_back(DYN_B_C_R);
			primSeq.push_back(DYN_B_C_L);
		}
		primSeq.push_back(DYN_B_C_R);
	}  
	else if(type == DYN_LL_C_LRm){
		int n = 10;
		for(int i = 0; i < n; i++){
			primSeq.push_back(DYN_LL_C_L);
			primSeq.push_back(DYN_LL_C_R);
		}		
	} 
	else if(type == DYN_LR_C_RLm){
		int n = 10;
		for(int i = 0; i < n; i++){
			primSeq.push_back(DYN_LR_C_R);
			primSeq.push_back(DYN_LR_C_L);
		}		
	}
	else if(type == DYN_FDL_C_LRm){
		int n = 2;
		for(int i = 0; i < n; i++){
			primSeq.push_back(DYN_FDL_C_L);
			primSeq.push_back(DYN_FDL_C_R);
		}
		//primSeq.push_back(DYN_FDL_C_L);		
	}
	else if(type == DYN_FDR_C_RLm){
		int n = 2;
		for(int i = 0; i < n; i++){
			primSeq.push_back(DYN_FDR_C_R);
			primSeq.push_back(DYN_FDR_C_L);
		}		
		//primSeq.push_back(DYN_FDR_C_R);
	}
	else if(type == DYN_BDL_C_LRm){
		int n = 2;
		for(int i = 0; i < n; i++){
			primSeq.push_back(DYN_BDL_C_L);
			primSeq.push_back(DYN_BDL_C_R);
		}		
		//primSeq.push_back(DYN_BDL_C_L);
	}
	else if(type == DYN_BDR_C_RLm){
		int n = 2;
		for(int i = 0; i < n; i++){
			primSeq.push_back(DYN_BDR_C_R);
			primSeq.push_back(DYN_BDR_C_L);
		}		
		//primSeq.push_back(DYN_BDR_C_R);
	}  
	else if(type == DYN_FCL_C_LRm){
		int n = 2;
		for(int i = 0; i < n; i++){
			primSeq.push_back(DYN_FCL_C_L);
			primSeq.push_back(DYN_FCL_C_R);
		}		
	}
	else if(type == DYN_FCR_C_RLm){
		int n = 2;
		for(int i = 0; i < n; i++){
			primSeq.push_back(DYN_FCR_C_R);
			primSeq.push_back(DYN_FCR_C_L);
		}		
	}  	      	    
	else primSeq.push_back(type);


	float t = tk;
	bool jnt_lim_flag = false;
	bool collision_flag = false; 
	bool equilibrium_flag = false;
	bool occlusion_flag = false; 	
	Eigen::Matrix4f Thand_init;
	Eigen::VectorXf vhand_init;		 
	float mg_time = 0.0;
	float t_start = simGetSimulationTime();	
	float t_current;	
	int i_cc = 0; // collision checks counter
	
	for(int i = 0; i < primSeq.size(); i++){
		
		prim = new MovementPrimitive((int)primSeq.at(i), qCurr, tk, robot, kin);	
		type = primSeq.at(i);
		duration = prim->getDuration();

		//////////////////////////////////////////
		if(type == FREE_COM) support = R_FOOT;
		if(type == DYN_F_S_L) support = R_FOOT;	
		if(type == DYN_F_C_L) support = R_FOOT;	
		if(type == DYN_F_C_R) support = L_FOOT;	
		if(type == DYN_B_C_L) support = R_FOOT;	
		if(type == DYN_B_C_R) support = L_FOOT;	
		if(type == DYN_LL_C_L) support = R_FOOT;	
		if(type == DYN_LL_C_R) support = L_FOOT;	
		if(type == DYN_LR_C_L) support = R_FOOT;	
		if(type == DYN_LR_C_R) support = L_FOOT;	
		if(type == DYN_FDL_C_L) support = R_FOOT;	
		if(type == DYN_FDL_C_R) support = L_FOOT;	
		if(type == DYN_FDR_C_L) support = R_FOOT;	
		if(type == DYN_FDR_C_R) support = L_FOOT;	
		if(type == DYN_BDL_C_L) support = R_FOOT;	
		if(type == DYN_BDL_C_R) support = L_FOOT;	
		if(type == DYN_BDR_C_L) support = R_FOOT;	
		if(type == DYN_BDR_C_R) support = L_FOOT;	
		if(type == DYN_FCL_C_L) support = R_FOOT;	
		if(type == DYN_FCL_C_R) support = L_FOOT;
		if(type == DYN_FCR_C_L) support = R_FOOT;	
		if(type == DYN_FCR_C_R) support = L_FOOT;

		if(support == R_FOOT) swing = L_FOOT;
		else swing = R_FOOT;		
		//////////////////////////////////////////

		Tsup_w = kin->forwardKinematics(qCurr, support);
		Tswg_w = kin->forwardKinematics(qCurr, swing);	
	
		Configuration qNext(dof);
		qNext = qCurr;

		Thand_init = kin->forwardKinematics(qCurr, R_HAND);
		vhand_init = t2v(Thand_init);
		phand_init = vhand_init.block(0, 0, 3, 1);

		////////////////////////////////////////////////////////////////
		while(t < (tk + prim->getDuration() + 0.025)){
	
			qNext = FirstOrderIntegrator(qNext, t, intStep);

			// Validation...
			jnt_lim_flag = jointLimitsCheck(qNext);
			if(jnt_lim_flag) break; 
			if(i_cc == 3){
				collision_flag = collisionCheck(qNext); 
				if(collision_flag) break;	
				i_cc = 0;
			}
			else i_cc++;	
			 
			motion.insert(std::pair<float, Configuration>(t, qNext));
			t = t + intStep;	

		
			//////////////////////////////////////////////////////////////////////////////////
			//t_current = simGetSimulationTime();	
			//mg_time = t_current - t_start;
			//if(planning_time + mg_time > (time_budget-0.10)){
				//motion.clear();
				//return motion;
			//}
			//////////////////////////////////////////////////////////////////////////////////		
		}
		
		if(jnt_lim_flag || collision_flag || equilibrium_flag || occlusion_flag){
			if(jnt_lim_flag) std::cout << "jnt_lim_flag++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
			if(collision_flag) std::cout << "collision_flag+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
			//motion.clear();
			return motion;
		}

		qCurr = qNext; 
		tk = t;
	
	}
	
	return motion;
}

Configuration MotionGenerator::FirstOrderIntegrator(Configuration& q, float t, float h){
 	
	Eigen::VectorXf k(dof);
	k = f(t, q);
	 
	Configuration qNext(dof);
	qNext = update_q(q, h, k);

	if(taskType == TASK_NAVIGATION){
		qNext.setqjntComponent(20, 1.48 + 3.0*qNext.getqjntComponent(6)/4.0 - 3.0*qNext.getqjntComponent(17)/4.0);
		qNext.setqjntComponent(10, 1.48 + 3.0*qNext.getqjntComponent(17)/4.0 - 3.0*qNext.getqjntComponent(6)/4.0);
	}

	return qNext;	
	
}

Configuration MotionGenerator::RungeKutta4(Configuration& q, float t, float h){
 	
	Configuration q1(dof), q2(dof), q3(dof), q4(dof);
	Eigen::VectorXf k1(dof), k2(dof), k3(dof), k4(dof);
	 
	q1 = q;
	k1 = f(t, q1);
	q2 = update_q(q, h/2, k1);
	k2 = f(t+h/2, q2);
	q3 = update_q(q, h/2, k2);
	k3 = f(t+h/2, q3);
	q4 = update_q(q, h, k3);
	k4 = f(t+h, q4);
	 
	Configuration qNext(dof);
	Eigen::VectorXf kfin(dof);
	kfin = k1 + 2*k2 + 2*k3 + k4;
	qNext = update_q(q, h/6, kfin);
	
	return qNext;	
	
}

Eigen::VectorXf MotionGenerator::f(float t, Configuration& q){
	
	Eigen::Matrix3f Rsup_w = Tsup_w.block<3,3>(0,0);
	Eigen::VectorXf q_dot(dof);

	// ********* LOCOMOTION ***************************************************************************************
	int mzCoM = 0;
	int mzswg = 0;	
	if(prim->getzCoM() != NULL) mzCoM = prim->getzCoM()->getPosition(tk).size();
	if(prim->getzswg() != NULL) mzswg = prim->getzswg()->getPosition(tk).size();
		
	int mL = mzCoM + mzswg;  
	
	Eigen::MatrixXf JL(mL, dof);
	Eigen::VectorXf tLc(mL);
	Eigen::VectorXf tLd(mL);
	Eigen::VectorXf tLd_dot(mL);
	Eigen::MatrixXf I = Eigen::MatrixXf::Identity(dof, dof);
	Eigen::MatrixXf KL(mL, mL); 
	JL.setZero();
	tLc.setZero();
	tLd.setZero();
	tLd_dot.setZero();
	KL = Eigen::MatrixXf::Identity(mL, mL);
	
	int is = 0;	
	 	
	if(prim->getzCoM() != NULL){		
		Eigen::MatrixXf JCoM = kin->getJacobian(q, support, COM);
		Eigen::MatrixXf JTorso = kin->getJacobian(q, support, TORSO);
		
		JCoM = Rsup_w * JCoM;
		JTorso.block(0,0,3,dof) = Rsup_w * JTorso.block(0,0,3,dof);
		JTorso.block(3,0,3,dof) = Rsup_w * JTorso.block(3,0,3,dof);

		Eigen::VectorXf tc_CoM(6);
		Eigen::Vector3f pc_CoM = q.getqCoMPosition();
		Eigen::Vector3f oc_CoM = q.getqCoMOrientation();
		tc_CoM.block(0, 0, 3, 1) = pc_CoM;
		tc_CoM.block(3, 0, 3, 1) = oc_CoM;  
		//std::cout << "tc_CoM:: " << tc_CoM.transpose() << std::endl;
		
		Eigen::VectorXf td_CoM = prim->getzCoM()->getPosition(t);
		Eigen::VectorXf td_dot_CoM = prim->getzCoM()->getVelocity(t);
		
		Eigen::VectorXf constComp = prim->getzCoM()->getConstrainedComponents();
		for(int i = 0; i < 3; i++){
			if(constComp(i) == 1){
				JL.block(is, 0, 1, dof) = JCoM.block(i, 0, 1, dof);
				tLc(is) = tc_CoM(i);
				tLd(is) = td_CoM(is);
				tLd_dot(is) = td_dot_CoM(is);
				is++;		 
			}
		}
		for(int i = 3; i < 6; i++){
			if(constComp(i) == 1){
				JL.block(is, 0, 1, dof) = JTorso.block(i, 0, 1, dof); 
				tLc(is) = tc_CoM(i);
				tLd(is) = td_CoM(is);
				tLd_dot(is) = td_dot_CoM(is);
				is++;		 
			}
		}	

		Eigen::VectorXf KCoM = prim->getzCoM()->getGains();		
		for(int i = 0; i < mzCoM; i++) KL(i,i) = KCoM(i) * KL(i,i);			
	}
  
	if(prim->getzswg() != NULL){
		Eigen::MatrixXf Jswg = kin->getJacobian(q, support, swing);
		
		Jswg.block(0,0,3,dof) = Rsup_w * Jswg.block(0,0,3,dof);
		Jswg.block(3,0,3,dof) = Rsup_w * Jswg.block(3,0,3,dof);
		
		Tswg_w = kin->forwardKinematics(q, swing);
		Eigen::VectorXf tc_swg(6);
		tc_swg = t2v(Tswg_w);
		Eigen::VectorXf td_swg = prim->getzswg()->getPosition(t);
		Eigen::VectorXf td_dot_swg = prim->getzswg()->getVelocity(t);
		for(int i = is; i < is + mzswg; i++) JL.block(i, 0, 1, dof) = Jswg.block(i - is, 0, 1, dof);
		tLc.block(is, 0, mzswg, 1) = tc_swg;		
		tLd.block(is, 0, mzswg, 1) = td_swg;
		tLd_dot.block(is, 0, mzswg, 1) = td_dot_swg;

		Eigen::VectorXf Kswg(mzswg);
		//Kswg << 2, 2, 50, 1, 1, 1; 
		Kswg << 5, 5, 50, 0.1, 0.1, 0.1; 
		for(int i = is; i < is + mzswg; i++) KL(i,i) = Kswg(i - is) * KL(i,i);

		is = is + mzswg;
	}	

	Eigen::VectorXf eL = tLd - tLc;	
	//std::cout << "eL :: " << eL.transpose() << std::endl;
	//std::cout << "eL size:: " << eL.size() << std::endl;
	//std::cout << "tLd :: " << tLd.transpose() << std::endl;
	//std::cout << "tLc :: " << tLc.transpose() << std::endl;
	
	if(mL == 12){
		//std::cout << "tLd :: " << tLd(3) << " " << tLd(4) << " " << tLd(5) << std::endl;
		//std::cout << "tLc :: " << tLc(3) << " " << tLc(4) << " " << tLc(5) << std::endl;

		eL(3) = angleSignedDistance(tLd(3), tLc(3));
		eL(4) = angleSignedDistance(tLd(4), tLc(4));
		eL(5) = angleSignedDistance(tLd(5), tLc(5));
		eL(9) = angleSignedDistance(tLd(9), tLc(9));
		eL(10) = angleSignedDistance(tLd(10), tLc(10));
		eL(11) = angleSignedDistance(tLd(11), tLc(11));

		std::cout << "eL :: " << eL.transpose() << std::endl;
		/*
		eL(3) = std::min(fabs(tLd(3) - tLc(3)), fabs(2*M_PI - fabs(tLd(3) - tLc(3))));
		eL(4) = std::min(fabs(tLd(4) - tLc(4)), fabs(2*M_PI - fabs(tLd(4) - tLc(4))));
		eL(5) = std::min(fabs(tLd(5) - tLc(5)), fabs(2*M_PI - fabs(tLd(5) - tLc(5))));
		eL(9) = std::min(fabs(tLd(9) - tLc(9)), fabs(2*M_PI - fabs(tLd(9) - tLc(9))));
		eL(10) = std::min(fabs(tLd(10) - tLc(10)), fabs(2*M_PI - fabs(tLd(10) - tLc(10))));
		eL(11) = std::min(fabs(tLd(11) - tLc(11)), fabs(2*M_PI - fabs(tLd(11) - tLc(11))));
		*/
	}
	else if(mL == 11){
		std::cout << "mL = 11" << std::endl;
		eL(3) = angleSignedDistance(tLd(3), tLc(3));
		eL(4) = angleSignedDistance(tLd(4), tLc(4));

		eL(8) = angleSignedDistance(tLd(8), tLc(8));		
		eL(9) = angleSignedDistance(tLd(9), tLc(9));
		eL(10) = angleSignedDistance(tLd(10), tLc(10));

		//std::cout << "eL :: " << eL.transpose() << std::endl;
	}	
	else{
		eL(6) = angleSignedDistance(tLd(6), tLc(6));
		eL(7) = angleSignedDistance(tLd(7), tLc(7));
		eL(8) = angleSignedDistance(tLd(8), tLc(8));
		/*
		if(prim->getzswg() != NULL){
			for(int i = 3; i > 0; i--) eL(mL-i) = std::min(fabs(tLd(mL-i) - tLc(mL-i)), fabs(2*M_PI - fabs(tLd(mL-i) - tLc(mL-i))));
		}
		*/
	}	

	/*	 
	if(prim->getzswg() != NULL){
		for(int i = 3; i > 0; i--) eL(mL-i) = std::min(fabs(tLd(mL-i) - tLc(mL-i)), fabs(2*M_PI - fabs(tLd(mL-i) - tLc(mL-i))));
	}
	*/
	//std::cout << "eL :: " << eL.transpose() << std::endl;
	 

	Eigen::VectorXf xL_dot = tLd_dot + KL * eL;
	Eigen::VectorXf x1_dot = xL_dot;		
	Eigen::MatrixXf J1 = JL;

	Eigen::VectorXf qLim1_dot(dof);
	Eigen::VectorXf qLim2_dot(dof);
	Eigen::VectorXf omega_rand(dof);
	omega_rand.setZero();
	omega_rand = computeRandomVelocities();
	Eigen::VectorXf omega_FREE_COM(dof);
	omega_FREE_COM.setZero();
	if(type == FREE_COM) omega_FREE_COM = computeBalanceVelocities(q);
	Eigen::VectorXf omega(dof);
	omega = omega_FREE_COM + omega_rand;
	//omega_rand.setZero();
	omega.setZero();

	if(taskType == TASK_NAVIGATION){ 
		Eigen::MatrixXf J1p = J1;
		for(int j = 0; j < dof; j++){
			if(j==0||j==1||j==4||j==5||j==10||j==11||j==12||j==15||j==16||j==20||j==21||j==22)
				for(int i = 0; i < J1p.rows(); i++) J1p(i, j) = 0.0;
		}
		Eigen::MatrixXf J1g = pinv(J1p);
		Eigen::VectorXf q1_dot = J1g * x1_dot;
		Eigen::MatrixXf P1 = I - J1g*J1;
		float alpha = 0.5;
		qLim1_dot = - alpha * computeGradHrange(q);	
		qLim1_dot = normalize(qLim1_dot, q1_dot.norm());
		qLim2_dot = computeJLA(q, q1_dot, P1);	 
		//q_dot = q1_dot + P1 * (omega_rand); // without JLA
		//q_dot = q1_dot + P1 * (qLim1_dot + omega); // JLA Liegeois
		q_dot = q1_dot + P1 * (qLim2_dot + omega);  //JLA Marey & Chaumette
		return q_dot;
	}


	int mzvisual = 0;
	if(taskType == TASK_TRAJECTORY_VISUAL || taskType == TASK_PATH_VISUAL) mzvisual = task->getPosition(t).size();   
	int mV = std::max(0, mzvisual - 1);  
	Eigen::MatrixXf JV(mV, dof);
	Eigen::VectorXf tVc(mV);
	Eigen::VectorXf tVd(mV);
	Eigen::MatrixXf KV(mV, mV); 
	JV.setZero();
	tVc.setZero();
	tVd.setZero();
	KV = Eigen::MatrixXf::Identity(mV, mV);
	Eigen::VectorXf tVd_dot(mV);

	Eigen::Vector3f pCoM_w_k = qCurr.getqCoMPosition();
	Eigen::Vector3f td_hand_target = task->getAttractor(task->getLastReachedAttractor()+1);		
	float d = sqrt(std::pow(pCoM_w_k(0)-td_hand_target(0),2)+std::pow(pCoM_w_k(1)-td_hand_target(1),2)+std::pow(pCoM_w_k(2)-td_hand_target(2),2));

	int mzhand = 0;
	if(taskType == TASK_SETPOINT && d < D2) mzhand = task->getPosition(t).size();
	else if(taskType == TASK_TRAJECTORY || taskType == TASK_PATH) mzhand = task->getPosition(t).size(); 			
	int mM = mzhand;  
	Eigen::MatrixXf JM(mM, dof);
	Eigen::VectorXf tMc(mM);
	Eigen::VectorXf tMd(mM);
	Eigen::MatrixXf KM(mM, mM); 
	JM.setZero();
	tMc.setZero();
	tMd.setZero();
	KM = Eigen::MatrixXf::Identity(mM, mM);
	Eigen::VectorXf tMd_dot(mM);

	switch(taskType){
		case TASK_SETPOINT: {
			if(mzhand != 0){
				is = 0;
				Eigen::MatrixXf Jhand = kin->getJacobian(q, support, R_HAND);
				Jhand.block(0,0,3,dof) = Rsup_w * Jhand.block(0,0,3,dof);
				Jhand.block(3,0,3,dof) = Rsup_w * Jhand.block(3,0,3,dof);
				Thand_w = kin->forwardKinematics(q, R_HAND);
				Eigen::VectorXf tc_hand(6); 
				tc_hand = t2v(Thand_w);
		
				Eigen::Vector3f td_hand; // local target
				Eigen::MatrixXf Thand_w_k = kin->forwardKinematics(qCurr, R_HAND);
				Eigen::VectorXf vhand_w_k = t2v(Thand_w_k);
				Eigen::Vector3f pCoM_w_k = qCurr.getqCoMPosition();

				if(mzCoM == 0) td_hand = td_hand_target;			
				else{	
					Eigen::VectorXf td_CoM_fin = prim->getzCoM()->getPosition(tk + duration);
					Eigen::Vector3f pCoM_w = qCurr.getqCoMPosition();
					Eigen::Vector3f oCoM_w = qCurr.getqCoMOrientation();
					Eigen::VectorXf vCoM_w(6);
					if(mzCoM > 2){
						vCoM_w.block(0, 0, 3, 1) = td_CoM_fin.block(0, 0, 3, 1);
						vCoM_w.block(3, 0, 3, 1) = oCoM_w;
					}
					else{
						vCoM_w.block(0, 0, 2, 1) = td_CoM_fin.block(0, 0, 2, 1);  
						vCoM_w(2) = pCoM_w(2);									  
						vCoM_w.block(3, 0, 3, 1) = oCoM_w;						  
					}
					Eigen::Matrix4f TCoM_w = v2t(vCoM_w);
					Eigen::Matrix4f Tw_CoM = TCoM_w.inverse();
 
					Eigen::Vector4f phom_w;
					phom_w.block(0, 0, 3, 1) = td_hand_target;
					phom_w(3) = 1.0;

					Eigen::Vector4f phom_CoM = Tw_CoM * phom_w;
					Eigen::Vector3f p_CoM;
					p_CoM = phom_CoM.block(0, 0, 3, 1);
					float d = sqrt(std::pow(vCoM_w(0)-td_hand_target(0),2) + std::pow(vCoM_w(1)-td_hand_target(1),2) + std::pow(vCoM_w(2)-td_hand_target(2),2));
			
					Eigen::Vector3f d_star;
					d_star << 0.12, -0.16, -0.002;				

					//float beta = 1.0 / (1.0 + exp( -8.0 * ((d - D0) / (D1 - D0)) + 3.0));
					float d1 = 0.3;
					float d0 = 0.15;
					float beta = 1.0 / (1.0 + exp( -8.0 * ((d - d0) / (d1 - d0)) + 3.0));
		
					Eigen::Vector3f d_bar = beta * d_star + (1.0 - beta) * p_CoM;
					Eigen::Vector3f d_k = vhand_w_k.block(0, 0, 3, 1) - pCoM_w_k;
					float alpha = 0.8;
					Eigen::MatrixXf A = Eigen::MatrixXf::Identity(3,3);
					A = alpha * A;
					Eigen::Vector3f pCoM_fin = vCoM_w.block(0, 0, 3, 1);
					td_hand = pCoM_fin + d_k + A * (d_bar - d_k);		
				}		
		
				Eigen::VectorXf td_dot_hand = task->getVelocity(t);
				for(int i = is; i < is + mzhand; i++) JM.block(i, 0, 1, dof) = Jhand.block(i - is, 0, 1, dof);
				tMc.block(is, 0, mzhand, 1) = tc_hand.block(0, 0, mzhand, 1);		
				tMd.block(is, 0, mzhand, 1) = td_hand;
				tMd_dot = td_dot_hand;

				KM(0,0) = fabs( (tMd(0) - phand_init(0)) ) / duration;
				KM(1,1) = fabs( (tMd(1) - phand_init(1)) ) / duration;
				KM(2,2) = fabs( (tMd(2) - phand_init(2)) ) / duration;

				is = is + mzhand;		 
			} 	
			break;
		}
			
		case TASK_TRAJECTORY: {
			is = 0;
			Eigen::MatrixXf Jhand = kin->getJacobian(q, support, R_HAND);
			Jhand.block(0,0,3,dof) = Rsup_w * Jhand.block(0,0,3,dof);
			Jhand.block(3,0,3,dof) = Rsup_w * Jhand.block(3,0,3,dof);
			Thand_w = kin->forwardKinematics(q, R_HAND);
			Eigen::VectorXf tc_hand(6); 
			tc_hand = t2v(Thand_w);

			Eigen::Vector3f td_hand;
			Eigen::Vector3f td_dot_hand;
			//if(prim->getzswg()->isInFootTrajHold(t)){
			//	td_hand = tc_hand.block(is, 0, mzhand, 1);
			//	td_dot_hand.setZero(); 
			//}
			//else{
				td_hand = task->getPosition(t);
				td_dot_hand = task->getVelocity(t);
			//}

			for(int i = is; i < is + mzhand; i++) JM.block(i, 0, 1, dof) = Jhand.block(i - is, 0, 1, dof);
			tMc.block(is, 0, mzhand, 1) = tc_hand.block(0, 0, mzhand, 1);		
			tMd.block(is, 0, mzhand, 1) = td_hand;
			tMd_dot.block(is, 0, mzhand, 1) = td_dot_hand;

			KM = 2.0 * KM;
			
			break;
 		}
			
		case TASK_PATH: {
			is = 0;
			Eigen::MatrixXf Jhand = kin->getJacobian(q, support, R_HAND);
			Jhand.block(0,0,3,dof) = Rsup_w * Jhand.block(0,0,3,dof);
			Jhand.block(3,0,3,dof) = Rsup_w * Jhand.block(3,0,3,dof);
			Thand_w = kin->forwardKinematics(q, R_HAND);
			Eigen::VectorXf tc_hand(6); 
			tc_hand = t2v(Thand_w);
			
			Eigen::Vector3f td_hand; // local target
			Eigen::MatrixXf Thand_w_k = kin->forwardKinematics(qCurr, R_HAND);
			Eigen::VectorXf vhand_w_k = t2v(Thand_w_k);
			Eigen::Vector3f pCoM_w_k = qCurr.getqCoMPosition();

			Eigen::Vector3f phand_w_k = vhand_w_k.block(0, 0, 3, 1);

			float si = task->computeClosestPoint(phand_w_k, -1.0);
			float sf;
			if(mzCoM == 0) td_hand = td_hand_target;			
			else{	
				Eigen::VectorXf td_CoM_fin = prim->getzCoM()->getPosition(tk + duration);
				Eigen::Vector3f pCoM_w = qCurr.getqCoMPosition();
				Eigen::Vector3f oCoM_w = qCurr.getqCoMOrientation();
				Eigen::VectorXf vCoM_w(6);
				if(mzCoM > 2){
					vCoM_w.block(0, 0, 3, 1) = td_CoM_fin.block(0, 0, 3, 1);
					vCoM_w.block(3, 0, 3, 1) = oCoM_w;
				}
				else{
					vCoM_w.block(0, 0, 2, 1) = td_CoM_fin.block(0, 0, 2, 1);  
					vCoM_w(2) = pCoM_w(2);									  
					vCoM_w.block(3, 0, 3, 1) = oCoM_w;						  
				}
		
				Eigen::Vector3f d_star;
				d_star << 0.12, -0.16, -0.002;	
	
				Eigen::Vector3f d_bar = d_star;
				Eigen::Vector3f d_k = vhand_w_k.block(0, 0, 3, 1) - pCoM_w_k;
				float alpha = 0.8;
				Eigen::MatrixXf A = Eigen::MatrixXf::Identity(3,3);
				A = alpha * A;
				Eigen::Vector3f pCoM_fin = vCoM_w.block(0, 0, 3, 1);
				td_hand = pCoM_fin + d_k + A * (d_bar - d_k);	
			}		
			
			Eigen::Vector3f pi = task->getPosition(si);
			sf = task->computeClosestPoint(td_hand, si);

			float b = (t-tk) / duration;		
			float s = b*sf + (1.0 - b)*si;
			td_hand = task->getPosition(s);

			Eigen::VectorXf td_dot_hand = ((sf-si)/duration) * task->getVelocity(s);		
			for(int i = is; i < is + mzhand; i++) JM.block(i, 0, 1, dof) = Jhand.block(i - is, 0, 1, dof);
			tMc.block(is, 0, mzhand, 1) = tc_hand.block(0, 0, mzhand, 1);		
			tMd.block(is, 0, mzhand, 1) = td_hand;
			tMd_dot = td_dot_hand;

			KM = 5.0 * KM;

			break;
		}	
			
		case TASK_TRAJECTORY_VISUAL: {
			is = 0;
			Eigen::Vector3f p_w = task->getPosition(t);  
			Eigen::Vector3f pdot_w = task->getVelocity(t); 
	
			Eigen::MatrixXf Tc_w = kin->forwardKinematics(q, TOP_CAMERA);
			Eigen::MatrixXf Tw_c = Tc_w.inverse();		
			Eigen::Matrix3f Rc_w = Tc_w.block<3,3>(0,0);
			Eigen::Matrix3f Rw_c = Rc_w.inverse();

			Eigen::VectorXf p_w_hom(4);
			p_w_hom << p_w(0), p_w(1), p_w(2), 1.0;	
			Eigen::VectorXf p_c_hom(4);
			p_c_hom << Tw_c * p_w_hom;
			Eigen::Vector3f r_c;
			float lambda = 0.2;
			float u = lambda * p_c_hom(0)/p_c_hom(2);
			float v = lambda * p_c_hom(1)/p_c_hom(2);
			r_c << u, v, lambda;
				
			float z = p_c_hom(2);
			Eigen::MatrixXf Jp(2,6);
			Jp(0,0) = -lambda/z;
			Jp(0,1) = 0.0;
			Jp(0,2) = u/z;
			Jp(0,3) = (u*v)/z;
			Jp(0,4) = -(lambda + (std::pow(u,2)/lambda));
			Jp(0,5) = v;
			Jp(1,0) = 0.0;
			Jp(1,1) = -(lambda/z);
			Jp(1,2) = v/z;
			Jp(1,3) = lambda + (std::pow(v,2)/lambda);
			Jp(1,4) = -((u*v)/z);
			Jp(1,5) = -u;
	
			Eigen::MatrixXf Jc_sup = kin->getJacobian(q, support, TOP_CAMERA);
			Eigen::MatrixXf Jc_w(Jc_sup.rows(), Jc_sup.cols());
			Jc_w.block(0,0,3,dof) = Rsup_w * Jc_sup.block(0,0,3,dof);
			Jc_w.block(3,0,3,dof) = Rsup_w * Jc_sup.block(3,0,3,dof);
								
			Eigen::MatrixXf Jc_c(Jc_w.rows(), Jc_w.cols());
			Jc_c.block(0,0,3,dof) = Rw_c * Jc_w.block(0,0,3,dof);
			Jc_c.block(3,0,3,dof) = Rw_c * Jc_w.block(3,0,3,dof);
	
			JV = Jp * Jc_c;
			tVc(0) = u;
			tVc(1) = v;
			tVd.setZero();
			Eigen::Vector3f pdot_c = Rw_c * pdot_w;
			Eigen::VectorXf pdot_c_full(6);
			pdot_c_full << pdot_c(0), pdot_c(1), pdot_c(2), 0.0, 0.0, 0.0;
			tVd_dot = Jp * pdot_c_full;  

			KV = 50.0 * KV;
			 
			break;
		}
			
		case TASK_PATH_VISUAL: {

			is = 0;
			Eigen::Vector3f p_w;
			Eigen::Vector3f pdot_w;
			
			Eigen::Vector3f pc_des; 
			Eigen::MatrixXf Tc_w_k = kin->forwardKinematics(qCurr, TOP_CAMERA);
			Eigen::VectorXf vc_w_k = t2v(Tc_w_k);
			Eigen::Vector3f pCoM_w_k = qCurr.getqCoMPosition();
			Eigen::Vector3f pc_w_k = vc_w_k.block(0, 0, 3, 1);

			Eigen::Vector3f pCoM_fin, oCoM_fin; 

			float si = task->computeClosestPoint(vc_w_k, 0.0);
			float sf;
			if(mzCoM == 0) sf = 1.0;
			else{	
				Eigen::VectorXf td_CoM_fin = prim->getzCoM()->getPosition(tk + duration);
				Eigen::Vector3f pCoM_w = qCurr.getqCoMPosition();
				Eigen::Vector3f oCoM_w = qCurr.getqCoMOrientation();
				Eigen::VectorXf vCoM_w(6);
				if(mzCoM > 2){
					vCoM_w.block(0, 0, 3, 1) = td_CoM_fin.block(0, 0, 3, 1);
					vCoM_w.block(3, 0, 3, 1) = oCoM_w;
				}
				else{
					vCoM_w.block(0, 0, 2, 1) = td_CoM_fin.block(0, 0, 2, 1);  
					vCoM_w(2) = pCoM_w(2);									  
					vCoM_w.block(3, 0, 3, 1) = oCoM_w;						  
				}
		
				Eigen::Vector3f d_star;
				d_star << 0.037, -0.036, 0.226;
				Eigen::Vector3f d_k = vc_w_k.block(0, 0, 3, 1) - pCoM_w_k;
				float alpha = 0.8;
				Eigen::MatrixXf A = Eigen::MatrixXf::Identity(3,3);
				A = alpha * A;
				pCoM_fin = vCoM_w.block(0, 0, 3, 1);
				oCoM_fin = vCoM_w.block(3, 0, 3, 1);
				pc_des = pCoM_fin + d_k + A * (d_star - d_k);

				float s = si;
				float s_inc = 0.01;
				p_w = task->getPosition(s);		
				float dist = std::sqrt(std::pow(p_w(0)-pc_des(0),2) + std::pow(p_w(1)-pc_des(1),2) + std::pow(p_w(2)-pc_des(2),2));	
				float l = 0.0;
				sf = s;
				s = s + s_inc;
				float d_bar = 1.0; //0.8;
				float l_bar = 0.1;
				bool occ = false;
				Eigen::Vector3f p_w_prev;	
				while(s <= 1.0 && dist < d_bar && l < l_bar && !occ){
					p_w_prev = task->getPosition(s - s_inc);							
					p_w = task->getPosition(s);		
					dist = std::sqrt(std::pow(p_w(0)-pc_des(0), 2) + std::pow(p_w(1)-pc_des(1), 2) + std::pow(p_w(2)-pc_des(2), 2));	
					l = l + std::sqrt(std::pow(p_w(0)-p_w_prev(0),2) + std::pow(p_w(1)-p_w_prev(1),2) + std::pow(p_w(2)-p_w_prev(2),2));
					occ = occlusionCheckPoint(qCurr, p_w);
					sf = s;
					s = s + s_inc;
				}
				sf = sf - s_inc;
			}  	

			Eigen::Vector3f pi = task->getPosition(si);
			Eigen::Vector3f pf = task->getPosition(sf);
				
			float b = (t-tk) / duration;		
			float s = b*sf + (1.0 - b)*si;

			p_w = task->getPosition(s);  
			pdot_w  = ((sf-si)/duration) * task->getVelocity(s);		
		
			Eigen::MatrixXf Tc_w = kin->forwardKinematics(q, TOP_CAMERA);
			Eigen::MatrixXf Tw_c = Tc_w.inverse();		
			Eigen::Matrix3f Rc_w = Tc_w.block<3,3>(0,0);
			Eigen::Matrix3f Rw_c = Rc_w.inverse();

			Eigen::VectorXf p_w_hom(4);
			p_w_hom << p_w(0), p_w(1), p_w(2), 1.0;	
			Eigen::VectorXf p_c_hom(4);
			p_c_hom << Tw_c * p_w_hom;
			Eigen::Vector3f r_c;
			float lambda = 0.2;
			float u = lambda * p_c_hom(0)/p_c_hom(2);
			float v = lambda * p_c_hom(1)/p_c_hom(2);
			r_c << u, v, lambda;
				
			float z = p_c_hom(2);
			Eigen::MatrixXf Jp(2,6);
			Jp(0,0) = -lambda/z;
			Jp(0,1) = 0.0;
			Jp(0,2) = u/z;
			Jp(0,3) = (u*v)/z;
			Jp(0,4) = -(lambda + (std::pow(u,2)/lambda));
			Jp(0,5) = v;
			Jp(1,0) = 0.0;
			Jp(1,1) = -(lambda/z);
			Jp(1,2) = v/z;
			Jp(1,3) = lambda + (std::pow(v,2)/lambda);
			Jp(1,4) = -((u*v)/z);
			Jp(1,5) = -u;

			Eigen::MatrixXf Jc_sup = kin->getJacobian(q, support, TOP_CAMERA);
			Eigen::MatrixXf Jc_w(Jc_sup.rows(), Jc_sup.cols());
			Jc_w.block(0,0,3,dof) = Rsup_w * Jc_sup.block(0,0,3,dof);
			Jc_w.block(3,0,3,dof) = Rsup_w * Jc_sup.block(3,0,3,dof);
								
			Eigen::MatrixXf Jc_c(Jc_w.rows(), Jc_w.cols());
			Jc_c.block(0,0,3,dof) = Rw_c * Jc_w.block(0,0,3,dof);
			Jc_c.block(3,0,3,dof) = Rw_c * Jc_w.block(3,0,3,dof);
	
			JV = Jp * Jc_c;
			tVc(0) = u;
			tVc(1) = v;
			tVd.setZero();
			Eigen::Vector3f pdot_c = Rw_c * pdot_w;
		
			Eigen::VectorXf pdot_c_full(6);
			pdot_c_full << pdot_c(0), pdot_c(1), pdot_c(2), 0.0, 0.0, 0.0;
			tVd_dot = Jp * pdot_c_full;  
				
			KV = 50.0 * KV;
					 
			break;
		}
	
	} // close switch

	Eigen::VectorXf x2_dot;
	Eigen::MatrixXf J2;

	if(mzvisual != 0){  // visual
		Eigen::VectorXf eV = tVd - tVc;	
		Eigen::VectorXf xV_dot(mV);
		xV_dot = tVd_dot + KV * eV; 
		x2_dot = xV_dot;
		J2 = JV;
	}

	if(mzhand != 0){  // set-point 			
		Eigen::VectorXf eM = tMd - tMc;	
		Eigen::VectorXf xM_dot(mM);
		if(taskType == TASK_SETPOINT) xM_dot = tMd_dot + KM * sign(eM);
		else xM_dot = tMd_dot + KM * eM; 
		x2_dot = xM_dot;
		J2 = JM;
	}

	if(mzCoM != 0){
		if(mzhand != 0){		
			Eigen::MatrixXf J1p = J1;
			for(int j = 0; j < dof; j++){
				if(j==0||j==1||j==4||j==5||j==10||j==11||j==12||j==15||j==16||j==20||j==21||j==22)
					for(int i = 0; i < J1p.rows(); i++) J1p(i, j) = 0.0;
			}
			Eigen::MatrixXf J1g = pinv(J1p);	
			Eigen::VectorXf q1_dot = J1g * x1_dot;
			Eigen::MatrixXf P1 = I - J1g*J1;
			Eigen::MatrixXf B = J2*P1;
			Eigen::MatrixXf Bp = B;
			for(int j = 0; j < dof; j++){
				if(j==0||j==1||j==4||j==5||j==10||j==11||j==12)
					for(int i = 0; i < Bp.rows(); i++) Bp(i, j) = 0.0;
			}
			Eigen::MatrixXf Bg = pinv(Bp);
			Eigen::MatrixXf P2 = I - Bg*B;
			Eigen::VectorXf qT_dot(dof);
			qT_dot = q1_dot + P1*Bg * (x2_dot - J2*q1_dot);	
			Eigen::MatrixXf PA2 = P1*P2;
			float alpha = 0.5;
			qLim1_dot = - alpha * computeGradHrange(q);	
			qLim1_dot = normalize(qLim1_dot, qT_dot.norm());
			qLim2_dot = computeJLA(q, qT_dot, PA2);
			
			//q_dot = qT_dot + PA2 * (omega_rand); // without JLA
			//q_dot = qT_dot + PA2 * (qLim1_dot + omega); // JLA Liegeois
			q_dot = qT_dot + PA2 * (qLim2_dot + omega);  //JLA Marey & Chaumette
			if(taskType == TASK_TRAJECTORY || taskType == TASK_PATH) q_dot = qT_dot;
		}
		else if(mzvisual != 0){
			Eigen::MatrixXf J1p = J1;
			for(int j = 0; j < dof; j++){
				if(j== 0||j==1||j==4||j==5||j==10||j==11||j==12||j==15||j==16||j==20||j==21||j==22)
					for(int i = 0; i < J1p.rows(); i++) J1p(i, j) = 0.0;
			}
			Eigen::MatrixXf J1g = pinv(J1p);	
			Eigen::VectorXf q1_dot = J1g * x1_dot;
			Eigen::MatrixXf P1 = I - J1g*J1;
			Eigen::MatrixXf B = J2*P1;
			Eigen::MatrixXf Bp = B;
			Eigen::MatrixXf Bg = pinv(Bp);
			Eigen::MatrixXf P2 = I - Bg*B;
			Eigen::VectorXf qT_dot(dof);
			qT_dot = q1_dot + P1*Bg * (x2_dot - J2*q1_dot);	
			Eigen::MatrixXf PA2 = P1*P2;
			float alpha = 0.5;
			qLim1_dot = - alpha * computeGradHrange(q);	
			qLim1_dot = normalize(qLim1_dot, qT_dot.norm());
			qLim2_dot = computeJLA(q, qT_dot, PA2);
			
			//q_dot = qT_dot + PA2 * (omega_rand); // without JLA
			//q_dot = qT_dot + PA2 * (qLim1_dot + omega); // JLA Liegeois
			q_dot = qT_dot + PA2 * (qLim2_dot + omega);  //JLA Marey & Chaumette
			
		}
		else{
			Eigen::MatrixXf J1p = J1;
			for(int j = 0; j < dof; j++){
				if(j==0||j==1||j==4||j==5||j==10||j==11||j==12||j==15||j==16||j==20||j==21||j==22)
					for(int i = 0; i < J1p.rows(); i++) J1p(i, j) = 0.0;
			}

			/*
			Eigen::MatrixXf J1g = pinv(J1p);
			Eigen::VectorXf q1_dot = J1g * x1_dot;
			Eigen::MatrixXf P1 = I - J1g*J1;
			Eigen::VectorXf qT_dot(dof);
			qT_dot = q1_dot;  

			//std::cout << "qT_dot :: " << qT_dot.transpose() << std::endl;
			//std::cout << "J1g :: " << J1g.block(2,0, 1, 12) << std::endl;
			Eigen::MatrixXf A = (J1p*J1p.transpose());
			std::cout << "J1p size:: " << J1p.rows() << " " << J1p.cols() << std::endl;						
			std::cout << "A det:: " << A.determinant() << std::endl;
			Eigen::MatrixXf Jcom = J1p.block(0,0,6,23);	 					
			Eigen::MatrixXf Jswg = J1p.block(6,0,6,23);		
			//std::cout << "Jcom size:: " << Jcom.rows() << " " << Jcom.cols() << std::endl;	
			//std::cout << "Jswg size:: " << Jswg.rows() << " " << Jswg.cols() << std::endl;	
			Eigen::FullPivLU<Eigen::MatrixXf> lu_decompJcom(Jcom);
			float rankJcom = lu_decompJcom.rank();	
			std::cout << "Jcom rank:: " << rankJcom << std::endl;									
			Eigen::FullPivLU<Eigen::MatrixXf> lu_decompJswg(Jswg);
			float rankJswg = lu_decompJswg.rank();	
			std::cout << "Jswg rank:: " << rankJswg << std::endl;	
			Eigen::FullPivLU<Eigen::MatrixXf> lu_decompJ1p(J1p);
			float rankJ1p = lu_decompJ1p.rank();	
			std::cout << "J1p rank:: " << rankJ1p << std::endl;										

			Eigen::MatrixXf Acom = (Jcom*Jcom.transpose());						
			Eigen::MatrixXf Aswg = (Jswg*Jswg.transpose());						
			std::cout << "Acom det:: " << Acom.determinant() << std::endl;
			std::cout << "Aswg det:: " << Aswg.determinant() << std::endl;
			
			Eigen::MatrixXf PA2 = P1;  
			float alpha = 0.5;
			qLim1_dot = - alpha * computeGradHrange(q);	
			qLim1_dot = normalize(qLim1_dot, qT_dot.norm());
			qLim2_dot = computeJLA(q, qT_dot, PA2);
			
			//q_dot = qT_dot + PA2 * (omega_rand); // without JLA
			//q_dot = qT_dot + PA2 * (qLim1_dot + omega); // JLA Liegeois
			q_dot = qT_dot + PA2 * (qLim2_dot + omega);  //JLA Marey & Chaumette
			*/


			/*
			Eigen::MatrixXf Jcom = J1p.block(0,0,6,23);	 					
			Eigen::MatrixXf Jswg = J1p.block(6,0,6,23);		
			Eigen::VectorXf xcom_dot = x1_dot.block(0,0,6,1);	 					
			Eigen::VectorXf xswg_dot = x1_dot.block(6,0,6,1);

			Eigen::MatrixXf Jcomg = pinv(Jcom);
			Eigen::VectorXf qcom_dot = Jcomg * xcom_dot;
			Eigen::MatrixXf Pcom = I - Jcomg*Jcom;
			Eigen::MatrixXf B = Jswg*Pcom;
			Eigen::MatrixXf Bg = pinv(B);
			//Eigen::MatrixXf Jswgg = pinv(Jswg);

			Eigen::FullPivLU<Eigen::MatrixXf> lu_decompPcom(Pcom);
			float rankPcom = lu_decompPcom.rank();	
			std::cout << "Pcom rank:: " << rankPcom << std::endl;		
			Eigen::FullPivLU<Eigen::MatrixXf> lu_decompB(B);
			float rankB = lu_decompB.rank();	
			std::cout << "B rank:: " << rankB << std::endl;		

			q_dot = qcom_dot + Pcom*Bg*(xswg_dot - Jswg*qcom_dot);
			*/
			
			Eigen::MatrixXf J1g = pinv(J1p);
			q_dot = J1g * x1_dot;
					
		}
	}
	else{ // FREE_COM	
		Eigen::MatrixXf J1p = pinv(J1);	
		Eigen::VectorXf q1_dot = J1p * x1_dot;
		Eigen::MatrixXf P1 = I - J1p*J1;
		Eigen::MatrixXf B = J2*P1;
		Eigen::MatrixXf Bp = pinv(B);
		Eigen::MatrixXf P2 = I - Bp*B;
		Eigen::VectorXf qT_dot(dof);
		qT_dot = q1_dot + P1*Bp * (x2_dot - J2*q1_dot);	
		Eigen::MatrixXf PA2 = P1*P2;
		float alpha = 0.5;
		qLim1_dot = - alpha * computeGradHrange(q);	
		qLim1_dot = normalize(qLim1_dot, qT_dot.norm());
		qLim2_dot = computeJLA(q, qT_dot, PA2);

		q_dot = qT_dot + PA2 * (qLim2_dot + omega);  //JLA Marey & Chaumette						
	}

	
		
	return q_dot;
	
}

Eigen::VectorXf MotionGenerator::normalize(Eigen::VectorXf qR_dot, float norm){
	Eigen::VectorXf qRand_dot(dof);
	qRand_dot.setZero();	
	
	float qR_dot_norm = qR_dot.norm();
	qRand_dot = norm/qR_dot_norm * qR_dot;  	

	return qRand_dot; 
}

Eigen::VectorXf MotionGenerator::computeRandomVelocities(){
	Eigen::VectorXf omega_rand(dof);
	omega_rand.setZero();

	for(int i = 0; i < dof; i++){
		if(i != 0 && i != 1 && i != 4 && i != 5 && i != 10 && i != 11 && i != 12)
		omega_rand(i) = -1 + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX/(1 - (-1))));
	}
	float omega_norm = omega_rand.norm();
	float omega_norm_des = 0.3;	
	omega_rand = (omega_norm_des / omega_norm) * omega_rand;

	return omega_rand;
}

Eigen::VectorXf MotionGenerator::computeBalanceVelocities(Configuration& q){
	Eigen::VectorXf omega_bal(dof);
	omega_bal.setZero();

	Eigen::Vector3f p = q.getqCoMPosition();
	
	Eigen::VectorXf p1_lf(4), p2_lf(4), p3_rf(4), p4_rf(4);
	p1_lf << +0.09, +0.05, -0.02, 1.0;
	p2_lf << -0.05, +0.05, -0.02, 1.0;
	p3_rf << +0.09, -0.05, -0.02, 1.0;
	p4_rf << -0.05, -0.05, -0.02, 1.0;
	Eigen::MatrixXf Trf_w = kin->forwardKinematics(q, R_FOOT);
	Eigen::MatrixXf Tlf_w = kin->forwardKinematics(q, L_FOOT);
	Eigen::VectorXf p1_w = Tlf_w * p1_lf;
	Eigen::VectorXf p2_w = Tlf_w * p2_lf;
	Eigen::VectorXf p3_w = Trf_w * p3_rf;
	Eigen::VectorXf p4_w = Trf_w * p4_rf;
	Eigen::Vector2f c;
	c(0) = (p1_w(0) + p2_w(0) + p3_w(0) + p4_w(0)) / 4.0;
	c(1) = (p1_w(1) + p2_w(1) + p3_w(1) + p4_w(1)) / 4.0; 	
 

	Eigen::MatrixXf JCoM = kin->getJacobian(q, support, COM);

	Eigen::VectorXf gradHBalance(dof);
	for(int i = 0; i < dof; i++) gradHBalance(i) = 2*(p(0)-c(0))*JCoM(0,i) + 2*(p(1)-c(1))*JCoM(1,i); 
	
	float eta = 0.5;
	
	omega_bal = - eta * gradHBalance; 	

	return omega_bal;
}

Configuration MotionGenerator::update_q(Configuration& q, float h, Eigen::VectorXf k){

	Eigen::VectorXf qjnt;
	Configuration qAux(dof);
	
	qjnt = q.getqjnt() + h*k;
	for(int i = 0; i < dof; i++) qAux.setqjntComponent(i, qjnt(i));

	Eigen::Matrix4f Tsup_com = kin->forwardKinematicsWrtCoM(qAux, support);
	Eigen::Matrix4f Tcom_sup = Tsup_com.inverse(); 	
	Eigen::Matrix4f Tcom_w = Tsup_w * Tcom_sup; 
	Eigen::VectorXf pcom_qAux = t2v(Tcom_w);
	qAux.setqCoMPosition(pcom_qAux.block<3,1>(0,0));
	qAux.setqCoMOrientation(pcom_qAux.block<3,1>(3,0));

	return qAux;

}

MovementPrimitiveType MotionGenerator::getPrimitiveType(){
	return type;
}

float MotionGenerator::getMotionDuration(){
	return duration;
}

Eigen::VectorXf MotionGenerator::computeGradHrange(Configuration& q){
	Eigen::VectorXf qjnt(dof);
	qjnt = q.getqjnt();
	
	Eigen::VectorXf grad(dof);
	Eigen::VectorXf qmid(dof);
	Eigen::VectorXf range(dof);
	for(int i = 0; i < dof; i++){
		float max = robot->getJoints().at(i).getJointMaxValue(); 
		float min = robot->getJoints().at(i).getJointMinValue();
		qmid(i) = (max + min) / 2;
		range(i) = max - min;
	}

	float qi;
	for(int i = 0; i < dof; i++){
		qi = q.getqjntComponent(i);
		grad(i) = (qi - qmid(i)) / (dof*std::pow(range(i) , 2));
	}	
	
	return grad;
}

// This function is not used in the code. For testing only.
float MotionGenerator::computeHrange(Eigen::VectorXf q){
	Eigen::VectorXf qmid(dof);
	Eigen::VectorXf range(dof);
	for(int i = 0; i < dof; i++){
		float max = robot->getJoints().at(i).getJointMaxValue(); 
		float min = robot->getJoints().at(i).getJointMinValue();
		qmid(i) = (max + min) / 2;
		range(i) = max - min;
	}

	float sum = 0;
	for(int i = 0; i < dof; i++){
		sum = sum + std::pow( (q(i) - qmid(i)) / range(i) , 2 );
	}
	float c = ( 1 / (2 * dof) ) * sum;
	return c;
}

Eigen::VectorXf MotionGenerator::computeJLA(Configuration& q, Eigen::VectorXf q1_dot, Eigen::MatrixXf P1){
	Eigen::VectorXf qjnt(dof);
	qjnt = q.getqjnt();
	
	Eigen::VectorXf qJLA_dot(dof);
	qJLA_dot.setZero();
	Eigen::VectorXf qJLA_dot_i(dof);
	for(int i = 0; i < dof; i++){
		qJLA_dot_i = computeJLA_i(qjnt, i, q1_dot, P1);
		qJLA_dot = qJLA_dot + qJLA_dot_i;
	}			
	
	return qJLA_dot;
}

Eigen::VectorXf MotionGenerator::computeJLA_i(Eigen::VectorXf qjnt, int i, Eigen::VectorXf q1_dot, Eigen::MatrixXf P1){
	Eigen::VectorXf qJLA_dot_i(dof);
	qJLA_dot_i.setZero();
	
	Eigen::VectorXf g_i = computeActivationSignFunction(qjnt, i);
	float lambda_l_i = computeTuningFunction(qjnt, i);
	float lambda_sec_i = computeAdaptiveGainFunction(qjnt, i, q1_dot, P1, g_i);
	
	qJLA_dot_i = - lambda_sec_i * lambda_l_i * g_i;	
			
	return qJLA_dot_i;
}

Eigen::VectorXf MotionGenerator::computeActivationSignFunction(Eigen::VectorXf qjnt, int i){
	Eigen::VectorXf g_i(dof);
	g_i.setZero();

	float q_i = qjnt(i);
	float q_i_min = robot->getJoints().at(i).getJointMinValue();
	float q_i_max = robot->getJoints().at(i).getJointMaxValue(); 
	float delta_i = q_i_max - q_i_min;
	float ro = 0.1;
	float q_l0_i_min = q_i_min + ro * delta_i;
	float q_l0_i_max = q_i_max - ro * delta_i; 
	
	for(int i0 = 0; i0 < dof; i0++){
		if(q_i < q_l0_i_min && i == i0) g_i(i0) = -1;
		if(q_l0_i_max < q_i && i == i0) g_i(i0) = +1;
	}	

	return g_i;	
}

float MotionGenerator::computeTuningFunction(Eigen::VectorXf qjnt, int i){
	float lambda_l_i = 0;
	
	float q_i = qjnt(i);
	float q_i_min = robot->getJoints().at(i).getJointMinValue();
	float q_i_max = robot->getJoints().at(i).getJointMaxValue(); 
	float delta_i = q_i_max - q_i_min;
	float ro = 0.1;
	float ro1 = 0.2;
	float q_l0_i_min = q_i_min + ro * delta_i;
	float q_l0_i_max = q_i_max - ro * delta_i; 
	float q_l1_i_min = q_l0_i_min - ro1 * ro * delta_i;
	float q_l1_i_max = q_l0_i_max + ro1 * ro * delta_i;
	float lambda_l_i_max = 1 / (1 + exp(- 12 * ((q_i - q_l0_i_max) / (q_l1_i_min - q_l0_i_max)) + 6));
	float lambda_l_i_min = 1 / (1 + exp(- 12 * ((q_i - q_l0_i_min) / (q_l1_i_min - q_l0_i_max)) + 6));
	float lambda_l0_i_max = 1 / (1 + exp(- 12 * ((q_l0_i_max - q_l0_i_max) / (q_l1_i_min - q_l0_i_max)) + 6));
	float lambda_l0_i_min = 1 / (1 + exp(- 12 * ((q_l0_i_min - q_l0_i_min) / (q_l1_i_min - q_l0_i_max)) + 6));
	float lambda_l1_i_max = 1 / (1 + exp(- 12 * ((q_l1_i_max - q_l0_i_max) / (q_l1_i_min - q_l0_i_max)) + 6));
	float lambda_l1_i_min = 1 / (1 + exp(- 12 * ((q_l1_i_min - q_l0_i_min) / (q_l1_i_min - q_l0_i_max)) + 6));
		 
	if(q_i < q_l1_i_min || q_l1_i_max < q_i) lambda_l_i = 1;
	if(q_i >= q_l1_i_min && q_i <= q_l0_i_min) lambda_l_i = (lambda_l_i_min - lambda_l0_i_min) / (lambda_l1_i_min - lambda_l0_i_min);
	if(q_i >= q_l0_i_max && q_i <= q_l1_i_max) lambda_l_i = (lambda_l_i_max - lambda_l0_i_max) / (lambda_l1_i_max - lambda_l0_i_max);
	if(q_i > q_l0_i_min && q_i < q_l0_i_max) lambda_l_i = 0;

	return lambda_l_i;
}

float MotionGenerator::computeAdaptiveGainFunction(Eigen::VectorXf qjnt, int i, Eigen::VectorXf q1_dot, Eigen::MatrixXf P1, Eigen::VectorXf g_i){
	float lambda_sec_i = 0;
	float lambda_i = 0.2;
	
	Eigen::VectorXf den = P1 * g_i;
	if(g_i(i) != 0) lambda_sec_i = (1 + lambda_i) * (std::abs(q1_dot(i)) / std::abs(den(i)));
	
	return lambda_sec_i;
}

bool MotionGenerator::jointLimitsCheck(Configuration& q){
	bool res = false;
	
	for(int i = 0; i < dof; i++){
		float q_i = q.getqjntComponent(i);
		float min_i = robot->getJoints().at(i).getJointMinValue();
		float max_i = robot->getJoints().at(i).getJointMaxValue();
		if(q_i < min_i || q_i > max_i){
			std::cout << "Joint Limit Violation: " << i << " min: " << min_i << " max: " << max_i << " q_i: " << q_i << std::endl;

			res = true;
			break;			
		}
	}	
	
	return res;
}

bool MotionGenerator::collisionCheck(Configuration& q){
	bool res = false;

	if(!isInArea(q, pCoMRoot)){
		//std::cout << "MG:: configuration outside the known area------" << std::endl;
		return false;
	}

	simInt robot_handle = simGetCollectionHandle("Robot");	
	simInt torso_handle = simGetCollectionHandle("Torso");
	simInt rarm_handle = simGetCollectionHandle("Right_Arm");
	simInt larm_handle = simGetCollectionHandle("Left_Arm");
	simInt rleg_handle = simGetCollectionHandle("Right_Leg");
	simInt lleg_handle = simGetCollectionHandle("Left_Leg");

	simInt wall_handle = simGetObjectHandle("Wall");	

	// Disable the dynamic engine
	simSetBooleanParameter(sim_boolparam_dynamics_handling_enabled,false);
	robot->setConfigurationStat(q);

	if(simCheckCollision(robot_handle, wall_handle) > 0){
		//std::cout << "Collision 1" << std::endl;
		return true;
	}
	if(simCheckCollision(rarm_handle, torso_handle) > 0){
		//std::cout << "Collision 2" << std::endl;
		return true;
	}		
	if(simCheckCollision(larm_handle, torso_handle) > 0){
		//std::cout << "Collision 3" << std::endl;
		return true;
	}		
	if(simCheckCollision(rarm_handle, rleg_handle) > 0){
		//std::cout << "Collision 4" << std::endl;
		return true;
	}		
	if(simCheckCollision(larm_handle, lleg_handle) > 0){
		//std::cout << "Collision 5" << std::endl;
		return true;
	}		
	if(simCheckCollision(rleg_handle, lleg_handle) > 0){
		//std::cout << "Collision 6" << std::endl;
		return true;
	}		

	return res;
} 

bool MotionGenerator::occlusionCheckPoint(Configuration& q, Eigen::Vector3f ps){
	bool res = false;

	simInt occ_handleN = simGetObjectHandle("SphereOcclusion"); 
	simFloat posN[3];
	posN[0] = 0.0;
	posN[1] = 0.0;
	posN[2] = -2.0;
	simSetObjectPosition(occ_handleN, -1, posN);

	simInt occ_handle = simGetObjectHandle("SphereOcclusionPoint");
	simInt robot_handle = simGetCollectionHandle("Robot");	

	Eigen::Vector3f p_a = ps;  
	
	Eigen::MatrixXf Tc_w = kin->forwardKinematics(q, TOP_CAMERA);	
	Eigen::VectorXf vc_w = t2v(Tc_w);
	Eigen::Vector3f p_b;
	p_b << vc_w(0), vc_w(1), vc_w(2); 

	// Disable the dynamic engine
	simSetBooleanParameter(sim_boolparam_dynamics_handling_enabled,false);
		
	simFloat pos[3];
	Eigen::Vector3f p;
	float s = 0.0;
	float inc = 0.05;
	while(s < 1.0){
		p = s * p_b + (1.0 - s) * p_a;
		pos[0] = p(0);
		pos[1] = p(1);
		pos[2] = p(2);
		simSetObjectPosition(occ_handle, -1, pos);
		
		if(simCheckCollision(occ_handle, sim_handle_all) > 0){ 
			if(simCheckCollision(robot_handle, occ_handle) == 0) return true;	 		
		}
			
		s = s + inc;
	}	
	

	return res;
} 

 
bool MotionGenerator::occlusionCheck(Configuration& q, float t){

	simInt occ_handleN = simGetObjectHandle("SphereOcclusionPoint");  
	simFloat posN[3];
	posN[0] = 0.0;
	posN[1] = 0.0;
	posN[2] = -2.0;
	simSetObjectPosition(occ_handleN, -1, posN);

	float k = t;
	if(task->getTaskType() != TASK_TRAJECTORY_VISUAL && task->getTaskType() != TASK_PATH_VISUAL) return false;

	if(task->getTaskType() == TASK_PATH_VISUAL){
		Eigen::MatrixXf Tc_w = kin->forwardKinematics(q, TOP_CAMERA);
		Eigen::VectorXf vc_w = t2v(Tc_w);	
		float s = task->computeClosestPoint(vc_w, 0.0);
		k = s;
	}

	bool res = false;

	simInt occ_handle = simGetObjectHandle("SphereOcclusion");
	simInt robot_handle = simGetCollectionHandle("Robot");	

	Eigen::Vector3f p_a = task->getPosition(k);  
	
	Eigen::MatrixXf Tc_w = kin->forwardKinematics(q, TOP_CAMERA);	
	Eigen::VectorXf vc_w = t2v(Tc_w);
	Eigen::Vector3f p_b;
	p_b << vc_w(0), vc_w(1), vc_w(2); 	
		
	// Disable the dynamic engine
	simSetBooleanParameter(sim_boolparam_dynamics_handling_enabled,false);
		
	simFloat pos[3];
	Eigen::Vector3f p;
	float s = 0.0;
	float inc = 0.05;
	while(s < 1.0){
		p = s * p_b + (1.0 - s) * p_a;
		pos[0] = p(0);
		pos[1] = p(1);
		pos[2] = p(2);
		simSetObjectPosition(occ_handle, -1, pos);
		
		if(simCheckCollision(occ_handle, sim_handle_all) > 0){ 
			if(simCheckCollision(robot_handle, occ_handle) == 0) return true;	 		
		}
			
		s = s + inc;
	}	
	
	return res;
} 
 


Eigen::VectorXf MotionGenerator::sign(Eigen::VectorXf eM){
	Eigen::VectorXf e(eM.rows());
	for(int i = 0; i < e.rows(); i++){
		if(eM(i) > 0) e(i) = +1;
		else if(eM(i) == 0) e(i) = 0;
		else e(i) = -1;
	}

	return e;

}

/*
MovementPrimitiveType MotionGenerator::chooseMovementPrimitive(std::vector<MovementPrimitiveType> av_set, Eigen::VectorXf weights_av_set){

	MovementPrimitiveType typeRes;
	
	Eigen::VectorXf prob_distr = weights_av_set;
	for(int i = 1; i < av_set.size(); i++) prob_distr(i) = prob_distr(i - 1) + prob_distr(i); 
	
	float r = static_cast<float>(rand()) / (static_cast<float>(RAND_MAX)/(prob_distr(av_set.size()-1)));
	int type_index;	
	int k = 0;
	while(k < av_set.size() && r > prob_distr(k)) k++;
	type_index = k;

	typeRes = av_set.at(type_index);

	return typeRes; 	
}
*/

MovementPrimitiveType MotionGenerator::chooseMovementPrimitive(std::vector<MovementPrimitiveType> av_set, Eigen::VectorXf weights_av_set){
	
	MovementPrimitiveType typeRes;
	
	/*
	Eigen::Vector3f pCoMCurr = qCurr.getqCoMPosition();
	//Eigen::Vector3f oCoMCurr = qCurr.getqCoMOrientation();
	Eigen::Matrix4f Thand = kin->forwardKinematics(qCurr, R_HAND);
	Eigen::VectorXf vhand = t2v(Thand);	
	Eigen::Vector3f goal = task->getAttractor(task->getLastReachedAttractor()+1);

	float dx = goal(0) - vhand(0);
	float dy = goal(1) - vhand(1);
	float dz = goal(2) - vhand(2);
	//float d = sqrt(std::pow(vhand(0) - goal(0), 2) + std::pow(vhand(1) - goal(1), 2) + std::pow(vhand(2) - goal(2), 2));	
	//float d = sqrt(std::pow(vhand(0) - goal(0), 2) + std::pow(vhand(1) - goal(1), 2));	
	float d = sqrt(std::pow(pCoMCurr(0) - goal(0), 2) + std::pow(pCoMCurr(1) - goal(1), 2) + std::pow(pCoMCurr(2) - goal(2), 2));	

	//std::cout << "dx = " << dx << " dy = " << dy << " dz = " << dz << " d = " << d << std::endl;  
	if(d < 0.2) return FREE_COM;

	float b = 10.0;
	for(int i = 0; i < av_set.size(); i++){
		if(dx < 0.9){
			if(dy > 0.10){
				if(av_set.at(i) == DYN_LL_C_L) weights_av_set(i) = weights_av_set(i) * b;
				if(av_set.at(i) == DYN_LL_C_R) weights_av_set(i) = weights_av_set(i) * b;
	 
			}
			else if(dy < -0.15){
				if(av_set.at(i) == DYN_LR_C_L) weights_av_set(i) = weights_av_set(i) * b;
				if(av_set.at(i) == DYN_LR_C_R) weights_av_set(i) = weights_av_set(i) * b;
			}
			else{
				if(av_set.at(i) == DYN_F_C_L) weights_av_set(i) = weights_av_set(i) * b;
				if(av_set.at(i) == DYN_F_C_R) weights_av_set(i) = weights_av_set(i) * b;
			}
		}
	}
	*/

	Eigen::VectorXf prob_distr = weights_av_set;
	for(int i = 1; i < av_set.size(); i++) prob_distr(i) = prob_distr(i - 1) + prob_distr(i); 
	
	float r = static_cast<float>(rand()) / (static_cast<float>(RAND_MAX)/(prob_distr(av_set.size()-1)));
	int type_index;	
	int k = 0;
	while(k < av_set.size() && r > prob_distr(k)) k++;
	type_index = k;

	typeRes = av_set.at(type_index);

	return typeRes; 	
}

bool MotionGenerator::equilibriumCheck(Configuration& q){   // to be re-defined
	bool res = false;
	
	Eigen::Vector3f pCoM = q.getqCoMPosition();

	Eigen::VectorXf pFoot1 = t2v(Tsup_w);
	Eigen::VectorXf pFoot2 = t2v(Tswg_w);

	float midFeet_x = 0.5 * pFoot1(0) + 0.5 * pFoot2(0);
	float midFeet_y = 0.5 * pFoot1(1) + 0.5 * pFoot2(1); 
	Eigen::Vector2f midFeet;
	midFeet(0) = midFeet_x;
	midFeet(1) = midFeet_y;

	float d = 0.1;
	if( pCoM(0) > midFeet(0) - d && pCoM(0) < midFeet(0) + d && pCoM(1) > midFeet(1) - d && pCoM(1) < midFeet(1) + d ){
		res = false;
	}
	else res = true;
		
	return res;
}

float MotionGenerator::getSimulationTime(){
	float t_sim;
				
	std::string line;		
	std::ifstream fileSimulationTime("/home/paolo/Scrivania/PlanningData/sim_time.txt");
	
	while(getline(fileSimulationTime,line)){
	std::stringstream linestream(line);
	std::string value;
	
		int index = 0;
		while(getline(linestream,value,',')){
			std::string::iterator end_pos = std::remove(value.begin(), value.end(), ' ');
			value.erase(end_pos, value.end());
			t_sim = boost::lexical_cast<float>(value);
	    	index++;
		}
				
	}

	fileSimulationTime.close();
	
	return t_sim;
}		
	
void MotionGenerator::setCurrentKnownArea(Eigen::Vector3f _pCoMRoot){
	pCoMRoot = _pCoMRoot;	
}

void MotionGenerator::setPreviousKnownArea(Eigen::Vector3f _pCoMRootPrev){
	pCoMRootPrev = _pCoMRootPrev;	
}

bool MotionGenerator::isInArea(Configuration& q, Eigen::Vector3f pCoM){
	// computing the control points on the robot

	Eigen::VectorXf v1_h(6), v2_lh(6), v3_rh(6), v4_lf(6), v5_lf(6), v6_rf(6), v7_rf(6);
	v1_h << 0.09, 0.0, 0.0, 0.0, 0.0, 0.0; 
	v2_lh << 0.05, -0.007, 0.06, 0.0, 0.0, 0.0; 
	v3_rh << 0.06, 0.0, -0.01, 0.0, 0.0, 0.0; 
	v4_lf << 0.01, 0.0, 0.0, 0.0, 0.0, 0.0; 
	v5_lf << -0.065, 0.0, 0.0, 0.0, 0.0, 0.0; 
	v6_rf << 0.01, 0.0, 0.0, 0.0, 0.0, 0.0; 
	v7_rf << -0.065, 0.0, 0.0, 0.0, 0.0, 0.0; 
	Eigen::MatrixXf T1_h = v2t(v1_h);
	Eigen::MatrixXf T2_lh = v2t(v2_lh);
	Eigen::MatrixXf T3_rh = v2t(v3_rh);
	Eigen::MatrixXf T4_lf = v2t(v4_lf);
	Eigen::MatrixXf T5_lf = v2t(v5_lf);
	Eigen::MatrixXf T6_rf = v2t(v6_rf);
	Eigen::MatrixXf T7_rf = v2t(v7_rf);

	Eigen::MatrixXf Th_w = kin->forwardKinematics(q, HEAD);
	Eigen::MatrixXf Tlh_w = kin->forwardKinematics(q, L_HAND);
	Eigen::MatrixXf Trh_w = kin->forwardKinematics(q, R_HAND);
	Eigen::MatrixXf Tlf_w = kin->forwardKinematics(q, L_FOOT);			
	Eigen::MatrixXf Trf_w = kin->forwardKinematics(q, R_FOOT);			
	
	Eigen::MatrixXf T1 = Th_w * T1_h;
	Eigen::MatrixXf T2 = Tlh_w * T2_lh;
	Eigen::MatrixXf T3 = Trh_w * T3_rh;
	Eigen::MatrixXf T4 = Tlf_w * T4_lf;
	Eigen::MatrixXf T5 = Tlf_w * T5_lf;
	Eigen::MatrixXf T6 = Trf_w * T6_rf;
	Eigen::MatrixXf T7 = Trf_w * T7_rf;

	Eigen::VectorXf v1 = t2v(T1);
	Eigen::VectorXf v2 = t2v(T2);
	Eigen::VectorXf v3 = t2v(T3);
	Eigen::VectorXf v4 = t2v(T4);
	Eigen::VectorXf v5 = t2v(T5);
	Eigen::VectorXf v6 = t2v(T6);
	Eigen::VectorXf v7 = t2v(T7);

	float d1 = sqrt(std::pow(pCoM(0) - v1(0), 2) + std::pow(pCoM(1) - v1(1), 2) + std::pow(pCoM(2) - v1(2), 2));
	float d2 = sqrt(std::pow(pCoM(0) - v2(0), 2) + std::pow(pCoM(1) - v2(1), 2) + std::pow(pCoM(2) - v2(2), 2));
	float d3 = sqrt(std::pow(pCoM(0) - v3(0), 2) + std::pow(pCoM(1) - v3(1), 2) + std::pow(pCoM(2) - v3(2), 2));
	float d4 = sqrt(std::pow(pCoM(0) - v4(0), 2) + std::pow(pCoM(1) - v4(1), 2) + std::pow(pCoM(2) - v4(2), 2));
	float d5 = sqrt(std::pow(pCoM(0) - v5(0), 2) + std::pow(pCoM(1) - v5(1), 2) + std::pow(pCoM(2) - v5(2), 2));
	float d6 = sqrt(std::pow(pCoM(0) - v6(0), 2) + std::pow(pCoM(1) - v6(1), 2) + std::pow(pCoM(2) - v6(2), 2));
	float d7 = sqrt(std::pow(pCoM(0) - v7(0), 2) + std::pow(pCoM(1) - v7(1), 2) + std::pow(pCoM(2) - v7(2), 2));  

	//float dlf = fabs(t2v(Tlf_w)(1) - pCoM(1));
	//float drf = fabs(t2v(Trf_w)(1) - pCoM(1));
	//if(dlf > 0.1 || drf > 0.1) return false;


	float r = 5.0;
	if(d1 < r && d2 < r && d3 < r && d4 < r && d5 < r && d6 < r && d7 < r) return true;
	return false; 
}

void MotionGenerator::clearCoMPositions(){
	CoMPositions.clear();
}

void MotionGenerator::addCoMPosition(Eigen::Vector3f pCoM){
	CoMPositions.push_back(pCoM);
}

void MotionGenerator::setExplorationRef(Eigen::Vector3f _pCoMRef){
	pCoMRef = _pCoMRef;	
}

float MotionGenerator::angleSignedDistance(float a, float b){
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






