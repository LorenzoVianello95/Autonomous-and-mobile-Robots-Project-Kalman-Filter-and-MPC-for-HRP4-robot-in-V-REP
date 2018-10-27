/* 
 * File:   Joint.hpp
 * Author: Paolo Ferrari
 *
 * Created on Feb 9, 2016
 */

#ifndef JOINTS_HPP
#define	JOINTS_HPP

class Joint {
	public:
		Joint();
		Joint(unsigned short int h, float jmin, float jmax, float vmin, float vmax);
		virtual ~Joint();
		
		void setJointHandle(unsigned short int h);
		void setJointMinValue(float jmin);
		void setJointMaxValue(float jmax);
		void setJointMinVelocity(float vmin);
		void setJointMaxVelocity(float vmax);
		unsigned short int getJointHandle();
		float getJointMinValue();
		float getJointMaxValue();
		float getJointMinVelocity();
		float getJointMaxVelocity();
		void setJointModePassive(unsigned short int handle);
		void setJointModeForceTorque(unsigned short int handle);
		void printJoint();

	private:
		unsigned short int handle;
		float jnt_min, jnt_max, vel_min, vel_max;
};

#endif	/* JOINT_HPP */

