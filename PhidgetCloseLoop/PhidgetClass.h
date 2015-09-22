// Stepper-simple.h : header file
//
#ifndef PHIDGET_WRAPPER_H
#define PHIDGET_WRAPPER_H

// - Phidget Wrapper librairy -
// Wrapper for the C librairy of Phidget21 (need to be install in c:/program files) using MotorControler or StepperControler
// Wrote by David St-Onge, 2015 and inspired by the C++ stepper example of Phidget.
//
// Copyright 2008 Phidgets Inc.  All rights reserved.
// This work is licensed under the Creative Commons Attribution 2.5 Canada License. 
// view a copy of this license, visit http://creativecommons.org/licenses/by/2.5/ca/

#include "phidget21.h"

#define PI 3.141618

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

class CPhidgetWrapper
{
private:
	CPhidgetStepperHandle stepper;
	CPhidgetMotorControlHandle motorctl;

	//Display the properties of the attached phidget to the screen.  We will be displaying the name, serial number and version of the attached device.
	int display_properties(CPhidgetHandle phid);
	// PID for current or position (encoder) control loop
	double PID(double error, double errorlast);
	int type_ref;// motor type, 0:stepper, 1:dc
	double gearRatio, errorlast, deadBand, integral, derivative, target;
	// default gains.
	double K[3]; double MaxVel; double MinVel;

public:
	CPhidgetWrapper();
	~CPhidgetWrapper() {}
	int AttachHandler(CPhidgetHandle IFK);
	int DetachHandler(CPhidgetHandle IFK);
	int ErrorHandler(CPhidgetHandle IFK, int ErrorCode, const char *Description);
	int PositionChangeHandler(CPhidgetStepperHandle IFK, int Index, __int64 Value);
	int EncoderUpdateHandler(CPhidgetMotorControlHandle phid, int index, int positionChange);
	int CurrentUpdateHandler(CPhidgetMotorControlHandle IFK, int index, double current);

	int InitStepper(int num);
	int stepper_simple(int num, __int64 targetsteps);
	int CloseStepper(int num);
	void setvel(double pwr);
	double SinStepper(int num, double t, int &stepcommand);
	int GoToStepper(int num, int steps);
	
	int Init(int type);
	int InitMotorCtl(int num);
	int closeMot();
	int CloseMotorCtl(int num);

	void setGains(double P, double I, double D, double max, double min);
	double distance360(double input, double feedback);
	int rad2steps(double rad);
	double steps2rad(__int64 steps);
	bool started_pos, started_cur;
	double curr_pos, curr_cur, curr_vel;
	int devid;
};

#endif