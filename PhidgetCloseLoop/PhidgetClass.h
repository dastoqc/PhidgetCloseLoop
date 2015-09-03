// Stepper-simple.h : header file
//
#ifndef PHIDGET_WRAPPER_H
#define PHIDGET_WRAPPER_H

// - Stepper simple -
// This simple example sets up a Stepper object, hooks the event handlers and opens it for device connections.  
// Once an Advanced Servo is attached it will move the motor to various positions.
//
// Please note that this example was designed to work with only one Phidget Stepper connected. 
//
// Copyright 2008 Phidgets Inc.  All rights reserved.
// This work is licensed under the Creative Commons Attribution 2.5 Canada License. 
// view a copy of this license, visit http://creativecommons.org/licenses/by/2.5/ca/

//#include <unistd.h>
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

public:
	CPhidgetWrapper() {devid=0; stepper=0; type_ref=0; motorctl=0; started=false; gearRatio=24; error=0; curr_pos=0; curr_vel=0; vmin=1; K[0]=60; K[1]=3; K[2]=1; deadBand=0.05; maxOutput=100; integral=0; derivative=0; target=0;}
	//for Jabb curves: k={50,3,1} //DS vmin=25
	~CPhidgetWrapper() {}
	int AttachHandler(CPhidgetHandle IFK);
	int DetachHandler(CPhidgetHandle IFK);
	int ErrorHandler(CPhidgetHandle IFK, int ErrorCode, const char *Description);
	int PositionChangeHandler(CPhidgetStepperHandle IFK, int Index, __int64 Value);
	int EncoderUpdateHandler(CPhidgetMotorControlHandle phid, int index, int positionChange);

	int InitStepper(int num);
	int stepper_simple(int num, __int64 targetsteps);
	int CloseStepper(int num);
	void setvel(double pwr);
	double SinStepper(int num, double t, int &stepcommand);
	double GetPos(int num);
	int GoToStepper(int num, int steps);
	
	int Init(int type);
	int InitMotorCtl(int num);
	int closeMot();
	int CloseMotorCtl(int num);

	double PID();
	double distance360(double input, double feedback);
	int rad2steps(double rad);
	double steps2rad(__int64 steps);
	bool started;
	int type_ref;
	int devid;
	double gearRatio, error, deadBand, K[3], vmin, maxOutput, integral, derivative, target, curr_pos, curr_vel;
};

#endif