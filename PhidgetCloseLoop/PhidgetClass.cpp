// - Stepper simple -
// This simple example sets up a Stepper object, hooks the event handlers and opens it for device connections.  
// Once an Advanced Servo is attached it will move the motor to various positions.
//
// Please note that this example was designed to work with only one Phidget Stepper connected. 
//
// Copyright 2008 Phidgets Inc.  All rights reserved.
// This work is licensed under the Creative Commons Attribution 2.5 Canada License. 
// view a copy of this license, visit http://creativecommons.org/licenses/by/2.5/ca/

#include <stdio.h>
#include <iostream>
#include <windows.h>
#include <stdlib.h>
#include "math.h"
#include "phidget21.h"
#include "PhidgetClass.h"
#include "eventhandlers.h"

/*#ifndef WIN32
#ifndef __stdcall
#define __stdcall __attribute__((stdcall))
#endif
#endif*/

using namespace std;

CPhidgetWrapper::CPhidgetWrapper() {
	//id of the card
	devid = 0;
	// handle for the stepper communication
	stepper = 0;
	// is it a stepper? if not a dc motor.
	type_ref = 0;
	// handle for the dcmotor communication
	motorctl = 0;
	// boolean to start the pid control (one at the time!)
	started_pos = false; started_cur = false;
	// ratio of the gear box of the dc motor (to get the right rotation from the encoder)
	gearRatio = 24;
	// current values reed from the board.
	curr_pos = 0;curr_cur = 0;curr_vel = 0;
	// PID control values
	K[0] = 50; K[1] = 1; K[2] = 10;
	errorlast = 0.0; deadBand = 0.00001; integral = 0; derivative = 0;
	// command in position or current for closed-loop
	target = 0;
}

int CPhidgetWrapper::AttachHandler(CPhidgetHandle IFK)
{
	int serialNo;
	const char *name;

	CPhidget_getDeviceName (IFK, &name);
	CPhidget_getSerialNumber(IFK, &serialNo);
	printf("%s %10d attached!\n", name, serialNo);

	return 0;
}

int CPhidgetWrapper::DetachHandler(CPhidgetHandle IFK)
{
	int serialNo;
	const char *name;

	CPhidget_getDeviceName (IFK, &name);
	CPhidget_getSerialNumber(IFK, &serialNo);
	printf("%s %10d detached!\n", name, serialNo);

	return 0;
}

int CPhidgetWrapper::ErrorHandler(CPhidgetHandle IFK, int ErrorCode, const char *Description)
{
	printf("Error handled. %d - %s\n", ErrorCode, Description);
	return 0;
}

int CPhidgetWrapper::PositionChangeHandler(CPhidgetStepperHandle IFK, int Index, __int64 Value)
{
	__int64 curr_posloc;
	//printf("Error handled. %d - %s\n", ErrorCode, Description);
	CPhidgetStepper_getCurrentPosition(stepper, 0, &curr_posloc);
	curr_pos=steps2rad(curr_posloc);
	__int64 stepcommand = rad2steps(target);
	if(abs(stepcommand)>10000)	stepcommand=sgn(stepcommand)*10000;
	GoToStepper(0, stepcommand);
	return 0;
}

int CPhidgetWrapper::CurrentUpdateHandler(CPhidgetMotorControlHandle IFK, int index, double current)
{
	if (current > 0.5)
		cout << "High current!!!" << endl;

	if (current > 1.5) {
		cout << "Dangerous current!!!" << endl;
		setvel(0);
	}

	curr_cur = current;

	// If feedback is negative, loop back around to 360
	//if (feedback < 0)
	//    feedback += 2*PI;

	// If the start button has been pressed,
	if (started_cur)
	{
		// Calculate and set the new output from the control loop
		double output = PID(target - curr_cur, errorlast);
		CPhidgetMotorControl_setVelocity(motorctl, 0, output);
		errorlast = target - curr_cur;
	}
		
		
	return 0;
}
// This event handler fires every 8ms, regardless of whether the position has changed. 
// If you want to create an event that only fires when position is changed, use "EncoderPositionChange"
int CPhidgetWrapper::EncoderUpdateHandler(CPhidgetMotorControlHandle phid, int index, int positionChange)
{
	// Divide the encoder value by the gearbox reduction ratio, since the encoder is on
    // the rear shaft and rotates much faster than the primary shaft
    // Then mod 360 since this program only controls the position of the motor
    // and not the number of rotations
	double feedback = (double)positionChange*2*PI/512.0 / gearRatio;//fmod((double)(positionChange / gearRatio), 2*PI);
	curr_pos+=feedback;
	curr_vel=feedback/8.0*1000.0;
	//cout << "Velocities: " << curr_vel*gearRatio << endl;
 
    // If feedback is negative, loop back around to 360
    //if (feedback < 0)
    //    feedback += 2*PI;
 
    // If the start button has been pressed,
    if (started_pos)
    {
        // Calculate and set the new output from the control loop
        double output = PID(target - curr_pos, errorlast);
		CPhidgetMotorControl_setVelocity(motorctl, 0, output);
		errorlast = target - curr_pos;
    }
	return 0;
}

void CPhidgetWrapper::setvel(double pwr)
{
	if (abs(pwr) > MaxVel) {
		cout << "Max output reached!!" << endl;
		pwr = sgn(pwr) * 100.0;
	}
	if (abs(pwr) < MinVel) {
		cout << "Min output reached!!" << endl;
		pwr = 0.0;
	}
	CPhidgetMotorControl_setVelocity(motorctl, 0, pwr);
}

//Display the properties of the attached phidget to the screen.  We will be displaying the name, serial number and version of the attached device.
int CPhidgetWrapper::display_properties(CPhidgetHandle phid)
{
	int serialNo, version, numMotors;
	const char* ptr;

	CPhidget_getDeviceType((CPhidgetHandle)phid, &ptr);
	CPhidget_getSerialNumber((CPhidgetHandle)phid, &serialNo);
	CPhidget_getDeviceVersion((CPhidgetHandle)phid, &version);

	if(strcmp(ptr,"PhidgetStepper")==0)
		CPhidgetStepper_getMotorCount((CPhidgetStepperHandle)phid, &numMotors);
	else
		CPhidgetMotorControl_getMotorCount ((CPhidgetMotorControlHandle)phid, &numMotors);

	cout << ptr << endl;
	cout << "Serial Number: " << serialNo << ", Version: " << version << ", Motors: " << numMotors << endl;

	devid=serialNo;

	return 0;
}

int CPhidgetWrapper::rad2steps(double rad)
{
	return floor(rad/((1.8/2.0)*PI/180.0));//floor(rad/((1.8/16.0)*PI/180.0));
}

double CPhidgetWrapper::steps2rad(__int64 steps)
{
	return (double)steps*((1.8/2.0)*PI/180.0);//(double)steps*((1.8/16.0)*PI/180.0);
}

// This function calculates the shortest distance to the target and the direction of rotation necessary to reach it.
double CPhidgetWrapper::distance360(double input, double feedback)
{
    if (input > feedback)
    {
        // Return the number with lesser magnitude- this is the fastest direction
        if (abs(input - feedback) < abs(-2*PI + input - feedback))
            return(input - feedback);
        else
            return(-2*PI + input - feedback);
    }
    else
    {
        // Return the number with lesser magnitude- this is the fastest direction
        if (abs(input - feedback) < abs(2*PI - feedback + input))
            return (input - feedback);
        else
            return (2*PI - input + feedback);
    }

}

int CPhidgetWrapper::Init(int type)
{
	int rc;
	type_ref=type;
	if(type)
		rc = InitStepper(0);
	else
		rc = InitMotorCtl(0);
	return rc;
}

int CPhidgetWrapper::InitStepper(int num)
{
	int result;
	__int64 curr_posloc;
	const char *err;
	double minAccel, maxAccel, maxVel;

	//create the stepper object
	CPhidgetStepper_create(&stepper);

	//Set the handlers to be run when the device is plugged in or opened from software, unplugged or closed from software, or generates an error.
	CPhidget_set_OnAttach_Handler((CPhidgetHandle)stepper, CAttachHandler, this);
	CPhidget_set_OnDetach_Handler((CPhidgetHandle)stepper, CDetachHandler, this);
	CPhidget_set_OnError_Handler((CPhidgetHandle)stepper, CErrorHandler, this);

	//Registers a callback that will run when the motor position is changed.
	//Requires the handle for the Phidget, the function that will be called, and an arbitrary pointer that will be supplied to the callback function (may be NULL).
	CPhidgetStepper_set_OnPositionChange_Handler(stepper, CPositionChangeHandler, NULL);

	//open the device for connections
	CPhidget_open((CPhidgetHandle)stepper, -1);

	//get the program to wait for an stepper device to be attached
	cout<<"Waiting for Phidget to be attached...."<<endl;
	if((result = CPhidget_waitForAttachment((CPhidgetHandle)stepper, 10000)))
	{
		CPhidget_getErrorDescription(result, &err);
		cout<<"Problem waiting for attachment: "<<err<<endl;
		return 0;
	}
	
	//Display the properties of the attached device
	display_properties((CPhidgetHandle)stepper);

	//read event data
	cout<<"Reading....."<<endl;

	//Set up some initial acceleration and velocity values
	CPhidgetStepper_getAccelerationMin(stepper, num, &minAccel);
	CPhidgetStepper_getAccelerationMax(stepper, num, &maxAccel);
	CPhidgetStepper_setAcceleration(stepper, num, floor(maxAccel/50.0));
	CPhidgetStepper_getVelocityMax(stepper, num, &maxVel);
	CPhidgetStepper_setVelocityLimit(stepper, num, floor(maxVel/90.0));

	//display current motor position if available
	if(CPhidgetStepper_getCurrentPosition(stepper, num, &curr_posloc) == EPHIDGET_OK){
			cout<<"Motor: "<<num<<" > Starting Position: "<<curr_posloc<<endl;
			CPhidgetStepper_setCurrentPosition(stepper, num, 0);
	}

	CPhidgetStepper_setEngaged(stepper, num, 1);
	Sleep(100);
	return 1;
}

int CPhidgetWrapper::InitMotorCtl(int num)
{
	int result;
	__int64 curr_pos;
	const char *err;
	double minAccel, maxAccel, maxVel;
	
	//create the motor controller object
	CPhidgetMotorControl_create(&motorctl);

	//Set the handlers to be run when the device is plugged in or opened from software, unplugged or closed from software, or generates an error.
	CPhidget_set_OnAttach_Handler((CPhidgetHandle)motorctl, CAttachHandler, this);
	CPhidget_set_OnDetach_Handler((CPhidgetHandle)motorctl, CDetachHandler, this);
	CPhidget_set_OnError_Handler((CPhidgetHandle)motorctl, CErrorHandler, this);
	CPhidgetMotorControl_set_OnEncoderPositionUpdate_Handler(motorctl, CEncoderUpdateHandler, this);
	CPhidgetMotorControl_set_OnCurrentUpdate_Handler(motorctl, CCurrentUpdateHandler, this);

	//open the device for connections
	CPhidget_open((CPhidgetHandle)motorctl, -1);

	//get the program to wait for an stepper device to be attached
	cout<<"Waiting for Phidget to be attached...."<<endl;
	if((result = CPhidget_waitForAttachment((CPhidgetHandle)motorctl, 10000)))
	{
		CPhidget_getErrorDescription(result, &err);
		cout<<"Problem waiting for attachment: "<<err<<endl;
		return 0;
	}
	
	//Display the properties of the attached device
	display_properties((CPhidgetHandle)motorctl);

	//read event data
	cout<<"Reading....."<<endl;

	//Set up some initial acceleration and velocity values
	CPhidgetMotorControl_getAccelerationMin(motorctl, num, &minAccel);
	CPhidgetMotorControl_getAccelerationMax(motorctl, num, &maxAccel);
	CPhidgetMotorControl_setAcceleration(motorctl, num, maxAccel);
	CPhidgetMotorControl_getVelocity(motorctl, num, &maxVel);
	CPhidgetMotorControl_setVelocity(motorctl, num, maxVel);
	CPhidgetMotorControl_setEncoderPosition(motorctl, num, 0);
	CPhidgetMotorControl_setBackEMFSensingState(motorctl, num, 0);

	Sleep(100);
	return 1;
}

int CPhidgetWrapper::stepper_simple(int num, __int64 targetsteps)
{
	__int64 curr_pos;
	char FPS[50];
	int stopped;

	cout<<"SINUS"<<endl;
	int start = GetTickCount();int elapsed=0;int target=0;int freq=0;int i=0;
	while((double)elapsed/1000.0<10.0)		//sinus test for 10 sec.
	{
		elapsed = GetTickCount()-start;
		target = rad2steps(PI/2*sin(2*(double)elapsed/1000.0));
		CPhidgetStepper_setTargetPosition (stepper, num, target);
		CPhidgetStepper_getCurrentPosition(stepper, num, &curr_pos);
		freq = GetTickCount()-start-elapsed;
		if(freq!=0)
			sprintf(FPS,"%3.3f", 1/((double)freq/1000.0));
		else
			sprintf(FPS,"<1000");
//		m_FPSControl.SetWindowText(CString(FPS));
		i++;
//		pLineSeriesO->SetPoints(XValues,YValuesO,i);
//		pLineSeriesI->SetPoints(XValues,YValuesI,i);
	}

	cout<<"STRAIGHT"<<" Target: "<<targetsteps<<endl;
	CPhidgetStepper_setCurrentPosition(stepper, num, 0);
	CPhidgetStepper_setTargetPosition (stepper, num, targetsteps);
	CPhidgetStepper_getCurrentPosition(stepper, num, &curr_pos);
	i=0;
	while(curr_pos!=targetsteps){
		//dataready=false;
		CPhidgetStepper_getCurrentPosition(stepper, num, &curr_pos);
		//TRACE("Motor: %i > Current Position: %lld\n (%i)", num, curr_pos, GetTickCount());
		//YValues[i]=steps2rad(curr_pos);XValues[i]=(double)i/10.0;
		//i++;
		//dataready=true;
		//pLineSeries->SetPoints(XValues,YValues,1000);
		//Sleep(10);
	}

	stopped = PFALSE;
	while(!stopped)
	{
		CPhidgetStepper_getStopped(stepper, num, &stopped);
		//usleep(100000);
	}

	//all done, exit
	return 0;
}

double CPhidgetWrapper::SinStepper(int num, double t, int &stepcommand)
{
	__int64 curr_posloc;
	//cout<<"Elapsed: "<<t<<endl;
	__int64 target = rad2steps(PI/2*sin(4*t));
	CPhidgetStepper_setTargetPosition (stepper, num, target);
	//CPhidgetStepper_getCurrentPosition(stepper, num, &curr_pos);
	return steps2rad(curr_posloc);
}

int CPhidgetWrapper::GoToStepper(int num, int steps)
{
	CPhidgetStepper_setTargetPosition (stepper, num, steps);
	return 0;
}

void CPhidgetWrapper::setGains(double P, double I, double D, double max, double min)
{
	K[0] = P;
	K[1] = I;
	K[2] = D;
	MaxVel = max; MinVel = min;
	return;
}
// This function does the control system calculations and sets output to the duty cycle that the motor needs to run at.
double CPhidgetWrapper::PID(double error, double errorlast)
{
	double output = 0;
	double dt = 8.0/1000.0;
    // Calculate how far we are from the target
    //double errorlast = error;
    //error = target-curr_pos;//distance360(target, feedback);
 
    // If the error is within the specified deadband, and the motor is moving slowly enough,
    // Or if the motor's target is a physical limit and that limit is hit (within deadband margins),
    if ((abs(error) <= deadBand && abs(output) <= MinVel))
    {
        // Stop the motor
        output = 0;
        error = 0;
    }
    else
    {
        // Else, update motor duty cycle with the newest output value
        // This equation is a simple PID control loop
        output = ((K[0] * error) + (K[1] * integral)) + (K[2] * derivative);
 
        //output = Kp * error;
 
        //cout << "Error: " << error << endl;
    }
 
 
    //Prevent output value from exceeding maximum output specified by user, 
    //And prevent the duty cycle from falling below the minimum velocity (excluding zero)
    //The minimum velocity exists because some DC motors with gearboxes will not be able to overcome the detent torque
    //of the gearbox at low velocities.
    if (abs(output) >= MaxVel){
        output = sgn(output)*MaxVel;
		cout << "Reach max output" << endl;
	}
	else if (abs(output) < MinVel)
		output = sgn(output)*MinVel;
    else
        integral += (error * dt);
 
    // Calculate the derivative for the next iteration
    derivative = (error - errorlast) / dt;
 
    // Record the previous encoder value for the next iteration of the control loop
    //feedbacklast = feedback;
	//cout << "Error: " << error << ", vel: " << output << " (" << integral << ", " << derivative << ")" << endl;
	return output;
}

int CPhidgetWrapper::closeMot()
{
	int rc;
	started_pos = false; started_cur = 0;
	Sleep(100);
	if(type_ref)
		rc = CloseStepper(0);
	else
		rc = CloseMotorCtl(0);
	return rc;
}

int CPhidgetWrapper::CloseStepper(int num)
{
	int stopped = PFALSE;
	while(!stopped)
	{
		CPhidgetStepper_getStopped(stepper, num, &stopped);
		//usleep(100000);
	}

	CPhidgetStepper_setEngaged(stepper, num, 0);

	//printf("Press any key to end\n");
	//getchar();

	//since user input has been read, this is a signal to terminate the program so we will close the phidget and delete the object we created
	cout<<"Closing Stepper..."<<endl;
	CPhidget_close((CPhidgetHandle)stepper);

	CPhidget_delete((CPhidgetHandle)stepper);
	return 0;
}

int CPhidgetWrapper::CloseMotorCtl(int num)
{
	CPhidgetMotorControl_setVelocity(motorctl, num, 0);

	//since user input has been read, this is a signal to terminate the program so we will close the phidget and delete the object we created
	cout<<"Closing Motor Control..."<<endl;
	CPhidget_close((CPhidgetHandle)motorctl);

	CPhidget_delete((CPhidgetHandle)motorctl);
	return 0;
}

//int main(int argc, char* argv[])
//{
//	stepper_simple();
//	return 0;
//}

