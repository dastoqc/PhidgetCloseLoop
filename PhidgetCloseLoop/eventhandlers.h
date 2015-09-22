// eventhandlers.h : header file
//
#ifndef EVENTHANDLERS_H
#define EVENTHANDLERS_H

#include "phidget21.h"

int CCONV CAttachHandler(CPhidgetHandle IFK, void *userptr)
{
	((CPhidgetWrapper *)userptr)->AttachHandler(IFK);
	return 0;
};
int CCONV CDetachHandler(CPhidgetHandle IFK, void *userptr)
{
	((CPhidgetWrapper *)userptr)->DetachHandler(IFK);
	return 0;
};
int CCONV CErrorHandler(CPhidgetHandle IFK, void *userptr, int ErrorCode, const char *unknown)
{
	((CPhidgetWrapper *)userptr)->ErrorHandler(IFK, ErrorCode, unknown);
	return 0;
};
static int CCONV CPositionChangeHandler(CPhidgetStepperHandle stepper, void *userptr, int Index, __int64 Value)
{
	((CPhidgetWrapper *)userptr)->PositionChangeHandler(stepper, Index, Value);
	return 0;
};
static int CCONV CEncoderUpdateHandler(CPhidgetMotorControlHandle phid, void *userptr, int index, int positionChange)
{
	((CPhidgetWrapper *)userptr)->EncoderUpdateHandler(phid, index, positionChange);
	return 0;
};
static int CCONV CCurrentUpdateHandler(CPhidgetMotorControlHandle phid, void *userPtr, int index, double current)
{
	((CPhidgetWrapper *)userPtr)->CurrentUpdateHandler(phid, index, current);
	return 0;
};

#endif
