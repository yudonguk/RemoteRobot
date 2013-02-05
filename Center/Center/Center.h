
#ifndef _Center_COMPONENT_H
#define _Center_COMPONENT_H
/*
 *  Generated sources by OPRoS Component Generator (OCG V2.0[Symbol])
 *   
 */
#include <Component.h>
#include <InputDataPort.h>
#include <OutputDataPort.h>
#include <InputEventPort.h>
#include <OutputEventPort.h>
#include <Event.h>
#include <OPRoSTypes.h>

#include "PhantomOmniServiceRequired.h"
#include "RobotArmControllerServiceRequired.h"
#include "JoystickServiceRequired.h"
#include "WheelControllerServiceRequired.h"

class Center: public Component
{
protected:
// service
// method for required
	PhantomOmniServiceRequired* ptrPhantomOmniService;
	RobotArmControllerServiceRequired* ptrRobotArmControllerService;
	JoystickServiceRequired* ptrJoystickService;
	WheelControllerServiceRequired* ptrWheelControllerService;

public:
	Center();
	Center(const std::string &compId);
	virtual ~Center();
	virtual void portSetup();

protected:
	virtual ReturnType onInitialize();
	virtual ReturnType onStart();
	virtual ReturnType onStop();
	virtual ReturnType onReset();
	virtual ReturnType onError();
	virtual ReturnType onRecover();
	virtual ReturnType onDestroy();

public:
	virtual ReturnType onEvent(Event *evt);
	virtual ReturnType onExecute();
	virtual ReturnType onUpdated();
	virtual ReturnType onPeriodChanged();

private:
	void ControlOnPhantom();
	void ControlOnJoystick();
};

#endif

