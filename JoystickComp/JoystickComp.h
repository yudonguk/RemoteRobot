
#ifndef _JoystickComp_COMPONENT_H
#define _JoystickComp_COMPONENT_H
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
#include <OprosLock.h>

#include "Joystick.h"
#include "JoystickServiceProvided.h"


class JoystickComp: public Component
	,public IJoystickService
{
protected:
//data
	OutputDataPort< JoystickData > joystickData;

#if defined(WIN32)
	OprosApiHandle hOprosAPI;
#else
	void *hOprosAPI;
#endif
	Joystick *joystick;
	ReturnType lastError;
	OprosLock lock;
	Property parameter;

protected:
// service
// method for provider
	IJoystickService *ptrJoystickService;

public:
virtual ReturnType SetParameter(Property parameter);
virtual Property GetParameter();
virtual ReturnType GetError();
virtual JoystickData GetJoystickData();

protected:
// data


//event


// method for provider
	

// method for required
	


// symbol value generation
	

public:
	JoystickComp();
	JoystickComp(const std::string &compId);
	virtual ~JoystickComp();
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


};

#endif

