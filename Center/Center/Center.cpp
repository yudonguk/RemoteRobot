
/*
*  Generated sources by OPRoS Component Generator (OCG V2.1 [Symbol,Topic])
*  
*/
#include <Component.h>
#include <InputDataPort.h>
#include <OutputDataPort.h>
#include <InputEventPort.h>
#include <OutputEventPort.h>
#include <OPRoSTypes.h>
#include <EventData.h>
#include <ServiceUtils.h>

#include <device/ApiTypes.h>

#include "Center.h"

//
// constructor declaration
//
Center::Center()
{
	ptrRobotArmControllerService = NULL;
	ptrPhantomOmniService = NULL;
	ptrJoystickService = NULL;
	ptrWheelControllerService = NULL;

	portSetup();
}

//
// constructor declaration (with name)
//
Center::Center(const std::string &name):Component(name)
{
	ptrRobotArmControllerService = NULL;
	ptrPhantomOmniService = NULL;
	ptrJoystickService = NULL;
	ptrWheelControllerService = NULL;

	portSetup();
}

//
// destructor declaration
//

Center::~Center()
{
}

void Center::portSetup() 
{
	ptrRobotArmControllerService = new RobotArmControllerServiceRequired();
	addPort("RobotArmControllerService", ptrRobotArmControllerService);

	ptrPhantomOmniService = new PhantomOmniServiceRequired();
	addPort("PhantomOmniService",ptrPhantomOmniService);

	ptrJoystickService = new JoystickServiceRequired();
	addPort("JoystickService", ptrJoystickService);

	ptrWheelControllerService = new WheelControllerServiceRequired();
	addPort("WheelControllerService", ptrWheelControllerService);
}

// Call back Declaration
ReturnType Center::onInitialize()
{
	// user code here
	return OPROS_SUCCESS;
}

ReturnType Center::onStart()
{
	// user code here
	return OPROS_SUCCESS;
}

ReturnType Center::onStop()
{
	// user code here
	return OPROS_SUCCESS;
}

ReturnType Center::onReset()
{
	// user code here
	return OPROS_SUCCESS;
}

ReturnType Center::onError()
{
	// user code here
	return OPROS_SUCCESS;
}

ReturnType Center::onRecover()
{
	// user code here
	return OPROS_SUCCESS;
}

ReturnType Center::onDestroy()
{
	// user code here
	return OPROS_SUCCESS;
}

ReturnType Center::onEvent(Event *evt)
{
	// user code here
	return OPROS_SUCCESS;
}

ReturnType Center::onExecute()
{
	ControlOnJoystick();
	//ControlOnPhantom();
	return OPROS_SUCCESS;
}

ReturnType Center::onUpdated()
{
	// user code here
	return OPROS_SUCCESS;
}

ReturnType Center::onPeriodChanged()
{
	// user code here
	return OPROS_SUCCESS;
}

void Center::ControlOnPhantom()
{
	const ObjectPosition offsetPosition(0.1, 0.0, 0.2, 90, 0, 90);
	const double scale = 0.004;

	if (ptrRobotArmControllerService->GetManipulateState('L') != 0
		&& ptrRobotArmControllerService->GetManipulateState('R') != 0)
	{
		auto endpoint = ptrPhantomOmniService->GetPosition();

		std::swap(endpoint.x, endpoint.z);
		std::swap(endpoint.y, endpoint.z);

		std::swap(endpoint.roll, endpoint.pitch);

		endpoint.x *= -scale;
		endpoint.y *= -scale;
		endpoint.z *= scale;

		endpoint = endpoint + offsetPosition;

		endpoint.y += 0.1;

		endpoint.roll = offsetPosition.roll;
		endpoint.pitch = offsetPosition.pitch;
		endpoint.yaw = offsetPosition.yaw;

		std::vector<ObjectPosition> leftEndEffoctorPath;
		std::vector<ObjectPosition> rightEndEffoctorPath;

		leftEndEffoctorPath.push_back(endpoint);

		endpoint.y -= 0.2;
		endpoint.roll = -90;
		endpoint.pitch = 0;
		endpoint.yaw = 90.0;
		rightEndEffoctorPath.push_back(endpoint);

		ptrRobotArmControllerService->Manipulate('L', leftEndEffoctorPath);
		ptrRobotArmControllerService->Manipulate('R', rightEndEffoctorPath);
	}

	PhantomButtonState buttonState = ptrPhantomOmniService->GetButtonState();

	//¿Þ¼Õ
	switch(ptrRobotArmControllerService->GetGripperState('L'))
	{
	case 1:
		if (!(buttonState & PHANTOM_BUTTON1))
			ptrRobotArmControllerService->Grab('L', false);
		break;
	case -1:
		if (buttonState & PHANTOM_BUTTON1)
			ptrRobotArmControllerService->Grab('L', true);		
		break;
	}

	//¿À¸¥¼Õ
	switch(ptrRobotArmControllerService->GetGripperState('R'))
	{
	case 1:
		if (!(buttonState & PHANTOM_BUTTON2))
			ptrRobotArmControllerService->Grab('R', false);
		break;
	case -1:
		if (buttonState & PHANTOM_BUTTON2)
			ptrRobotArmControllerService->Grab('R', true);		
		break;
	}
}

void Center::ControlOnJoystick()
{
	static bool mode = true;
	static bool previousButton = false;
	static ObjectPosition offsetPosition(0.1, 0.0, 0.3, 90, 0, 90);

	const double scale = 0.01;
	auto controlData = ptrJoystickService->GetJoystickData();

	if (!previousButton && controlData.button[5])
	{
		mode = !mode;
	}

	previousButton = controlData.button[5];

	if (mode)
	{
		const double maxLinearVelocity = 0.2;
		const double maxAngularVelocity = 30;

		ptrWheelControllerService->DriveWheel(maxLinearVelocity * -controlData.y
			, maxAngularVelocity * -controlData.x);
	}
	else
	{

		if (ptrRobotArmControllerService->GetManipulateState('L') != 0
			&& ptrRobotArmControllerService->GetManipulateState('R') != 0)
		{
			offsetPosition.x += -controlData.y * scale;
			offsetPosition.y += -controlData.x * scale;
			offsetPosition.z += (abs(controlData.z ) < 0.0001 ? 0 : controlData.z) * scale;

			auto endpoint = offsetPosition;

			std::vector<ObjectPosition> leftEndEffoctorPath;
			std::vector<ObjectPosition> rightEndEffoctorPath;

			endpoint.y += 0.1;
			endpoint.roll = 90;
			endpoint.pitch = 0;
			endpoint.yaw = 90;

			leftEndEffoctorPath.push_back(endpoint);

			endpoint.y -= 0.2;
			endpoint.roll = -90;
			endpoint.pitch = 0;
			endpoint.yaw = 90.0;

			rightEndEffoctorPath.push_back(endpoint);

			ptrRobotArmControllerService->Manipulate('L', leftEndEffoctorPath);
			ptrRobotArmControllerService->Manipulate('R', rightEndEffoctorPath);
		}

		//¿Þ¼Õ
		switch(ptrRobotArmControllerService->GetGripperState('L'))
		{
		case 1:
			if (!(controlData.button[0]))
				ptrRobotArmControllerService->Grab('L', false);
			break;
		case -1:
			if (controlData.button[0])
				ptrRobotArmControllerService->Grab('L', true);		
			break;
		}
		//¿À¸¥¼Õ
		switch(ptrRobotArmControllerService->GetGripperState('R'))
		{
		case 1:
			if (!(controlData.button[1]))
				ptrRobotArmControllerService->Grab('R', false);
			break;
		case -1:
			if (controlData.button[1])
				ptrRobotArmControllerService->Grab('R', true);		
			break;
		}
	}
}

#ifndef MAKE_STATIC_COMPONENT
#ifdef WIN32

extern "C"
{
	__declspec(dllexport) Component *getComponent();
	__declspec(dllexport) void releaseComponent(Component *pcom);
}

Component *getComponent()
{
	return new Center();
}

void releaseComponent(Component *com)
{
	delete com;
}

#else
extern "C"
{
	Component *getComponent();
	void releaseComponent(Component *com);
}
Component *getComponent()
{
	return new Center();
}

void releaseComponent(Component *com)
{
	delete com;
}
#endif
#endif

