
#ifndef _RobotArmController_COMPONENT_H
#define _RobotArmController_COMPONENT_H
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

#include <functional>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include "RobotArmControllerServiceProvided.h"

#include "GripperServiceRequired.h"
#include "InverseKinematicsServiceRequired.h"
#include "TrajectoryGenerationServiceRequired.h"

#include "WorkerThread.h"

namespace ProcessState
{
	enum ProcessState
	{
		Processing = 0, Fail = -1, Success = 1
	};
}

class ManipulationInfo;

class RobotArmController: public Component, public IRobotArmControllerService
{
protected:
	// service
	GripperServiceRequired* ptrLeftGripperSerivce;
	InverseKinematicsServiceRequired* ptrLeftIKService;

	GripperServiceRequired* ptrRightGripperService;
	InverseKinematicsServiceRequired* ptrRightIkService;
		
	TrajectoryGenerationServiceRequired* ptrTrajectoryGenerationService;

protected:
	// monitor variables
	ObjectPosition mLeftDesiredPosition;
	ObjectPosition mRightDesiredPosition;

	std::vector<double> mLeftArmJointPosition;
	std::vector<double> mRightArmJointPosition;

public:
	RobotArmController();
	RobotArmController(const std::string &compId);

	virtual ~RobotArmController();
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

public:
	virtual void Manipulate(unsigned char index, std::vector<ObjectPosition> position);
	virtual void Grab(unsigned char index, bool isEnable);
	virtual int GetManipulateState(unsigned char index);
	virtual int GetGripperState(unsigned char index);

private:
	bool SelectRobotArm(unsigned char index, ManipulationInfo& outManipulationInfo);
	
	bool Manipulate(std::vector<ObjectPosition> path, ManipulationInfo info);
	bool Grab(bool isEnable, ManipulationInfo info);
	bool IsEqualPosition(const ObjectPosition& position1, const ObjectPosition& position2, double tolerance = 0.001);

	double CalculateDistance(const std::vector<ObjectPosition>& path);

private:
	boost::mutex mTrajectoryGenerationServiceMutex;

	boost::mutex mLeftArmMutex;
	boost::mutex mLeftGripperMutex;
	ProcessState::ProcessState mLeftManipulationState;
	ProcessState::ProcessState mLeftGrippingState;
	
	boost::mutex mRightArmMutex;
	boost::mutex mRightGripperMutex;
	ProcessState::ProcessState mRightManipulationState;
	ProcessState::ProcessState mRightGrippingState;
	
	WorkerThread mManipulationWorkerThread;
	WorkerThread mGrippingWorkerThread;

	unsigned int mDeltaTime;
};

#endif

