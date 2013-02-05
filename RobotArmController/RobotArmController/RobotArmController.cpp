
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

#include <device/OprosMath.h>

#include "RobotArmController.h"

class  ManipulationInfo
{
public:
	ManipulationInfo(GripperServiceRequired* pGripper_ = nullptr, InverseKinematicsServiceRequired* pInverseKinematics_ = nullptr
		, ProcessState::ProcessState* pGrippingState_ = nullptr, ProcessState::ProcessState* pManipulationState_ = nullptr
		, boost::mutex* pGripperMutex_ = nullptr, boost::mutex* pArmMutex_ = nullptr
		, ObjectPosition* pDesiredPositionMonitoring_ = nullptr, std::vector<double>* pJointPositionMonitoring_ = nullptr)
		: pGripper(pGripper_), pInverseKinematics(pInverseKinematics_)
		, pGrippingState(pGrippingState_), pManipulationState(pManipulationState_)
		, pGripperMutex(pGripperMutex_), pArmMutex(pArmMutex_)
		, pDesiredPositionMonitoring(pDesiredPositionMonitoring_), pJointPositionMonitoring(pJointPositionMonitoring_)
	{}
	
public:
	GripperServiceRequired* pGripper;
	InverseKinematicsServiceRequired* pInverseKinematics;

	ProcessState::ProcessState* pGrippingState;
	ProcessState::ProcessState* pManipulationState;
	
	boost::mutex* pGripperMutex;
	boost::mutex* pArmMutex;

	ObjectPosition* pDesiredPositionMonitoring;
	std::vector<double>* pJointPositionMonitoring;
};

//
// constructor declaration
//
RobotArmController::RobotArmController()
	: mManipulationWorkerThread(2), mGrippingWorkerThread(2)
{
	ptrTrajectoryGenerationService = NULL;
	ptrRightIkService = NULL;
	ptrRightGripperService = NULL;
	ptrLeftIKService = NULL;
	ptrLeftGripperSerivce = NULL;
		
	portSetup();
}

//
// constructor declaration (with name)
//
RobotArmController::RobotArmController(const std::string &name)
	: Component(name), mManipulationWorkerThread(2), mGrippingWorkerThread(2)
{
	ptrTrajectoryGenerationService = NULL;
	ptrRightIkService = NULL;
	ptrRightGripperService = NULL;
	ptrLeftIKService = NULL;
	ptrLeftGripperSerivce = NULL;
	
	portSetup();
}

//
// destructor declaration
//
RobotArmController::~RobotArmController() 
{
}

void RobotArmController::portSetup() 
{
	ProvidedServicePort *pa1;
	pa1 = new RobotArmControllerServiceProvided(this);
	addPort("RobotArmControllerService",pa1);

	ptrTrajectoryGenerationService = new TrajectoryGenerationServiceRequired();
	addPort("TrajectoryGenerationService",ptrTrajectoryGenerationService);

	ptrRightIkService = new InverseKinematicsServiceRequired();
	addPort("RightIkService",ptrRightIkService);

	ptrRightGripperService = new GripperServiceRequired();
	addPort("RightGripperService",ptrRightGripperService);

	ptrLeftIKService = new InverseKinematicsServiceRequired();
	addPort("LeftIKService",ptrLeftIKService);

	ptrLeftGripperSerivce = new GripperServiceRequired();
	addPort("LeftGripperSerivce",ptrLeftGripperSerivce);
	
	// export variable setup
	EXPORT_VARIABLE("rightDesiredPosition", mRightDesiredPosition);
	EXPORT_VARIABLE("leftDesiredPosition", mLeftDesiredPosition);
	EXPORT_VARIABLE("rightArmJointPosition", mRightArmJointPosition);
	EXPORT_VARIABLE("leftArmJointPosition", mLeftArmJointPosition);
}

// Call back Declaration
ReturnType RobotArmController::onInitialize()
{
	mDeltaTime = 50;
	return OPROS_SUCCESS;
}

ReturnType RobotArmController::onStart()
{
	//로봇팔 호밍
	//왼팔
	/*
	vector<double> jointPosition(ptrLeftGripperSerivce->GetPosition().size());
	
	jointPosition[2] = -60;
	jointPosition[3] = 45;
	jointPosition[4] = 45;
	jointPosition[5] = 20;
	jointPosition[6] = -90;

	ptrLeftGripperSerivce->SetPosition(jointPosition, std::vector<unsigned long>(jointPosition.size()));
	ptrLeftGripperSerivce->StopGripping();

	//오른팔

	jointPosition.clear();
	jointPosition.resize(ptrRightGripperService->GetPosition().size());
	
	jointPosition[2] = 60;
	jointPosition[3] = -45;
	jointPosition[4] = -45;
	jointPosition[5] = -20;
	jointPosition[6] = 90;

	ptrRightGripperService->SetPosition(jointPosition, std::vector<unsigned long>(jointPosition.size()));
	ptrRightGripperService->StopGripping();
	*/

	ptrLeftGripperSerivce->StopGripping();
	ptrRightGripperService->StopGripping();

	mLeftManipulationState = ProcessState::Success;
	mLeftGrippingState = ProcessState::Fail;
	mRightManipulationState = ProcessState::Success;
	mRightGrippingState = ProcessState::Fail;

	return OPROS_SUCCESS;
}
	
ReturnType RobotArmController::onStop()
{
	mManipulationWorkerThread.Clear();
	mGrippingWorkerThread.Clear();

	return OPROS_SUCCESS;
}

ReturnType RobotArmController::onReset()
{
	// user code here
	return OPROS_SUCCESS;
}

ReturnType RobotArmController::onError()
{
	// user code here
	return OPROS_SUCCESS;
}

ReturnType RobotArmController::onRecover()
{
	// user code here
	return OPROS_SUCCESS;
}

ReturnType RobotArmController::onDestroy()
{
	// user code here
	return OPROS_SUCCESS;
}

ReturnType RobotArmController::onEvent(Event *evt)
{
	// user code here
	return OPROS_SUCCESS;
}

ReturnType RobotArmController::onExecute()
{
	// user code here
	return OPROS_SUCCESS;
}
	
ReturnType RobotArmController::onUpdated()
{
	// user code here
	return OPROS_SUCCESS;
}

ReturnType RobotArmController::onPeriodChanged()
{
	// user code here
	return OPROS_SUCCESS;
}

int RobotArmController::GetGripperState(unsigned char index)
{
	ManipulationInfo manipulationInfo;

	if (SelectRobotArm(index, manipulationInfo))
	{
		return *manipulationInfo.pGrippingState;
	}

	return ProcessState::Fail;
}

int RobotArmController::GetManipulateState(unsigned char index)
{
	ManipulationInfo manipulationInfo;

	if (SelectRobotArm(index, manipulationInfo))
	{
		return *manipulationInfo.pManipulationState;
	}

	return ProcessState::Fail;
}

void RobotArmController::Grab(unsigned char index, bool isEnable)
{
	ManipulationInfo manipulationInfo;

	if (SelectRobotArm(index, manipulationInfo))
	{
		mGrippingWorkerThread.Enqueue(
		[this, isEnable, manipulationInfo]()
		{
			try
			{	
				*manipulationInfo.pGrippingState = ProcessState::Processing;

				if(Grab(isEnable, manipulationInfo))
				{
					*manipulationInfo.pGrippingState = ProcessState::Success;
				}
				else
				{
					*manipulationInfo.pGrippingState = ProcessState::Fail;
				}

			}
			catch (...)
			{
				*manipulationInfo.pGrippingState = ProcessState::Fail;
			}
		});
	}
}

bool RobotArmController::Grab( bool isEnable, ManipulationInfo info )
{
	boost::mutex::scoped_lock lock(*info.pGripperMutex);

	GripperServiceRequired& gripper = *info.pGripper;

	if (isEnable)
	{
		gripper.StartGripping();
		return true;
	}
	else
	{
		gripper.StopGripping();
		return false;
	}
}

void RobotArmController::Manipulate(unsigned char index, std::vector<ObjectPosition> position)
{
	if (false)	
	{
		auto jointPosition = ptrLeftGripperSerivce->GetPosition();

		jointPosition[2] = -90;
		jointPosition[3] = 45;
		jointPosition[4] = 45;
		jointPosition[5] = 20;
		jointPosition[6] = -90;

		ptrLeftIKService->SetJointPosition(jointPosition);
		auto objectPosition = ptrLeftIKService->GetCurrentPosition();

		ptrLeftGripperSerivce->SetPosition(jointPosition, std::vector<unsigned long>(jointPosition.size()));

		//////////////////////////////////////////////////////////////////////////
		
		jointPosition = ptrRightGripperService->GetPosition();

		jointPosition[2] = 90;
		jointPosition[3] = -45;
		jointPosition[4] = -45;
		jointPosition[5] = -20;
		jointPosition[6] = 90;

		ptrRightIkService->SetJointPosition(jointPosition);
		objectPosition = ptrRightIkService->GetCurrentPosition();

		ptrRightGripperService->SetPosition(jointPosition, std::vector<unsigned long>(jointPosition.size()));

		return;
	}

	ManipulationInfo manipulationInfo;

	if (SelectRobotArm(index, manipulationInfo))
	{
		mManipulationWorkerThread.Enqueue(
		[this, position, manipulationInfo]()
		{
			try
			{	
				*manipulationInfo.pManipulationState = ProcessState::Processing;

				if(Manipulate(position, manipulationInfo))
				{
					*manipulationInfo.pManipulationState = ProcessState::Success;
				}
				else
				{
					*manipulationInfo.pManipulationState = ProcessState::Fail;
				}

			}
			catch (...)
			{
				*manipulationInfo.pManipulationState = ProcessState::Fail;
			}
		});
	}
}

bool RobotArmController::Manipulate(std::vector<ObjectPosition> path, ManipulationInfo info)
{
	if (path.size() == 0)	
	{
		return false;
	}
	else if(path.size() != 1)
	{
		//경로 생성
		boost::mutex::scoped_lock lock(mTrajectoryGenerationServiceMutex);
			
		const ObjectPosition startPosition = *path.begin();
		const ObjectPosition deltaPosition = ObjectPosition(*path.rbegin()) - startPosition;

		const size_t pathCount = static_cast<size_t>(std::ceil(path.size() * 1.2));
		std::vector<std::valarray<double>> inputPath(pathCount);

		const double totalTime = CalculateDistance(path) * 50;

		for (size_t i = 0; i < pathCount; i++)
		{
			std::valarray<double>& position = inputPath[i];
			
			position.resize(3);

			const double ratio = max_val(double(i + 1) / path.size(), 1.0);

			position[0] = totalTime * (i + 1) / pathCount;
			position[1] = startPosition.x + deltaPosition.x * ratio;
			position[2] = startPosition.y + startPosition.y * ratio;
		}		

		if (ptrTrajectoryGenerationService->GenerateTrajectory(inputPath, mDeltaTime / 1000.0) != OPROS_SUCCESS)
		{
			return false;
		}

		std::vector<std::valarray<double>> generatedPath = ptrTrajectoryGenerationService->GetPosition();
		
		path.resize(generatedPath.size());
		
		for (size_t i = 0, end = path.size(); i < end; i++)
		{
			ObjectPosition& obejctPostion = path[i];
			std::valarray<double>& xyPosition = generatedPath[i];
			
			const double ratio = max_val((i + 1) * 1.2 / end, 1.0);
			
			obejctPostion.x = xyPosition[1];
			obejctPostion.y = xyPosition[2];
			obejctPostion.z = startPosition.z + deltaPosition.z * ratio;
			obejctPostion.roll = startPosition.roll + deltaPosition.roll * ratio;
			obejctPostion.pitch = startPosition.pitch + startPosition.pitch * ratio;
			obejctPostion.yaw = startPosition.yaw + startPosition.yaw * ratio;
		}
	}
	
	boost::mutex::scoped_lock lock(*info.pGripperMutex);

	GripperServiceRequired& gripper = *info.pGripper;
	std::vector<unsigned long> dummyTime(gripper.GetPosition().size());

	if (dummyTime.size() == 0)
	{
		return false;
	}

	InverseKinematicsServiceRequired& inverseKinematics = *info.pInverseKinematics;
	auto& desiredPositionMonitor = *info.pDesiredPositionMonitoring;
	auto& jointPositionMonitor = *info.pJointPositionMonitoring;

	//생성된 경로를 기반으로 IK 연산을 하여 로봇팔 제어

	std::for_each(path.begin(), path.end(),
	[&](const ObjectPosition& position)
	{
		std::vector<double> jointPosition = gripper.GetPosition();

		jointPositionMonitor = jointPosition;
		desiredPositionMonitor = position;
		
		//inverseKinematics.SetJointPosition(jointPosition);
		for(size_t i = 0; i < 10; i++)
			inverseKinematics.SetDesiredPosition(position);
		
		gripper.SetPosition(inverseKinematics.GetJointPosition(), dummyTime);

		boost::this_thread::sleep(boost::posix_time::millisec(mDeltaTime));
	});
	
	//최종 위치에 도달했는지 검사.
	const ObjectPosition& endPosition = *path.crbegin();
	
	inverseKinematics.SetJointPosition(gripper.GetPosition());
	const ObjectPosition currentPosition = inverseKinematics.GetCurrentPosition();
	
	if (!IsEqualPosition(endPosition, currentPosition, 0.01)) 
	{
		//현재 위치와 최종 목표 위치의 오차가 0.01m 이상인 경우 오류처리
		return false;
	}	

	return true;
}

bool RobotArmController::SelectRobotArm(unsigned char index, ManipulationInfo& outManipulationInfo)
{
	switch(index)
	{
	case 'l':
	case 'L':
		outManipulationInfo = ManipulationInfo(ptrLeftGripperSerivce, ptrLeftIKService
			, &mLeftGrippingState, &mLeftManipulationState, &mLeftGripperMutex, &mLeftArmMutex
			, &mLeftDesiredPosition, &mLeftArmJointPosition);
		break;

	case 'r':
	case 'R':
		outManipulationInfo = ManipulationInfo(ptrRightGripperService, ptrRightIkService
			, &mRightGrippingState, &mRightManipulationState, &mRightGripperMutex, &mRightArmMutex
			, &mRightDesiredPosition, &mRightArmJointPosition);
		break;		

	default:
		return false;
	}

	return true;
}

bool RobotArmController::IsEqualPosition( const ObjectPosition& position1, const ObjectPosition& position2, double tolerance /*= 0.0001*/ )
{
	double dx = position1.x - position2.x;
	double dy = position1.y - position2.y;
	double dz = position1.z - position2.z;

	return sqrt(dx * dx + dy * dy + dz * dz) < tolerance ? true : false;
}

double RobotArmController::CalculateDistance(const std::vector<ObjectPosition>& path)
{
	double distance = 0.0;

	for (size_t i = 1, end = path.size(); i < end; i++)
	{
		const ObjectPosition& previousPosition = path[i - 1];
		const ObjectPosition& currentPosition = path[i];

		double dx = currentPosition.x - previousPosition.x;
		double dy = currentPosition.y - previousPosition.y;
		double dz = currentPosition.z - previousPosition.z;

		distance += sqrt(dx * dx + dy * dy + dz * dz);
	}

	return distance;
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
	return new RobotArmController();
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
	return new RobotArmController();
}

void releaseComponent(Component *com)
{
	delete com;
}
#endif
#endif

