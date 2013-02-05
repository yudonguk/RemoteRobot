#ifdef TEST_BUILD

#include <Component.h>

#include <RequiredServicePort.h>

#include "RobotArmController.h"

#include "DynamicLibraryLoader.h"
#include "ProfileLoader.h"

#include "PhantomOmniDataType.h"
#include "PhantomOmniServiceRequired.h"
#include "JoystickServiceRequired.h"
#include "WheelControllerServiceRequired.h"

class ComponentLoader
{
public:
	ComponentLoader(const std::string& componentName, const std::string& profileName = std::string(""))
		: mDllLoader(componentName), mhTimer(nullptr)
	{
		mpComponent =  static_cast<Component*>(mDllLoader.GetFunction<Component*(*)()>("getComponent")());
		
		if (profileName.empty())
			mProfileLoader.Load(std::string(componentName).append(".xml"));
		else
			mProfileLoader.Load(profileName);
	}

	~ComponentLoader()
	{
		Destroy();
		delete mpComponent;
	}

public:
	Component* Initialize()
	{
		SetProperty(mProfileLoader.GetProperty());
		if (mpComponent->initialize() != OPROS_SUCCESS)
			return nullptr;
		else
			return mpComponent;
	}

	Component* Start(int periodicTime = -1)
	{
		if (mpComponent->start() != OPROS_SUCCESS)
			return nullptr;

		if (periodicTime > 0
			&& !CreateTimerQueueTimer(&mhTimer, nullptr, &TimerCallback, this, periodicTime, periodicTime, WT_EXECUTEINTIMERTHREAD))
		{
			return nullptr;
		}

		return mpComponent;
	}

	Component* Stop()
	{
		if (mhTimer != nullptr)
		{
			DeleteTimerQueueTimer(nullptr, mhTimer, nullptr);
			mhTimer = nullptr;
		}

		if (mpComponent->stop() != OPROS_SUCCESS)
			return nullptr;
		else
			return mpComponent;
	}

	Component* Destroy()
	{
		if (mhTimer != nullptr)
		{
			DeleteTimerQueueTimer(nullptr, mhTimer, nullptr);
			mhTimer = nullptr;
		}

		if (mpComponent->destroy() != OPROS_SUCCESS)
			return nullptr;
		else
			return mpComponent;
	}

	void SetProperty(Property& property)
	{
		typedef std::map<std::string, std::string> PropertyMap;
		PropertyMap& propertyMap = property.GetProperty();

		for (PropertyMap::const_iterator itor = propertyMap.begin()
			; itor != propertyMap.end(); ++itor)
		{
			mpComponent->setProperty(itor->first, itor->second);
		}
	}

	bool Connect(const std::string& serviceName, RequiredServicePort* pRequiredService)
	{
		auto pPort = mpComponent->getPort(serviceName);
		
		if (pPort->getType() != OPROS_SERVICE)
			return false;

		auto pServicePort = static_cast<ServicePort*>(pPort);
		
		if (pServicePort->getRole() != OPROS_SERVICE_PORT_PROVIDED)
			return false;

		if(pRequiredService->setPeer(pServicePort) != OPROS_SUCCESS)
			return false;

		return true;
	}

	ProvidedServicePort* GetProvidedServicePort(const std::string& serviceName)
	{
		auto pPort = mpComponent->getPort(serviceName);

		if (pPort->getType() != OPROS_SERVICE)
			return false;

		auto pServicePort = static_cast<ServicePort*>(pPort);

		if (pServicePort->getRole() != OPROS_SERVICE_PORT_PROVIDED)
			return nullptr;

		return static_cast<ProvidedServicePort*>(pServicePort);
	}

	RequiredServicePort* GetRequiredServicePort(const std::string& serviceName)
	{
		auto pPort = mpComponent->getPort(serviceName);

		if (pPort->getType() != OPROS_SERVICE)
			return false;

		auto pServicePort = static_cast<ServicePort*>(pPort);

		if (pServicePort->getRole() != OPROS_SERVICE_PORT_REQUIRED)
			return nullptr;

		return static_cast<RequiredServicePort*>(pServicePort);
	}

private:
	static void CALLBACK TimerCallback(void* param, BOOLEAN timerCalled)
	{
		static_cast<ComponentLoader*>(param)->RunComponentExecute();
	}

	void RunComponentExecute()
	{
		mpComponent->onExecute();
	}

private:
	DynamicLibraryLoader mDllLoader;
	ProfileLoader mProfileLoader;
	Component* mpComponent;
	HANDLE mhTimer;
};

int main()
{
	RobotArmController robotArmController;

	ComponentLoader leftGripperLoader("GripperComp", "LeftGripperComp.xml");
	ComponentLoader rightGripperLoader("GripperComp", "rightGripperComp.xml");

	ComponentLoader leftIKLoader("KITECH_DampedLeastSquareInverseKinematicsComp", "LeftInverseKinematicsComp.xml");
	ComponentLoader rightIKLoader("KITECH_DampedLeastSquareInverseKinematicsComp", "RightInverseKinematicsComp.xml");

	ComponentLoader trajectoryGenretorLoader("KitechMonotoneCubicTrajectoryGenerationComp");

	//ComponentLoader phantomOmniCompLoader("PhantomOmniComp");

	ComponentLoader centerLoader("Center");

	ComponentLoader joystickCompLoader("JoystickComp");

	ComponentLoader wheelCompLoader("WheelControllerComp");

	//서비스 연결
	leftGripperLoader.Connect("GripperService", static_cast<RequiredServicePort*>(robotArmController.getPort("LeftGripperSerivce")));
	rightGripperLoader.Connect("GripperService", static_cast<RequiredServicePort*>(robotArmController.getPort("RightGripperService")));

	leftIKLoader.Connect("InverseKinematicsService", static_cast<RequiredServicePort*>(robotArmController.getPort("LeftIKService")));
	rightIKLoader.Connect("InverseKinematicsService", static_cast<RequiredServicePort*>(robotArmController.getPort("RightIkService")));

	trajectoryGenretorLoader.Connect("TrajectoryGenerationService", static_cast<RequiredServicePort*>(robotArmController.getPort("TrajectoryGenerationService")));
	
	//phantomOmniCompLoader.Connect("PhantomOmniService", centerLoader.GetRequiredServicePort("PhantomOmniService"));
	centerLoader.GetRequiredServicePort("RobotArmControllerService")->setPeer(static_cast<ServicePort*>(robotArmController.getPort("RobotArmControllerService")));

	JoystickServiceRequired joystickService;
	WheelControllerServiceRequired wheelService;

	joystickCompLoader.Connect("JoystickService", &joystickService);
	wheelCompLoader.Connect("WheelControllerService", &wheelService);

	//컴포넌트 초기화
	robotArmController.initialize();
	leftIKLoader.Initialize();
	rightIKLoader.Initialize();
	leftGripperLoader.Initialize();
	rightGripperLoader.Initialize();
	trajectoryGenretorLoader.Initialize();
	//phantomOmniCompLoader.Initialize();
	centerLoader.Initialize();
	joystickCompLoader.Initialize();
	wheelCompLoader.Initialize();

	//컴포넌트 시작
	leftIKLoader.Start();
	rightIKLoader.Start();
	leftGripperLoader.Start();
	rightGripperLoader.Start();
	trajectoryGenretorLoader.Start();
	robotArmController.start();
	//phantomOmniCompLoader.Start(100);
	joystickCompLoader.Start();
	centerLoader.Start();
	wheelCompLoader.Start();

	//처리

	bool mode = true;
	bool previousButton = false;

	ObjectPosition offsetPosition(0.1, 0.0, 0.3, 90, 0, 90);
	const double scale = 0.01;
	for (;;)
	{
		auto controlData = joystickService.GetJoystickData();

		if (!previousButton && controlData.button[5])
		{
			mode = !mode;
		}

		previousButton = controlData.button[5];


		if (mode)
		{
			const double maxLinearVelocity = 0.2;
			const double maxAngularVelocity = 30;

			wheelService.DriveWheel(maxLinearVelocity * -controlData.y
				, maxAngularVelocity * -controlData.x);
		}
		else
		{

			if (robotArmController.GetManipulateState('L') != 0
				&& robotArmController.GetManipulateState('R') != 0)
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

				robotArmController.Manipulate('L', leftEndEffoctorPath);
				robotArmController.Manipulate('R', rightEndEffoctorPath);
			}

			//왼손
			switch(robotArmController.GetGripperState('L'))
			{
			case 1:
				if (!(controlData.button[0]))
					robotArmController.Grab('L', false);
				break;
			case -1:
				if (controlData.button[0])
					robotArmController.Grab('L', true);		
				break;
			}
			//오른손
			switch(robotArmController.GetGripperState('R'))
			{
			case 1:
				if (!(controlData.button[1]))
					robotArmController.Grab('R', false);
				break;
			case -1:
				if (controlData.button[1])
					robotArmController.Grab('R', true);		
				break;
			}
		}

		Sleep(100);
	}

	getchar();
	return 0;
}




#endif

