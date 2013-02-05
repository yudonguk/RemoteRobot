

#ifndef _PhantomOmniService_REQUIRED_PORT_H
#define _PhantomOmniService_REQUIRED_PORT_H

#include <OPRoSTypes.h>
#include <InputDataPort.h>
#include <OutputDataPort.h>
#include <InputEventPort.h>
#include <OutputEventPort.h>
#include <ProvidedServicePort.h>
#include <RequiredServicePort.h>
#include <ProvidedMethod.h>
#include <RequiredMethod.h>


#include <ApiTypes.h>


/*
 * 
 * PhantomOmniService : public RequiredServicePort
 *
 */
class PhantomOmniServiceRequired
   :public RequiredServicePort
{
protected:

	typedef RequiredMethod<ObjectPosition> GetPositionFuncType;
	GetPositionFuncType *GetPositionFunc;

	typedef RequiredMethod<vector<double>> GetJointAngleFuncType;
	GetJointAngleFuncType *GetJointAngleFunc;

	typedef RequiredMethod<float> GetForceFuncType;
	GetForceFuncType *GetForceFunc;

	typedef RequiredMethod<bool> SetForceFuncType;
	SetForceFuncType *SetForceFunc;

	typedef RequiredMethod<bool> StartSpringEffectFuncType;
	StartSpringEffectFuncType *StartSpringEffectFunc;

	typedef RequiredMethod<bool> StopEffectFuncType;
	StopEffectFuncType *StopEffectFunc;

	typedef RequiredMethod<PhantomButtonState> GetButtonStateFuncType;
	GetButtonStateFuncType *GetButtonStateFunc;

	typedef RequiredMethod<bool> CalibrationFuncType;
	CalibrationFuncType *CalibrationFunc;

    typedef ProvidedMethod<bool> StartVibrationEffectFuncType;
    StartVibrationEffectFuncType *StartVibrationEffectFunc;

public:
	//
	// Constructor
	//
	PhantomOmniServiceRequired()
	{
            GetPositionFunc = NULL;
            GetJointAngleFunc = NULL;
            GetForceFunc = NULL;
            SetForceFunc = NULL;
            StartSpringEffectFunc = NULL;
            StopEffectFunc = NULL;
            GetButtonStateFunc = NULL;
            CalibrationFunc = NULL;
            StartVibrationEffectFunc = NULL;

	   setup();
	}

	// method implementation for required method
	ObjectPosition GetPosition()
	{
            opros_assert(GetPositionFunc != NULL);
	
            return (*GetPositionFunc)();
		
	}

	vector<double> GetJointAngle()
	{
            opros_assert(GetJointAngleFunc != NULL);
	
            return (*GetJointAngleFunc)();
		
	}

	float GetForce()
	{
            opros_assert(GetForceFunc != NULL);
	
            return (*GetForceFunc)();
		
	}

	bool SetForce(float force)
	{
            opros_assert(SetForceFunc != NULL);
	
            return (*SetForceFunc)(force);
	}

	bool StartSpringEffect(float x,float y,float z)
	{
            opros_assert(StartSpringEffectFunc != NULL);
	
            return (*StartSpringEffectFunc)(x,y,z);
		
	}

	bool StopEffect()
	{
            opros_assert(StopEffectFunc != NULL);
	
            return (*StopEffectFunc)();
		
	}

	PhantomButtonState GetButtonState()
	{
            opros_assert(GetButtonStateFunc != NULL);
	
            return (*GetButtonStateFunc)();
		
	}

	bool Calibration()
	{
            opros_assert(CalibrationFunc != NULL);
	
            return (*CalibrationFunc)();
		
	}

    bool StartVibrationEffect(float frequency)
    {
        opros_assert(StartVibrationEffectFunc != NULL);

        return (*StartVibrationEffectFunc)();

    }

    // generated setup function
    virtual void setup()
    {
        Method *ptr_method;
    
        ptr_method = makeRequiredMethod(&PhantomOmniServiceRequired::GetPosition,"GetPosition");
        opros_assert(ptr_method != NULL);
        addMethod("GetPosition",ptr_method);
        GetPositionFunc = reinterpret_cast<GetPositionFuncType *>(ptr_method);
        ptr_method = NULL;

    
        ptr_method = makeRequiredMethod(&PhantomOmniServiceRequired::GetJointAngle,"GetJointAngle");
        opros_assert(ptr_method != NULL);
        addMethod("GetJointAngle",ptr_method);
        GetJointAngleFunc = reinterpret_cast<GetJointAngleFuncType *>(ptr_method);
        ptr_method = NULL;

    
        ptr_method = makeRequiredMethod(&PhantomOmniServiceRequired::GetForce,"GetForce");
        opros_assert(ptr_method != NULL);
        addMethod("GetForce",ptr_method);
        GetForceFunc = reinterpret_cast<GetForceFuncType *>(ptr_method);
        ptr_method = NULL;

    
        ptr_method = makeRequiredMethod(&PhantomOmniServiceRequired::SetForce,"SetForce");
        opros_assert(ptr_method != NULL);
        addMethod("SetForce",ptr_method);
        SetForceFunc = reinterpret_cast<SetForceFuncType *>(ptr_method);
        ptr_method = NULL;

    
        ptr_method = makeRequiredMethod(&PhantomOmniServiceRequired::StartSpringEffect,"StartSpringEffect");
        opros_assert(ptr_method != NULL);
        addMethod("StartSpringEffect",ptr_method);
        StartSpringEffectFunc = reinterpret_cast<StartSpringEffectFuncType *>(ptr_method);
        ptr_method = NULL;
    
        ptr_method = makeRequiredMethod(&PhantomOmniServiceRequired::StopEffect,"StopEffect");
        opros_assert(ptr_method != NULL);
        addMethod("StopEffect",ptr_method);
        StopEffectFunc = reinterpret_cast<StopEffectFuncType *>(ptr_method);
        ptr_method = NULL;

    
        ptr_method = makeRequiredMethod(&PhantomOmniServiceRequired::GetButtonState,"GetButtonState");
        opros_assert(ptr_method != NULL);
        addMethod("GetButtonState",ptr_method);
        GetButtonStateFunc = reinterpret_cast<GetButtonStateFuncType *>(ptr_method);
        ptr_method = NULL;

    
        ptr_method = makeRequiredMethod(&PhantomOmniServiceRequired::Calibration,"Calibration");
        opros_assert(ptr_method != NULL);
        addMethod("Calibration",ptr_method);
        CalibrationFunc = reinterpret_cast<CalibrationFuncType *>(ptr_method);
        ptr_method = NULL;

        ptr_method = makeRequiredMethod(&PhantomOmniServiceRequired::StartVibrationEffect,"StartVibrationEffect");
        opros_assert(ptr_method != NULL);
        addMethod("StartVibrationEffect",ptr_method);
        StartVibrationEffectFunc = reinterpret_cast<StartVibrationEffectFuncType *>(ptr_method);
        ptr_method = NULL;

    
    }
};
#endif
