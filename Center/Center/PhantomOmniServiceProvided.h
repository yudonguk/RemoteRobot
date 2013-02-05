

#ifndef _PhantomOmniService_PROVIDED_PORT_H
#define _PhantomOmniService_PROVIDED_PORT_H

#include <OPRoSTypes.h>
#include <InputDataPort.h>
#include <OutputDataPort.h>
#include <InputEventPort.h>
#include <OutputEventPort.h>
#include <ProvidedServicePort.h>
#include <RequiredServicePort.h>
#include <ProvidedMethod.h>
#include <RequiredMethod.h>



#ifndef _IPhantomOmniService_CLASS_DEF
#define _IPhantomOmniService_CLASS_DEF
/*
 * IPhantomOmniService
 *
 *  The comonent inherits this class and implements methods. 
 */
class IPhantomOmniService
{
 public:
    virtual ObjectPosition GetPosition()=0;
    virtual vector<double> GetJointAngle()=0;
    virtual float GetForce()=0;
    virtual bool SetForce(float force)=0;
    virtual bool StartSpringEffect(float x,float y,float z)=0;
    virtual bool StopEffect()=0;
    virtual PhantomButtonState GetButtonState()=0;
    virtual bool Calibration()=0;
    virtual bool StartVibrationEffect(float frequency)=0;
 };
#endif

/*
 * 
 * PhantomOmniService : public ProvidedServicePort
 *
 */
class PhantomOmniServiceProvided
	:public ProvidedServicePort, public IPhantomOmniService
{
protected:
    IPhantomOmniService *pcom;


   typedef ProvidedMethod<ObjectPosition> GetPositionFuncType;
   GetPositionFuncType *GetPositionFunc;

   typedef ProvidedMethod<vector<double>> GetJointAngleFuncType;
   GetJointAngleFuncType *GetJointAngleFunc;

   typedef ProvidedMethod<float> GetForceFuncType;
   GetForceFuncType *GetForceFunc;

   typedef ProvidedMethod<bool> SetForceFuncType;
   SetForceFuncType *SetForceFunc;

   typedef ProvidedMethod<bool> StartSpringEffectFuncType;
   StartSpringEffectFuncType *StartSpringEffectFunc;

   typedef ProvidedMethod<bool> StopEffectFuncType;
   StopEffectFuncType *StopEffectFunc;

   typedef ProvidedMethod<PhantomButtonState> GetButtonStateFuncType;
   GetButtonStateFuncType *GetButtonStateFunc;

   typedef ProvidedMethod<bool> CalibrationFuncType;
   CalibrationFuncType *CalibrationFunc;

   typedef ProvidedMethod<bool> StartVibrationEffectFuncType;
   StartVibrationEffectFuncType *StartVibrationEffectFunc;


public: // default method
   virtual ObjectPosition GetPosition()
   {
		opros_assert(GetPositionFunc != NULL);
		
            return (*GetPositionFunc)();
		
   }
   virtual vector<double> GetJointAngle()
   {
		opros_assert(GetJointAngleFunc != NULL);
		
            return (*GetJointAngleFunc)();
		
   }
   virtual float GetForce()
   {
		opros_assert(GetForceFunc != NULL);
		
            return (*GetForceFunc)();
		
   }
   virtual bool SetForce(float force)
   {
		opros_assert(SetForceFunc != NULL);
		
            return (*SetForceFunc)(force);
		
   }
   virtual bool StartSpringEffect(float x,float y,float z)
   {
		opros_assert(StartSpringEffectFunc != NULL);
		
            return (*StartSpringEffectFunc)(x,y,z);
		
   }
   virtual bool StopEffect()
   {
		opros_assert(StopEffectFunc != NULL);
		
            return (*StopEffectFunc)();
		
   }
   virtual PhantomButtonState GetButtonState()
   {
		opros_assert(GetButtonStateFunc != NULL);
		
            return (*GetButtonStateFunc)();
		
   }
   virtual bool Calibration()
   {
		opros_assert(CalibrationFunc != NULL);
		
            return (*CalibrationFunc)();
		
   }
   virtual bool StartVibrationEffect(float frequency)
   {
		opros_assert(StartVibrationEffectFunc != NULL);
		
            return (*StartVibrationEffectFunc)(frequency);
		
   }


public:
    //
    // Constructor
    //
    PhantomOmniServiceProvided(IPhantomOmniService *fn)
    {
        pcom = fn;

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

    // generated setup function
    virtual void setup()
    {
        Method *ptr_method;
    
        ptr_method = makeProvidedMethod(&IPhantomOmniService::GetPosition,pcom,"GetPosition");

        opros_assert(ptr_method != NULL);
        addMethod("GetPosition",ptr_method);
        GetPositionFunc = reinterpret_cast<GetPositionFuncType *>(ptr_method);
        ptr_method = NULL;

    
        ptr_method = makeProvidedMethod(&IPhantomOmniService::GetJointAngle,pcom,"GetJointAngle");

        opros_assert(ptr_method != NULL);
        addMethod("GetJointAngle",ptr_method);
        GetJointAngleFunc = reinterpret_cast<GetJointAngleFuncType *>(ptr_method);
        ptr_method = NULL;

    
        ptr_method = makeProvidedMethod(&IPhantomOmniService::GetForce,pcom,"GetForce");

        opros_assert(ptr_method != NULL);
        addMethod("GetForce",ptr_method);
        GetForceFunc = reinterpret_cast<GetForceFuncType *>(ptr_method);
        ptr_method = NULL;

    
        ptr_method = makeProvidedMethod(&IPhantomOmniService::SetForce,pcom,"SetForce");

        opros_assert(ptr_method != NULL);
        addMethod("SetForce",ptr_method);
        SetForceFunc = reinterpret_cast<SetForceFuncType *>(ptr_method);
        ptr_method = NULL;

    
        ptr_method = makeProvidedMethod(&IPhantomOmniService::StartSpringEffect,pcom,"StartSpringEffect");

        opros_assert(ptr_method != NULL);
        addMethod("StartSpringEffect",ptr_method);
        StartSpringEffectFunc = reinterpret_cast<StartSpringEffectFuncType *>(ptr_method);
        ptr_method = NULL;

    
        ptr_method = makeProvidedMethod(&IPhantomOmniService::StopEffect,pcom,"StopEffect");

        opros_assert(ptr_method != NULL);
        addMethod("StopEffect",ptr_method);
        StopEffectFunc = reinterpret_cast<StopEffectFuncType *>(ptr_method);
        ptr_method = NULL;

    
        ptr_method = makeProvidedMethod(&IPhantomOmniService::GetButtonState,pcom,"GetButtonState");

        opros_assert(ptr_method != NULL);
        addMethod("GetButtonState",ptr_method);
        GetButtonStateFunc = reinterpret_cast<GetButtonStateFuncType *>(ptr_method);
        ptr_method = NULL;

    
        ptr_method = makeProvidedMethod(&IPhantomOmniService::Calibration,pcom,"Calibration");

        opros_assert(ptr_method != NULL);
        addMethod("Calibration",ptr_method);
        CalibrationFunc = reinterpret_cast<CalibrationFuncType *>(ptr_method);
        ptr_method = NULL;

    
        ptr_method = makeProvidedMethod(&IPhantomOmniService::StartVibrationEffect,pcom,"StartVibrationEffect");

        opros_assert(ptr_method != NULL);
        addMethod("StartVibrationEffect",ptr_method);
        StartVibrationEffectFunc = reinterpret_cast<StartVibrationEffectFuncType *>(ptr_method);
        ptr_method = NULL;

    
    }
};
#endif
