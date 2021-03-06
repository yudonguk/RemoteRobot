

#ifndef _InverseKinematicsService_REQUIRED_PORT_H
#define _InverseKinematicsService_REQUIRED_PORT_H

#include <OPRoSTypes.h>
#include <InputDataPort.h>
#include <OutputDataPort.h>
#include <InputEventPort.h>
#include <OutputEventPort.h>
#include <ProvidedServicePort.h>
#include <RequiredServicePort.h>
#include <ProvidedMethod.h>
#include <RequiredMethod.h>


#include <vector>
#include <device/Property.h>
#include <device/ApiTypes.h>
		



/*
 * 
 * InverseKinematicsService : public RequiredServicePort
 *
 */
class InverseKinematicsServiceRequired
   :public RequiredServicePort
{
protected:

	typedef RequiredMethod<bool> SetParameterFuncType;
	SetParameterFuncType *SetParameterFunc;

	typedef RequiredMethod<Property> GetParameterFuncType;
	GetParameterFuncType *GetParameterFunc;

	typedef RequiredMethod<int> GetErrorFuncType;
	GetErrorFuncType *GetErrorFunc;

	typedef RequiredMethod<bool> SetJointPositionFuncType;
	SetJointPositionFuncType *SetJointPositionFunc;

	typedef RequiredMethod<vector<double>> GetJointPositionFuncType;
	GetJointPositionFuncType *GetJointPositionFunc;

	typedef RequiredMethod<bool> SetDesiredPositionFuncType;
	SetDesiredPositionFuncType *SetDesiredPositionFunc;

	typedef RequiredMethod<ObjectPosition> GetCurrentPositionFuncType;
	GetCurrentPositionFuncType *GetCurrentPositionFunc;

public:
	//
	// Constructor
	//
	InverseKinematicsServiceRequired()
	{
            SetParameterFunc = NULL;
            GetParameterFunc = NULL;
            GetErrorFunc = NULL;
            SetJointPositionFunc = NULL;
            GetJointPositionFunc = NULL;
            SetDesiredPositionFunc = NULL;
            GetCurrentPositionFunc = NULL;
            
	   setup();
	}

	// method implementation for required method
	bool SetParameter(Property parameter)
	{
            opros_assert(SetParameterFunc != NULL);
	
            return (*SetParameterFunc)(parameter);
		
	}

	Property GetParameter()
	{
            opros_assert(GetParameterFunc != NULL);
	
            return (*GetParameterFunc)();
		
	}

	int GetError()
	{
            opros_assert(GetErrorFunc != NULL);
	
            return (*GetErrorFunc)();
		
	}

	bool SetJointPosition(vector<double> jointPosition)
	{
            opros_assert(SetJointPositionFunc != NULL);
	
            return (*SetJointPositionFunc)(jointPosition);
		
	}

	vector<double> GetJointPosition()
	{
            opros_assert(GetJointPositionFunc != NULL);
	
            return (*GetJointPositionFunc)();
		
	}

	bool SetDesiredPosition(ObjectPosition desiredPosition)
	{
            opros_assert(SetDesiredPositionFunc != NULL);
	
            return (*SetDesiredPositionFunc)(desiredPosition);
		
	}

	ObjectPosition GetCurrentPosition()
	{
            opros_assert(GetCurrentPositionFunc != NULL);
	
            return (*GetCurrentPositionFunc)();
		
	}

	

    // generated setup function
    virtual void setup()
    {
        Method *ptr_method;
    
        ptr_method = makeRequiredMethod(&InverseKinematicsServiceRequired::SetParameter,"SetParameter");
        opros_assert(ptr_method != NULL);
        addMethod("SetParameter",ptr_method);
        SetParameterFunc = reinterpret_cast<SetParameterFuncType *>(ptr_method);
        ptr_method = NULL;

    
        ptr_method = makeRequiredMethod(&InverseKinematicsServiceRequired::GetParameter,"GetParameter");
        opros_assert(ptr_method != NULL);
        addMethod("GetParameter",ptr_method);
        GetParameterFunc = reinterpret_cast<GetParameterFuncType *>(ptr_method);
        ptr_method = NULL;

    
        ptr_method = makeRequiredMethod(&InverseKinematicsServiceRequired::GetError,"GetError");
        opros_assert(ptr_method != NULL);
        addMethod("GetError",ptr_method);
        GetErrorFunc = reinterpret_cast<GetErrorFuncType *>(ptr_method);
        ptr_method = NULL;

    
        ptr_method = makeRequiredMethod(&InverseKinematicsServiceRequired::SetJointPosition,"SetJointPosition");
        opros_assert(ptr_method != NULL);
        addMethod("SetJointPosition",ptr_method);
        SetJointPositionFunc = reinterpret_cast<SetJointPositionFuncType *>(ptr_method);
        ptr_method = NULL;

    
        ptr_method = makeRequiredMethod(&InverseKinematicsServiceRequired::GetJointPosition,"GetJointPosition");
        opros_assert(ptr_method != NULL);
        addMethod("GetJointPosition",ptr_method);
        GetJointPositionFunc = reinterpret_cast<GetJointPositionFuncType *>(ptr_method);
        ptr_method = NULL;

    
        ptr_method = makeRequiredMethod(&InverseKinematicsServiceRequired::SetDesiredPosition,"SetDesiredPosition");
        opros_assert(ptr_method != NULL);
        addMethod("SetDesiredPosition",ptr_method);
        SetDesiredPositionFunc = reinterpret_cast<SetDesiredPositionFuncType *>(ptr_method);
        ptr_method = NULL;

    
        ptr_method = makeRequiredMethod(&InverseKinematicsServiceRequired::GetCurrentPosition,"GetCurrentPosition");
        opros_assert(ptr_method != NULL);
        addMethod("GetCurrentPosition",ptr_method);
        GetCurrentPositionFunc = reinterpret_cast<GetCurrentPositionFuncType *>(ptr_method);
        ptr_method = NULL;

    
    }
};
#endif
