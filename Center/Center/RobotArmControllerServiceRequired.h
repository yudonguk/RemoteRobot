

#ifndef _RobotArmControllerService_REQUIRED_PORT_H
#define _RobotArmControllerService_REQUIRED_PORT_H

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
		



/*
 * 
 * RobotArmControllerService : public RequiredServicePort
 *
 */
class RobotArmControllerServiceRequired
   :public RequiredServicePort
{
protected:

	typedef RequiredMethod<void> ManipulateFuncType;
	ManipulateFuncType *ManipulateFunc;

	typedef RequiredMethod<void> GrabFuncType;
	GrabFuncType *GrabFunc;

	typedef RequiredMethod<int> GetManipulateStateFuncType;
	GetManipulateStateFuncType *GetManipulateStateFunc;

	typedef RequiredMethod<int> GetGripperStateFuncType;
	GetGripperStateFuncType *GetGripperStateFunc;

public:
	//
	// Constructor
	//
	RobotArmControllerServiceRequired()
	{
            ManipulateFunc = NULL;
            GrabFunc = NULL;
            GetManipulateStateFunc = NULL;
            GetGripperStateFunc = NULL;
            
	   setup();
	}

	// method implementation for required method
	void Manipulate(unsigned char index,std::vector<ObjectPosition> position)
	{
            opros_assert(ManipulateFunc != NULL);
	
            (*ManipulateFunc)(index,position);
		
	}

	void Grab(unsigned char index,bool isEnable)
	{
            opros_assert(GrabFunc != NULL);
	
            (*GrabFunc)(index,isEnable);
		
	}

	int GetManipulateState(unsigned char index)
	{
            opros_assert(GetManipulateStateFunc != NULL);
	
            return (*GetManipulateStateFunc)(index);
		
	}

	int GetGripperState(unsigned char index)
	{
            opros_assert(GetGripperStateFunc != NULL);
	
            return (*GetGripperStateFunc)(index);
		
	}

	

    // generated setup function
    virtual void setup()
    {
        Method *ptr_method;
    
        ptr_method = makeRequiredMethod(&RobotArmControllerServiceRequired::Manipulate,"Manipulate");
        opros_assert(ptr_method != NULL);
        addMethod("Manipulate",ptr_method);
        ManipulateFunc = reinterpret_cast<ManipulateFuncType *>(ptr_method);
        ptr_method = NULL;

    
        ptr_method = makeRequiredMethod(&RobotArmControllerServiceRequired::Grab,"Grab");
        opros_assert(ptr_method != NULL);
        addMethod("Grab",ptr_method);
        GrabFunc = reinterpret_cast<GrabFuncType *>(ptr_method);
        ptr_method = NULL;

    
        ptr_method = makeRequiredMethod(&RobotArmControllerServiceRequired::GetManipulateState,"GetManipulateState");
        opros_assert(ptr_method != NULL);
        addMethod("GetManipulateState",ptr_method);
        GetManipulateStateFunc = reinterpret_cast<GetManipulateStateFuncType *>(ptr_method);
        ptr_method = NULL;

    
        ptr_method = makeRequiredMethod(&RobotArmControllerServiceRequired::GetGripperState,"GetGripperState");
        opros_assert(ptr_method != NULL);
        addMethod("GetGripperState",ptr_method);
        GetGripperStateFunc = reinterpret_cast<GetGripperStateFuncType *>(ptr_method);
        ptr_method = NULL;

    
    }
};
#endif
