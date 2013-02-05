

#ifndef _RobotArmControllerService_PROVIDED_PORT_H
#define _RobotArmControllerService_PROVIDED_PORT_H

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
		

#ifndef _IRobotArmControllerService_CLASS_DEF
#define _IRobotArmControllerService_CLASS_DEF
/*
 * IRobotArmControllerService
 *
 *  The comonent inherits this class and implements methods. 
 */
class IRobotArmControllerService
{
 public:
    virtual void Manipulate(unsigned char index,std::vector<ObjectPosition> position)=0;
    virtual void Grab(unsigned char index,bool isEnable)=0;
    virtual int GetManipulateState(unsigned char index)=0;
    virtual int GetGripperState(unsigned char index)=0;
 };
#endif

/*
 * 
 * RobotArmControllerService : public ProvidedServicePort
 *
 */
class RobotArmControllerServiceProvided
	:public ProvidedServicePort, public IRobotArmControllerService
{
protected:
    IRobotArmControllerService *pcom;


   typedef ProvidedMethod<void> ManipulateFuncType;
   ManipulateFuncType *ManipulateFunc;

   typedef ProvidedMethod<void> GrabFuncType;
   GrabFuncType *GrabFunc;

   typedef ProvidedMethod<int> GetManipulateStateFuncType;
   GetManipulateStateFuncType *GetManipulateStateFunc;

   typedef ProvidedMethod<int> GetGripperStateFuncType;
   GetGripperStateFuncType *GetGripperStateFunc;


public: // default method
   virtual void Manipulate(unsigned char index,std::vector<ObjectPosition> position)
   {
		opros_assert(ManipulateFunc != NULL);
		(*ManipulateFunc)(index,position);
   }
   virtual void Grab(unsigned char index,bool isEnable)
   {
		opros_assert(GrabFunc != NULL);
		(*GrabFunc)(index,isEnable);
   }
   virtual int GetManipulateState(unsigned char index)
   {
		opros_assert(GetManipulateStateFunc != NULL);
		
            return (*GetManipulateStateFunc)(index);
		
   }
   virtual int GetGripperState(unsigned char index)
   {
		opros_assert(GetGripperStateFunc != NULL);
		
            return (*GetGripperStateFunc)(index);
		
   }


public:
    //
    // Constructor
    //
    RobotArmControllerServiceProvided(IRobotArmControllerService *fn)
    {
        pcom = fn;

        ManipulateFunc = NULL;
        GrabFunc = NULL;
        GetManipulateStateFunc = NULL;
        GetGripperStateFunc = NULL;
        

        setup();
    }

    // generated setup function
    virtual void setup()
    {
        Method *ptr_method;
    
        ptr_method = makeProvidedMethod(&IRobotArmControllerService::Manipulate,pcom,"Manipulate");

        opros_assert(ptr_method != NULL);
        addMethod("Manipulate",ptr_method);
        ManipulateFunc = reinterpret_cast<ManipulateFuncType *>(ptr_method);
        ptr_method = NULL;

    
        ptr_method = makeProvidedMethod(&IRobotArmControllerService::Grab,pcom,"Grab");

        opros_assert(ptr_method != NULL);
        addMethod("Grab",ptr_method);
        GrabFunc = reinterpret_cast<GrabFuncType *>(ptr_method);
        ptr_method = NULL;

    
        ptr_method = makeProvidedMethod(&IRobotArmControllerService::GetManipulateState,pcom,"GetManipulateState");

        opros_assert(ptr_method != NULL);
        addMethod("GetManipulateState",ptr_method);
        GetManipulateStateFunc = reinterpret_cast<GetManipulateStateFuncType *>(ptr_method);
        ptr_method = NULL;

    
        ptr_method = makeProvidedMethod(&IRobotArmControllerService::GetGripperState,pcom,"GetGripperState");

        opros_assert(ptr_method != NULL);
        addMethod("GetGripperState",ptr_method);
        GetGripperStateFunc = reinterpret_cast<GetGripperStateFuncType *>(ptr_method);
        ptr_method = NULL;

    
    }
};
#endif
