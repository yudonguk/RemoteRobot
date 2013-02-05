/*************************************************************

 file: JoystickComp.cpp
 author: E.C. Shin
 begin: January 30 2010
 copyright: (c) 2010 KITECH, OPRoS
 email: unchol@kitech.re.kr

***************************************************************

OPRoS source code is provided under a dual license mode:
 LGPL and OPRoS individually.

LGPL: You can redistribute it and/or modify it under the terms
 of the Less GNU General Public License as published by the Free
 Software Foundation, either version 3 of the License.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of 
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
 Less GNU General Public License for more details. 

 You should have received a copy of the Less GNU General Public
 License along with this program. If not, see 
 <http://www.gnu.org/licenses/>.

OPRoS individual license: An individual license is a license for
 modifying the source code of OPRoS and distiributing it in a
 closed format for commercial purposes.

 If you are interested in this option, please see 
 <http://www.opros.or.kr>.

This license policy may be changed without prior notice.

***************************************************************/
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



#include "JoystickComp.h"
#include "OprosPrintMessage.h"

//
// constructor declaration
//
JoystickComp::JoystickComp()
{
	hOprosAPI = NULL;
	joystick = NULL;
	lastError = OPROS_SUCCESS;

	portSetup();
}

//
// constructor declaration (with name)
//
JoystickComp::JoystickComp(const std::string &name):Component(name)
{
	hOprosAPI = NULL;
	joystick = NULL;
	lastError = OPROS_SUCCESS;
	
	portSetup();
}

//
// destructor declaration
//

JoystickComp::~JoystickComp() {
	onDestroy();
}

JoystickData JoystickComp::GetJoystickData()
{
	JoystickData result;

	if(joystick == NULL) {
		lastError = OPROS_PRECONDITION_NOT_MET;
		return result;
	}

	lock.Lock();
	if(joystick->GetJoystickData(result) < 0) {
		PrintMessage("ERROR : JoystickComp::GetJoystickData() -> Can't get the Joystick data.\n");
		lastError = OPROS_ENABLE_API_ERROR;
	}
	lock.Unlock();

	return result;
}

ReturnType JoystickComp::GetError()
{
	return lastError;
}

Property JoystickComp::GetParameter()
{
	Property parameter;
	lastError = OPROS_SUCCESS;

	if(joystick == NULL) {
		lastError = OPROS_PRECONDITION_NOT_MET;
		return parameter;
	}

	if(joystick->GetParameter(parameter) < 0) {
		lastError = OPROS_ENABLE_API_ERROR;
	}

	return this->parameter;
}

ReturnType JoystickComp::SetParameter(Property parameter)
{
	if(joystick == NULL) {
		return OPROS_PRECONDITION_NOT_MET;
	}

	if(joystick->SetParameter(parameter) < 0) {
		return OPROS_ENABLE_API_ERROR;
	}

	return OPROS_SUCCESS;
}


void JoystickComp::portSetup() {
	ProvidedServicePort *pa1;
	pa1=new JoystickServiceProvided(this);
	addPort("JoystickService",pa1);

//data port setup
	addPort("joystickData", &joystickData);




	// export variable setup


}

// Call back Declaration
ReturnType JoystickComp::onInitialize()
{
	//	XML에 저장된 프라퍼티를 parameter에 저장
	Property parameter;
	std::map<std::string, std::string> temp = getPropertyMap();
	parameter.SetProperty(temp);
	
	//	dll 파일이름을 확인하여 없으면 에러 리턴
	if(parameter.FindName("APIName") == false) {
		PrintMessage("ERROR : JoystickComp::onInitialize() -> Can't find the APIName in property\n");
		return OPROS_FIND_PROPERTY_ERROR;
	}

#if defined(WIN32)
	//	DLL 로드
	hOprosAPI = LoadLibrary((LPCSTR)parameter.GetValue("APIName").c_str());
	if(hOprosAPI == NULL) {
		PrintMessage("ERROR : JoystickComp::onInitialize() -> Can't find the %s\n", parameter.GetValue("APIName").c_str());
		return OPROS_FIND_DLL_ERROR;
	}
	
	//	API 로드
	GET_OPROS_API getOprosAPI;
	getOprosAPI = (GET_OPROS_API)GetProcAddress(hOprosAPI, "GetAPI");
	if(getOprosAPI == NULL) {
		PrintMessage("ERROR : JoystickComp::onInitialize() -> Can't get a handle of GetAPI Funtion\n");
		FreeLibrary(hOprosAPI);
		hOprosAPI = NULL;
		return OPROS_LOAD_DLL_ERROR;
	}
#else
	hOprosAPI = dlopen(parameter.GetValue("APIName").c_str(), RTLD_LAZY);
	if(hOprosAPI == NULL) {
		PrintMessage("ERROR : JoystickComp::onInitialize() -> Can't find the %s\n", parameter.GetValue("APIName").c_str());
		return OPROS_FIND_DLL_ERROR;
	}

	GET_OPROS_API getOprosAPI;
	getOprosAPI = (GET_OPROS_API)dlsym(hOprosAPI, "GetAPI");
	char *lastError = dlerror();
	if(lastError != NULL) {
		PrintMessage("ERROR : JoystickComp::onInitialize() -> Can't get a handle of GetAPI Funtion\n");
		dlclose(hOprosAPI);
		hOprosAPI = NULL;
		return OPROS_LOAD_DLL_ERROR;
	}
#endif
	
	joystick = static_cast<Joystick *>(getOprosAPI());
	if(joystick == NULL) {
		PrintMessage("ERROR : JoystickComp::onInitialize() -> Can't get a handle of Joystick API\n");
#if defined(WIN32)
		FreeLibrary(hOprosAPI);
#else
		dlclose(hOprosAPI);
#endif
		hOprosAPI = NULL;
		return OPROS_LOAD_DLL_ERROR;
	}

	//	API 초기화
	if(joystick->Initialize(parameter) != API_SUCCESS) {
		PrintMessage("ERROR : JoystickComp::onInitialize() -> Can't intilaize a Joystick API\n");
		
		delete joystick;
		joystick = NULL;

#if defined(WIN32)
		FreeLibrary(hOprosAPI);
#else
		dlclose(hOprosAPI);
#endif
		hOprosAPI = NULL;
		return OPROS_INITIALIZE_API_ERROR;
	}

	lastError = OPROS_SUCCESS;

	PrintMessage("SUCCESS : JoystickComp::onInitialize()\n");
	return OPROS_SUCCESS;
}

ReturnType JoystickComp::onStart()
{
	PrintMessage("SUCCESS : JoystickComp::onStart()1\n");
	if(joystick == NULL) {
		return OPROS_PRECONDITION_NOT_MET;
	}
	PrintMessage("SUCCESS : JoystickComp::onStart()2\n");

	//	API의 enable를 호출
	if(joystick->Enable() < 0) {
		PrintMessage("ERROR : JoystickComp::onStart() -> Can't enable the Joystick API\n");
		return OPROS_ENABLE_API_ERROR;
	}
	PrintMessage("SUCCESS : JoystickComp::onStart()3\n");

	return OPROS_SUCCESS;
}
	
ReturnType JoystickComp::onStop()
{
	if(joystick == NULL) {
		return OPROS_PRECONDITION_NOT_MET;
	}

	//	API의 disable 호출
	if(joystick->Disable() < 0) {
		PrintMessage("ERROR : JoystickComp::onStop() -> Can't disable the Joystick API\n");
		return OPROS_DISABLE_API_ERROR;
	}

	return OPROS_SUCCESS;
}

ReturnType JoystickComp::onReset()
{
	// user code here
	return OPROS_SUCCESS;
}

ReturnType JoystickComp::onError()
{
	// user code here
	return OPROS_SUCCESS;
}

ReturnType JoystickComp::onRecover()
{
	// user code here
	return OPROS_SUCCESS;
}

ReturnType JoystickComp::onDestroy()
{
	//	API 핸들 삭제
	if(joystick != NULL) {
		joystick->Finalize();
		delete joystick;
		joystick = NULL;
	}

	//	DLL 핸들 삭제
	if(hOprosAPI != NULL) {
#if defined(WIN32)
		FreeLibrary(hOprosAPI);
#else
		dlclose(hOprosAPI);
#endif
		hOprosAPI = NULL;
	}

	return OPROS_SUCCESS;
}

ReturnType JoystickComp::onEvent(Event *evt)
{
	// user code here
	return OPROS_SUCCESS;
}

ReturnType JoystickComp::onExecute()
{
	if(joystick == NULL) {
		return OPROS_PRECONDITION_NOT_MET;
	}

	JoystickData result;

	lock.Lock();
	if(joystick->GetJoystickData(result) < 0) {
		PrintMessage("ERROR : JoystickComp::onExecute() -> Can't get the Joystick data.\n");
		lock.Unlock();
		return OPROS_CALL_API_ERROR;
	}
	lock.Unlock();

	joystickData.push(result);

	return OPROS_SUCCESS;
}
	
ReturnType JoystickComp::onUpdated()
{
	// user code here
	return OPROS_SUCCESS;
}

ReturnType JoystickComp::onPeriodChanged()
{
	// user code here
	return OPROS_SUCCESS;
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
	return new JoystickComp();
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
	return new JoystickComp();
}

void releaseComponent(Component *com)
{
	delete com;
}
#endif
#endif

