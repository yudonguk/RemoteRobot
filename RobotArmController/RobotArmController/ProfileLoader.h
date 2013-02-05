#ifndef __PROFILE_LOADER_H__
#define __PROFILE_LOADER_H__

#include <string>

#include <device/Property.h>

#include "Markup.h"

class ProfileLoader
{
public:
	ProfileLoader(){}
	ProfileLoader(const std::string& profileName_)
	{
		Load(profileName_);
	}
	~ProfileLoader(){}

public:
	void Load(const std::string& profileName_);
	
	inline std::string GetProfileName()
	{
		return profileName;
	}

	inline Property GetProperty()
	{
		return property;
	}

private:
	std::string profileName;
	CMarkup cmarkup;
	Property property;
};

void ProfileLoader::Load( const std::string& profileName_ )
{
	if (cmarkup.Load(profileName_) == false)
	{
		return; //�ε� ����
	}

	if (cmarkup.FindChildElem("properties") == false)
	{
		return; //properties�� ã�� ����
	}

	Property result;
	cmarkup.IntoElem();
	{
		for (; cmarkup.FindChildElem("property");)
		{
			std::string& name = cmarkup.GetChildAttrib("name");
			if (name == "")
			{
				continue; //name attribute ����
			}
			std::string& data = cmarkup.GetChildData();

			result.SetValue(name, data);
		}
	}
	cmarkup.OutOfElem();

	profileName = profileName_;
	property = result;
}


#endif