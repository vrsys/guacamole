// class header
#include <gua/OculusDeviceManager.hpp>

// external headers
#include <OVR.h>

#include <iostream>

namespace gua
{
	int OculusDeviceManager::numberOfDevicesCreated(0);

	OculusDeviceManager& OculusDeviceManager::getInstance()
	{
		static OculusDeviceManager instance;
		return instance;
	}

	OculusDeviceManager::OculusDeviceManager()
	{ 
		device_manager_ = OVR::DeviceManager::Create();
	}

	/* virtual */ OculusDeviceManager::~OculusDeviceManager() 
	{
		if (device_manager_)
			delete device_manager_;
	}

	OVR::HMDDevice* OculusDeviceManager::getNextAvailableDevice()
	{
		OVR::HMDDevice* deviceToReturn(nullptr);
    
    std::cout << "Getting some device" << std::endl;
  
		if (numberOfDevicesCreated == 0)
		{
      std::cout << "Getting first device" << std::endl;
			deviceToReturn = device_manager_->EnumerateDevices<OVR::HMDDevice>().CreateDevice();
		}
		else if (numberOfDevicesCreated == 1)
		{
      std::cout << "Getting second device" << std::endl;
			auto enumerator = device_manager_->EnumerateDevices<OVR::HMDDevice>();
			enumerator.Next();
			deviceToReturn = enumerator.CreateDevice();
		}

		if (deviceToReturn != nullptr)
			++numberOfDevicesCreated;

		/*	// Probably a better solution, to be used instead of the upper one as it supports more than 2 devices
			auto enumerator = device_manager_->EnumerateDevices<OVR::HMDDevice>();
			int currentDevice(1);
			bool deviceAvailable(true);

			while(currentDevice < numberOfDevicesCreated + 1)
			{
				if (enumerator.GetType() == Device_None)
				{
					deviceAvailable = false;
					break;
				}
				
				enumerator.Next();
				++currentDevice;
			}
			
			if (deviceAvailable)
			{
				deviceToReturn = enumerator.CreateDevice();
				++numberOfDevicesCreated;
			}	
		*/

		return deviceToReturn;
	}
}
