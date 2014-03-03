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
	
    	auto enumerator = device_manager_->EnumerateDevices<OVR::HMDDevice>();
    	//Surprisingly, it is not needed to call enumerator.Next() here
    	deviceToReturn = enumerator.CreateDevice();

    	if (deviceToReturn != nullptr)
		{
			++numberOfDevicesCreated;
		}

		std::cout << "Next device is number " << numberOfDevicesCreated << std::endl;

		return deviceToReturn;
	}

	//Redefined function from the Oculus SDK in order to support multiple Oculus Rifts
	OVR::SensorDevice* OculusDeviceManager::getNextSensor()
	{
	    //Gets a sensor in order to attach it to a device

	    OVR::SensorDevice* sensor(nullptr);

	    auto enumerator = device_manager_->EnumerateDevices<OVR::SensorDevice>();

	    int i(1);
	    while (i < numberOfDevicesCreated)
	    {
	       enumerator.Next();
	       ++i;
	    }

	    sensor = enumerator.CreateDevice();

	    if (sensor)
	        sensor->SetCoordinateFrame(OVR::SensorDevice::Coord_HMD);
	    else
	    	--numberOfDevicesCreated;	// If no sensor was detected for a device, the number of devices created is decreased again
	    return sensor;
	}
}
