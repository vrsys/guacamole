#ifndef GUA_OCULUS_DEVICE_MANAGER_HPP
#define GUA_OCULUS_DEVICE_MANAGER_HPP

#if defined (_MSC_VER)
  #if defined (GUA_OCULUS_LIBRARY)
    #define GUA_OCULUS_DLL __declspec( dllexport )
  #else
#define GUA_OCULUS_DLL __declspec( dllimport )
  #endif
#else
  #define GUA_OCULUS_DLL
#endif // #if defined(_MSC_VER)

namespace OVR {
  class SensorFusion;
  class DeviceManager;
  class HMDDevice;
  class SensorDevice;
}

namespace gua
{
	class GUA_OCULUS_DLL OculusDeviceManager
	{
		public:
			static OculusDeviceManager& getInstance();
			virtual ~OculusDeviceManager();

			OVR::HMDDevice* getNextAvailableDevice();

		protected:
			OculusDeviceManager();

		private:
			OVR::DeviceManager* device_manager_;
			static int numberOfDevicesCreated;

			OculusDeviceManager(OculusDeviceManager const& src);
			OculusDeviceManager& operator=(OculusDeviceManager const& src);
	};
}

#endif // GUA_OCULUS_DEVICE_MANAGER_HPP