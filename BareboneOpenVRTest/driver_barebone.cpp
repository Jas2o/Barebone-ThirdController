//============ Copyright (c) Valve Corporation, All rights reserved. ============

#include <openvr_driver.h>
#include <cstdio>
#include "driverlog.h"

#include <vector>
#include <thread>
#include <chrono>
#include <iostream>
#include <atlbase.h>

#include <map>
#include <process.h>
#include <tchar.h>
#include <thread>

#include "Gamepad.h"

//#if defined( _WINDOWS )
#include <windows.h>
//#endif

using namespace vr;
using namespace std;

typedef struct _Controller
{
	double	X;
	double	Y;
	double	Z;
	double	Yaw;
	double	Pitch;
	double	Roll;
} TController, * PController;

#if defined(_WIN32)
#define HMD_DLL_EXPORT extern "C" __declspec( dllexport )
#define HMD_DLL_IMPORT extern "C" __declspec( dllimport )
#elif defined(__GNUC__) || defined(COMPILER_GCC) || defined(__APPLE__)
#define HMD_DLL_EXPORT extern "C" __attribute__((visibility("default")))
#define HMD_DLL_IMPORT extern "C" 
#else
#error "Unsupported Platform."
#endif

bool ctrl = true;
TController MyCtrl;

inline HmdQuaternion_t HmdQuaternion_Init(double w, double x, double y, double z)
{
	HmdQuaternion_t quat;
	quat.w = w;
	quat.x = x;
	quat.y = y;
	quat.z = z;
	return quat;
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
class CBarebonesControllerDriver : public vr::ITrackedDeviceServerDriver
{
	//int32_t ControllerIndex;
public:
	bool Active;
	uint32_t FollowIndex;

	CBarebonesControllerDriver()
	{
		Active = false;
		FollowIndex = 0;
		CtrlIndex_t = vr::k_unTrackedDeviceIndexInvalid;
		m_ulPropertyContainer = vr::k_ulInvalidPropertyContainer;
	}

	virtual ~CBarebonesControllerDriver()
	{
	}

	// Purpose: Calculates quaternion (qw,qx,qy,qz) representing the rotation
	// from: https://github.com/Omnifinity/OpenVR-Tracking-Example/blob/master/HTC%20Lighthouse%20Tracking%20Example/LighthouseTracking.cpp
	//-----------------------------------------------------------------------------

	vr::HmdQuaternion_t GetRotation(vr::HmdMatrix34_t matrix) {
		vr::HmdQuaternion_t q;

		q.w = sqrt(fmax(0, 1 + matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2])) / 2;
		q.x = sqrt(fmax(0, 1 + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2])) / 2;
		q.y = sqrt(fmax(0, 1 - matrix.m[0][0] + matrix.m[1][1] - matrix.m[2][2])) / 2;
		q.z = sqrt(fmax(0, 1 - matrix.m[0][0] - matrix.m[1][1] + matrix.m[2][2])) / 2;
		q.x = copysign(q.x, matrix.m[2][1] - matrix.m[1][2]);
		q.y = copysign(q.y, matrix.m[0][2] - matrix.m[2][0]);
		q.z = copysign(q.z, matrix.m[1][0] - matrix.m[0][1]);
		return q;
	}

	/*
	virtual void SetControllerIndex(int32_t CtrlIndex)
	{
		ControllerIndex = CtrlIndex;
	}
	*/

	/** This is called before an HMD is returned to the application. It will always be
	* called before any display or tracking methods. Memory and processor use by the
	* ITrackedDeviceServerDriver object should be kept to a minimum until it is activated.
	* The pose listener is guaranteed to be valid until Deactivate is called, but
	* should not be used after that point. */
	virtual EVRInitError Activate(vr::TrackedDeviceIndex_t unObjectId)
	{
		CtrlIndex_t = unObjectId;
		m_ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(CtrlIndex_t);

		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_SerialNumber_String, "CTRL1Serial");

		vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, Prop_ControllerRoleHint_Int32, TrackedControllerRole_OptOut);

		uint64_t supportedButtons = 0xA4;
		vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, vr::Prop_SupportedButtons_Uint64, supportedButtons);


		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ControllerType_String, "vive_controller");
		//vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_LegacyInputProfile_String, "vive_controller");

		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ModelNumber_String, "ViveMV");
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ManufacturerName_String, "HTC");
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_RenderModelName_String, "vr_controller_vive_1_5");

		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_TrackingSystemName_String, "VR Controller");

		vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, Prop_CurrentUniverseId_Uint64, 2218716);

		// avoid "not fullscreen" warnings from vrmonitor
		vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, Prop_IsOnDesktop_Bool, false);

		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_InputProfilePath_String, "{barebones}/input/barebones_profile.json");

		bool bSetupIconUsingExternalResourceFile = false;
		if (!bSetupIconUsingExternalResourceFile)
		{
			// Setup properties directly in code.
			// Path values are of the form {drivername}\icons\some_icon_filename.png
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceOff_String, "{barebones}/icons/barebones_status_off.png");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceReady_String, "{barebones}/icons/barebones_status_ready.png");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceReadyAlert_String, "{barebones}/icons/barebones_status_ready_alert.png");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceNotReady_String, "{barebones}/icons/barebones_status_error.png");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceStandby_String, "{barebones}/icons/barebones_status_standby.png");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceAlertLow_String, "{barebones}/icons/barebones_status_ready_low.png");
		}

		vr::VRDriverInput()->CreateScalarComponent(
			m_ulPropertyContainer, "/input/trackpad/x", &leftJoystickXInputHandle,
			vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided
		);
		vr::VRDriverInput()->CreateScalarComponent(
			m_ulPropertyContainer, "/input/trackpad/y", &leftJoystickYInputHandle,
			vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided
		);

		vr::VRDriverInput()->CreateScalarComponent(
			m_ulPropertyContainer, "/input/trigger/value", &TriggerInputHandle,
			vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedOneSided
		);

		//  Buttons handles
		vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/application_menu/click", &m_start);
		vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/system/click", &m_back);

		vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/grip/click", &m_compA);
		vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/trigger/click", &m_compB);

		vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/trackpad/click", &m_compX);
		vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/trackpad/touch", &m_compY);

		// create our haptic component
		vr::VRDriverInput()->CreateHapticComponent(m_ulPropertyContainer, "/output/haptic", &m_compHaptic);

		return VRInitError_None;

	}

	/** This is called when The VR system is switching from this Hmd being the active display
	* to another Hmd being the active display. The driver should clean whatever memory
	* and thread use it can when it is deactivated */
	virtual void Deactivate()
	{
		CtrlIndex_t = vr::k_unTrackedDeviceIndexInvalid;
	}

	/** Handles a request from the system to put this device into standby mode. What that means is defined per-device. */
	virtual void EnterStandby()
	{
		DriverLog("Barebones: Entering Standby Mode\n");
	}

	/** Requests a component interface of the driver for device-specific functionality. The driver should return NULL
	* if the requested interface or version is not supported. */
	virtual void* GetComponent(const char* pchComponentNameAndVersion)
	{
		// override this to add a component to a driver
		return NULL;
	}

	virtual void PowerOff()
	{
	}

	/** A VR Watchdog has made this debug request of the driver. The set of valid requests is entirely
	* up to the driver and the Watchdog to figure out, as is the format of the response. Responses that
	* exceed the length of the supplied buffer should be truncated and null terminated */
	virtual void DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) override
	{
		if (unResponseBufferSize >= 1)
			pchResponseBuffer[0] = 0;
	}

	/*
	void Center() {
		double offset = 1.5;
		if (center == true) {
			MyCtrl.X = 0;
			MyCtrl.Y = offset;
			MyCtrl.Z = 0;
			MyCtrl.Yaw = 0;
			MyCtrl.Pitch = 0;
			MyCtrl.Roll = 0;
			center = false;
		}
	}
	*/

	// ------------------------------------
	// Tracking Methods
	// ------------------------------------
	virtual DriverPose_t GetPose()
	{
		DriverPose_t pose = { 0 };
		pose.qWorldFromDriverRotation = HmdQuaternion_Init(1, 0, 0, 0);
		pose.qDriverFromHeadRotation = HmdQuaternion_Init(1, 0, 0, 0);
		pose.qRotation = HmdQuaternion_Init(0, 0, 0, 0);

		if (Active) {
			//vr::TrackedDeviceIndex_t livCam = 2; //Should start as one of the controllers if LIV virtual tracker is not available
			/* //Not working yet
			for (vr::TrackedDeviceIndex_t idx = 0; idx < vr::k_unMaxTrackedDeviceCount; idx++)
			{
				PropertyContainerHandle_t handle = vr::VRProperties()->TrackedDeviceToPropertyContainer(idx);
				std::string cType = vr::VRProperties()->GetStringProperty(handle, Prop_ControllerType_String);
				//Have seen this as:
				//vive_controller
				//oculus_touch
				//*garbage*

				if (!cType.empty()) {
					DriverLog("%s", cType);
					if (strcmp(cType.data(), "liv_virtualcamera") == 0)
						//if (serial.find("liv_virtualcamera") != std::string::npos)
						livCam = idx;
				}
			}
			*/

			vr::TrackedDevicePose_t devicePoses[vr::k_unMaxTrackedDeviceCount];
			vr::VRServerDriverHost()->GetRawTrackedDevicePoses(0, devicePoses, vr::k_unMaxTrackedDeviceCount);
			vr::TrackedDevicePose_t tracker = devicePoses[FollowIndex];
			HmdQuaternion_t rot = GetRotation(tracker.mDeviceToAbsoluteTracking);

			pose.vecPosition[0] = tracker.mDeviceToAbsoluteTracking.m[0][3];
			pose.vecPosition[1] = tracker.mDeviceToAbsoluteTracking.m[1][3];
			pose.vecPosition[2] = tracker.mDeviceToAbsoluteTracking.m[2][3];

			pose.qRotation = rot;
		}

		cnt++;
		if (cnt == 101) {
			cnt = 0;
		}

		pose.poseIsValid = Active;
		pose.result = TrackingResult_Running_OK;
		pose.deviceIsConnected = Active;
		return pose;
	}

	void RunFrame()
	{
		//Update position here

		if (CtrlIndex_t != vr::k_unTrackedDeviceIndexInvalid)
		{
			vr::VRServerDriverHost()->TrackedDevicePoseUpdated(CtrlIndex_t, GetPose(), sizeof(DriverPose_t));
		}
	}
	std::string GetSerialNumber() const {

		return "CTRL1Serial";
	}

	void ProcessEvent(const vr::VREvent_t& vrEvent)
	{
		switch (vrEvent.eventType)
		{
		case vr::VREvent_Input_HapticVibration:
		{
			if (vrEvent.data.hapticVibration.componentHandle == m_compHaptic)
			{
				// This is where you would send a signal to your hardware to trigger actual haptic feedback
			}
		}
		break;
		}
	}

private:
	vr::TrackedDeviceIndex_t CtrlIndex_t;

	vr::PropertyContainerHandle_t m_ulPropertyContainer;

	std::string m_sSerialNumber;

	vr::VRInputComponentHandle_t TriggerInputHandle;

	vr::VRInputComponentHandle_t rightJoystickXInputHandle;
	vr::VRInputComponentHandle_t rightJoystickYInputHandle;
	vr::VRInputComponentHandle_t leftJoystickXInputHandle;
	vr::VRInputComponentHandle_t leftJoystickYInputHandle;

	vr::VRInputComponentHandle_t m_compHaptic;

	vr::VRInputComponentHandle_t m_compA;
	vr::VRInputComponentHandle_t m_compB;
	vr::VRInputComponentHandle_t m_compX;
	vr::VRInputComponentHandle_t m_compY;

	vr::VRInputComponentHandle_t m_start;
	vr::VRInputComponentHandle_t m_back;

	int check = 0;
	//bool center = true;

	int cnt = 0; //Count?
};

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
class BareboneProvider : public IServerTrackedDeviceProvider
{
public:
	virtual EVRInitError Init(vr::IVRDriverContext* pDriverContext);
	virtual const char* const* GetInterfaceVersions() { return vr::k_InterfaceVersions; }
	virtual bool ShouldBlockStandbyMode() { return false; }
	virtual void EnterStandby() {}
	virtual void LeaveStandby() {}
	virtual void RunFrame();
	virtual void Cleanup();
	virtual void PipeThread();

private:
	CBarebonesControllerDriver *m_pController = nullptr;
	//CBarebonesControllerDriver *m_pController2 = nullptr;
	HANDLE inPipe;
};

static BareboneProvider g_serverDriverNull;

EVRInitError BareboneProvider::Init(vr::IVRDriverContext* pDriverContext)
{
	VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);
	InitDriverLog(vr::VRDriverLog());

	DriverLog("barebones: Init\n");

	std::string inPipeName = "\\\\.\\pipe\\OpenVRvcPipeIn";

	inPipe = CreateNamedPipeA(inPipeName.c_str(),
		PIPE_ACCESS_DUPLEX,
		PIPE_TYPE_MESSAGE | PIPE_READMODE_MESSAGE | PIPE_WAIT,   // FILE_FLAG_FIRST_PIPE_INSTANCE is not needed but forces CreateNamedPipe(..) to fail if the pipe already exists...
		1,
		100 * 16,
		100 * 16,
		NMPWAIT_USE_DEFAULT_WAIT,
		NULL);

	std::thread pipeThread(&BareboneProvider::PipeThread, this);
	pipeThread.detach();

	m_pController = new CBarebonesControllerDriver();
	vr::VRServerDriverHost()->TrackedDeviceAdded(m_pController->GetSerialNumber().c_str(), vr::TrackedDeviceClass_Controller, m_pController);

	return VRInitError_None;
}

void BareboneProvider::PipeThread()
{
	char buffer[100];
	DWORD dwWritten;
	DWORD dwRead;

	for (;;)
	{
		ConnectNamedPipe(inPipe, NULL);

		if (ReadFile(inPipe, buffer, sizeof(buffer) - 1, &dwRead, NULL) != FALSE)
		{
			if (buffer[0] == 0)
				continue;

			if (buffer[0] == 127)
				m_pController->Active = false;
			else if (buffer[0] < 64) { //k_unMaxTrackedDeviceCount
				m_pController->FollowIndex = buffer[0];
				m_pController->Active = true;
			}
			else {
				//buffer[dwRead] = '\0'; //add terminating zero
				//MessageBoxA(NULL, buffer, "OpenVR barebone", MB_OK);
			}
		}
		DisconnectNamedPipe(inPipe);
	}
}

void BareboneProvider::Cleanup()
{
	CleanupDriverLog();
	delete m_pController;
	m_pController = NULL;
}

void BareboneProvider::RunFrame()
{
	if (m_pController)
	{
		m_pController->RunFrame();
	}

	vr::VREvent_t vrEvent;
	while (vr::VRServerDriverHost()->PollNextEvent(&vrEvent, sizeof(vrEvent)))
	{
		if (m_pController)
		{
			m_pController->ProcessEvent(vrEvent);
		}
	}
}

HMD_DLL_EXPORT void* HmdDriverFactory(const char* pInterfaceName, int* pReturnCode)
{
	if (0 == strcmp(IServerTrackedDeviceProvider_Version, pInterfaceName))
	{
		return &g_serverDriverNull;
	}
	if (pReturnCode)
		*pReturnCode = VRInitError_Init_InterfaceNotFound;

	return NULL;
}
