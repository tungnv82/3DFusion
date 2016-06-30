#ifndef __FUSION_
#define __FUSION_

#include "VirtualGrabber.h"
#include "crvi/gpu/kinfu/globals.h"
#include "FusionCommon.h"

struct KinFuApp;

namespace CRVI
{	
	class FUSION_API FusionAPI
	{		
	public:
		enum Resolution : signed int
		{
			Low = 0,
			Medium,
			High
		};

		enum ErrorCode : signed int
		{			
			NotInitialized = 0,
			ErrorInitializing,
			AcquisitionError,			
			NoMeshToSave,
			MeshDataError,
			PCLException,
			BadAllocation,
			GeneralException
		};


		FusionAPI(VirtualGrabber *virtualGrabber, bool displayPCLWindows, bool enableCurrentFrameCloudView); 
		~FusionAPI();

		const char * GetErrorMessage(CRVI::FusionAPI::ErrorCode errorCode);
		void InitKinFuApp(); //starts the kinfuapp thread and creates the kinfuApp with the parameters and registered callbacks previously given to the fusionAPI
		bool IsReadyToScan(); //indicates that the kinfuApp is fully initialized
		void Start(); //starts the kinfutracker
		void Stop(); //generates mesh from data in memory, letting the kinfutracker running, will trigger mesh ready event when done
		void Reset(); //clears the tsdf mesh data in memory and resets the kinfutracker	
		void Close(); //exit the kinFuApp mainloop and exits the kinfuapp thread
		void SaveMeshToPLYFile(); //save mesh data to mesh.ply file (must be called after mesh ready event)
		bool IsTracking(); //indicates that the kinfuTracker is tracking and that the frames are added to the fusion algorithm
		void ToggleBoundingBox(); //toggles wether or not bounding box is displayed in the scene cloud viewer
		void ClearSceneCloudViewClouds(); //clears the scene cloud view memory, eliminates unnecessary rendering to get more fps for a better fusion
		void PauseApp(); //pause the kinfuApp MainLoop
		void ResumeApp(); //resume the kinfuApp MainLoop

		CRVI::ViewPoint GetViewPoint(); // gets the current viewpoint (can only be safely called from inside a callback for now, SEE COMMENT INSIDE THE FUNCTION)				
				
		void RegisterErrorCallback(void(*error_callback)(CRVI::FusionAPI::ErrorCode errorCode));
		void RegisterMeshReadyCallback(void(*mesh_ready_callback)(CRVI::Mesh* mesh));
		void RegisterCurrentDepthFrameCallback(void(*current_depth_frame_callback)(CRVI::DepthFrame *deptFrame));
		void RegisterCurrentColorFrameCallback(void(*current_color_frame_callback)(CRVI::ColorFrame *colorFrame));
		void RegisterCurrentPointCloudCallback(void(*current_point_cloud_callback)(CRVI::PointCloud *pointCloud));
		void RegisterRayTracer3DViewCallback(void(*raytracer_3dview_callback)(CRVI::ColorFrame *raytracer_3dview));

		//freerunning mode or turntable mode
		void SetSystemMode(CRVI::Globals::SystemMode systemMode);

		//volume size of the scanning bounding box
		void SetVolumeSize(float x, float y, float z);

      // set distance from sensor to object at initial
      void SetDistanceToObject(float dist);

		//cuda cube resolution
		void SetResolution(Resolution resolution); 
		
	private:
		void Main();		
		
		bool mKinFuAppInitialized;		
		bool mDisplayPCLWindows;
		bool mEnableCurrentFrameCloudView;
		bool mIsThreadRunning;

		VirtualGrabber *mVirtualGrabber;			
		KinFuApp *mKinFuApp;		

		void(*mMeshReady_Callback)(CRVI::Mesh* mesh); 
		void(*mError_Callback)(CRVI::FusionAPI::ErrorCode errorCode);	
		void(*mCurrent_DepthFrame_Callback)(CRVI::DepthFrame *deptFrame);
		void(*mCurrent_ColorFrame_Callback)(CRVI::ColorFrame *colorFrame);
		void(*mCurrent_PointCloud_Callback)(CRVI::PointCloud *pointCloud);
		void(*mRayTracer_3DView_Callback)(CRVI::ColorFrame *raytracer_3dview);

		struct BoostData;
		BoostData* mBoostData;	
	};	
}

#endif
