#include "FusionAPI.h"
#include "KinFuApp.hpp"

struct CRVI::FusionAPI::BoostData
{	
	boost::thread* mainThread;	
	boost::mutex runMutex;
};

CRVI::FusionAPI::FusionAPI(VirtualGrabber *virtualGrabber, bool displayPCLWindows, bool enableCurrentFrameCloudView)
	:mError_Callback(nullptr), mMeshReady_Callback(nullptr), mCurrent_DepthFrame_Callback(nullptr), mCurrent_ColorFrame_Callback(nullptr),
	mCurrent_PointCloud_Callback(nullptr), mRayTracer_3DView_Callback(nullptr),	mVirtualGrabber(virtualGrabber),
	mBoostData(new BoostData()), mKinFuApp(nullptr), mKinFuAppInitialized(false), mDisplayPCLWindows(displayPCLWindows),
	mEnableCurrentFrameCloudView(enableCurrentFrameCloudView)
{		
	mBoostData->mainThread = nullptr;

	mBoostData->runMutex.lock();	
	mIsThreadRunning = false;
	mBoostData->runMutex.unlock();
}

CRVI::FusionAPI::~FusionAPI()
{		
	if (mBoostData->mainThread)
		delete mBoostData->mainThread;	

	delete mBoostData;
}

void CRVI::FusionAPI::Start()
{	
	//starts the kinfutracker
	if(mKinFuApp && mKinFuAppInitialized)
	{
		mKinFuApp->kinfu_.SetIsTracking(true);	
	}
	else
	{
		if(mError_Callback != nullptr)
		{
			(*mError_Callback)(CRVI::FusionAPI::ErrorCode::NotInitialized);
		}	
	}	
}

void CRVI::FusionAPI::Close()
{	
	//exits the kinfuapp
	if(mKinFuApp && mKinFuAppInitialized)
	{		
		boost::mutex::scoped_lock lock(kinFuAppMutex);
		if (!mKinFuApp->exit_)
		{
			mKinFuApp->exit_ = true;
		}
	}
	else
	{
		if(mError_Callback != nullptr)
		{
			(*mError_Callback)(CRVI::FusionAPI::ErrorCode::NotInitialized);
		}	
	}		
}

void CRVI::FusionAPI::PauseApp()
{
	if(mKinFuApp && mKinFuAppInitialized)
	{
		boost::mutex::scoped_lock lock(kinFuAppMutex);
		{
			if(!mKinFuApp->pause_)
			{				
				mKinFuApp->pause_ = true;
			}
		}		
	}
	else
	{
		if(mError_Callback != nullptr)
		{
			(*mError_Callback)(CRVI::FusionAPI::ErrorCode::NotInitialized);
		}	
	}
}

void CRVI::FusionAPI::ResumeApp()
{
	if(mKinFuApp && mKinFuAppInitialized)
	{
		boost::mutex::scoped_lock lock(kinFuAppMutex);
		{
			if(mKinFuApp->pause_)
			{				
				mKinFuApp->pause_ = false;
			}
		}		
	}
	else
	{
		if(mError_Callback != nullptr)
		{
			(*mError_Callback)(CRVI::FusionAPI::ErrorCode::NotInitialized);
		}	
	}
}

void CRVI::FusionAPI::SetSystemMode(CRVI::Globals::SystemMode systemMode)
{
	CRVI::Globals::systemMode = systemMode;
}

void CRVI::FusionAPI::SetVolumeSize(float x, float y, float z)
{
	CRVI::Globals::volume_Size_X = x;
	CRVI::Globals::volume_Size_Y = y;
	CRVI::Globals::volume_Size_Z = z;
}

void CRVI::FusionAPI::SetDistanceToObject(float dist)
{
	CRVI::Globals::dist = dist;
	
}

void CRVI::FusionAPI::SetResolution(Resolution resolution)
{
	switch(resolution)
	{
	case Resolution::Low:
		CRVI::Globals::resolution_X = 128;
		CRVI::Globals::resolution_Y = 128;
		CRVI::Globals::resolution_Z = 128;
		break;
	case Resolution::Medium:
		CRVI::Globals::resolution_X = 256;
		CRVI::Globals::resolution_Y = 256;
		CRVI::Globals::resolution_Z = 256;
		break;
	case Resolution::High:
		CRVI::Globals::resolution_X = 512;
		CRVI::Globals::resolution_Y = 512;
		CRVI::Globals::resolution_Z = 512;
		break;
	}
}

void CRVI::FusionAPI::ClearSceneCloudViewClouds()
{
	boost::mutex::scoped_lock lock(kinFuAppMutex);
	mKinFuApp->clear_clouds_requested_ = true;
}

void CRVI::FusionAPI::SaveMeshToPLYFile()
{
	if(mKinFuApp && mKinFuAppInitialized)
	{		
		boost::mutex::scoped_lock lock(kinFuAppMutex);		
		if(mKinFuApp->mesh_generated_)
		{
			mKinFuApp->writeMesh (7);		
		}
		else
		{
			if(mError_Callback != nullptr)
			{
				(*mError_Callback)(CRVI::FusionAPI::ErrorCode::NoMeshToSave);
			}	
		}
	}
	else
	{
		if(mError_Callback != nullptr)
		{
			(*mError_Callback)(CRVI::FusionAPI::ErrorCode::NotInitialized);
		}	
	}	
}

CRVI::ViewPoint CRVI::FusionAPI::GetViewPoint()
{
	CRVI::ViewPoint viewPoint;
	
	if(mKinFuApp && mKinFuAppInitialized)
	{
		////////#########   CRVI IMPORTANT CRVI   #########////////
		//
		//-->uncomment this mutex to call GetViewPoint() from outside a callback (so that the tracker in the kinfuapp thread doesn't update its position while we read it)
		//
		//-->comment the mutex if you call GetViewPoint() from inside a callback (some callbacks are called from a scope where this mutex is locked (kinfuapp->execute)
		//		and it would create a deadlock. ALL of the callbacks can safely call GetViewPoint without locking the mutex because the callbacks are all called by the kinfuapp thread
		//		which is the same thread that updates the tracker position. The kinfuapp thread cannot update the tracker position at the same time it is executing a callback.
		//		
		//-->implement your own alternative function if you need to use both of the possibilities
		//
		//boost::mutex::scoped_lock lock(kinFuAppMutex);	
		////////#########   CRVI IMPORTANT CRVI  #########////////

		Eigen::Affine3f pose = mKinFuApp->kinfu_.getCameraPose();
		Eigen::Matrix3f rotate = pose.rotation();
		Eigen::Vector3f t = pose.translation();

		for(int i = 0; i < 9 ; i ++)
		{
			viewPoint.rotation[i] = *(rotate.data() + i);
		}
		
		for(int i = 0; i < 3 ; i ++)
		{
			viewPoint.translation[i] = *(t.data() + i);
		}
	}
	else
	{
		if(mError_Callback != nullptr)
		{
			(*mError_Callback)(CRVI::FusionAPI::ErrorCode::NotInitialized);
		}	
	}
	return viewPoint;
}

void CRVI::FusionAPI::Stop()
{
	//scans the mesh in memory, will trigger mesh_ready event
	if(mKinFuApp && mKinFuAppInitialized)
	{
		boost::mutex::scoped_lock lock(kinFuAppMutex);
		mKinFuApp->scan_mesh_ = true;			
	}
	else
	{
		if(mError_Callback != nullptr)
		{
			(*mError_Callback)(CRVI::FusionAPI::ErrorCode::NotInitialized);
		}	
	}
}

bool CRVI::FusionAPI::IsTracking()
{
	//returns the state of the kinfu tracker
	bool isTracking = false;
	if(mKinFuApp && mKinFuAppInitialized)
	{
		isTracking = mKinFuApp->kinfu_.IsTracking();
	}
	else
	{
		if(mError_Callback != nullptr)
		{
			(*mError_Callback)(CRVI::FusionAPI::ErrorCode::NotInitialized);
		}	
	}	
	return isTracking;
}

void CRVI::FusionAPI::ToggleBoundingBox()
{
	//toggle bounding box in the scene cloud viewer
	if(mKinFuApp && mKinFuAppInitialized)
	{
		boost::mutex::scoped_lock lock(kinFuAppMutex);
		if(!mKinFuApp->toggle_cube_requested_)
		{
			mKinFuApp->toggle_cube_requested_ = true;
		}
	}
	else
	{
		if(mError_Callback != nullptr)
		{
			(*mError_Callback)(CRVI::FusionAPI::ErrorCode::NotInitialized);
		}	
	}	
}

void CRVI::FusionAPI::Reset()
{
	//reset the kinfutracker and clears the tsdf cube data (the model being built in memory)
	if(mKinFuApp && mKinFuAppInitialized)
	{
		boost::mutex::scoped_lock lock(kinFuAppMutex);
		{
			if(!mKinFuApp->reset_requested_)
			{				
				mKinFuApp->reset_requested_ = true;
			}
		}		
	}
	else
	{
		if(mError_Callback != nullptr)
		{
			(*mError_Callback)(CRVI::FusionAPI::ErrorCode::NotInitialized);
		}	
	}
}

void CRVI::FusionAPI::RegisterErrorCallback(void(*error_callback)(CRVI::FusionAPI::ErrorCode errorCode))
{
	mError_Callback = error_callback;
}

void CRVI::FusionAPI::RegisterMeshReadyCallback(void(*mesh_ready_callback)(CRVI::Mesh* mesh))
{
	mMeshReady_Callback = mesh_ready_callback;
}

void CRVI::FusionAPI::RegisterCurrentDepthFrameCallback(void(*current_depth_frame_callback)(CRVI::DepthFrame *deptFrame))
{
	mCurrent_DepthFrame_Callback = current_depth_frame_callback;
}

void CRVI::FusionAPI::RegisterCurrentColorFrameCallback(void(*current_color_frame_callback)(CRVI::ColorFrame *colorFrame))
{
	mCurrent_ColorFrame_Callback = current_color_frame_callback;
}

void CRVI::FusionAPI::RegisterCurrentPointCloudCallback(void(*current_point_cloud_callback)(CRVI::PointCloud *pointCloud))
{
	mCurrent_PointCloud_Callback = current_point_cloud_callback;
}

void CRVI::FusionAPI::RegisterRayTracer3DViewCallback(void(*raytracer_3dview_callback)(CRVI::ColorFrame *raytracer_3dview))
{
	mRayTracer_3DView_Callback = raytracer_3dview_callback;
}

void CRVI::FusionAPI::InitKinFuApp()
{
	mBoostData->runMutex.lock();
	if(!mIsThreadRunning)
	{
		mBoostData->mainThread = new boost::thread(&FusionAPI::Main, this);	
		mIsThreadRunning = true;
	}
	mBoostData->runMutex.unlock();
}

bool CRVI::FusionAPI::IsReadyToScan()
{
	return mKinFuApp && mKinFuAppInitialized;
}

const char * CRVI::FusionAPI::GetErrorMessage(CRVI::FusionAPI::ErrorCode errorCode)
{
	char * errorMsg = "UnknownErrorCode";

	switch(errorCode)
	{
		case CRVI::FusionAPI::ErrorCode::NotInitialized : errorMsg = "NotInitialized"; break;
		case CRVI::FusionAPI::ErrorCode::ErrorInitializing : errorMsg = "ErrorInitializing"; break;
		case CRVI::FusionAPI::ErrorCode::AcquisitionError : errorMsg = "AcquisitionError"; break;
		case CRVI::FusionAPI::ErrorCode::NoMeshToSave : errorMsg = "NoMeshToSave"; break;
		case CRVI::FusionAPI::ErrorCode::MeshDataError : errorMsg = "MeshDataError"; break;
		case CRVI::FusionAPI::ErrorCode::PCLException : errorMsg = "PCLException"; break;
		case CRVI::FusionAPI::ErrorCode::BadAllocation : errorMsg = "BadAllocation"; break;
		case CRVI::FusionAPI::ErrorCode::GeneralException : errorMsg = "GeneralException"; break;		
	}

	return errorMsg;
}

void CRVI::FusionAPI::Main()
{		
	try
	{	
		int device = 0;	
		pcl::gpu::setDevice (device);
		pcl::gpu::printShortCudaDeviceInfo (device);

		//  if (checkIfPreFermiGPU(device))
		//    return cout << endl << "Kinfu is supported only for Fermi and Kepler arhitectures. It is not even compiled for pre-Fermi by default. Exiting..." << endl, 1;
		
		int icp = 1;
		int viz = mDisplayPCLWindows ? 1 : 0;
			
		// std::string camera_pose_file;
		boost::shared_ptr<CameraPoseProcessor> pose_processor;
		/*if (pc::parse_argument (argc, argv, "-save_pose", camera_pose_file) && camera_pose_file.size () > 0)
		{
		pose_processor.reset (new CameraPoseWriter (camera_pose_file));
		}*/
		
		mKinFuApp = new KinFuApp (mVirtualGrabber, icp, viz, mEnableCurrentFrameCloudView, pose_processor);

		//pass the callbacks to the kinfuapp
		mKinFuApp->mesh_ready_callback_ = mMeshReady_Callback;
		mKinFuApp->error_callback_ = mError_Callback;
		mKinFuApp->current_depth_frame_callback_ = mCurrent_DepthFrame_Callback;
		mKinFuApp->current_color_frame_callback_ = mCurrent_ColorFrame_Callback;
		mKinFuApp->current_point_cloud_callback_ = mCurrent_PointCloud_Callback;
		mKinFuApp->raytracer_3dview_callback_ = mRayTracer_3DView_Callback;

		mKinFuAppInitialized = true;
	}
	catch (const std::exception& /*e*/)
	{
		if(mError_Callback != nullptr)
		{
			(*mError_Callback)(CRVI::FusionAPI::ErrorCode::ErrorInitializing);
		}			
	}
	
	if(mKinFuApp && mKinFuAppInitialized)
	{
		// starts the KinFuApp
		try 
		{ 
			if(mKinFuApp)		
				mKinFuApp->startMainLoop();	
		}
		catch (const pcl::PCLException& /*e*/)
		{ 
			if(mError_Callback != nullptr)
			{
				(*mError_Callback)(CRVI::FusionAPI::ErrorCode::PCLException);
			}		
		}
		catch (const std::bad_alloc& /*e*/) 
		{ 
			if(mError_Callback != nullptr)
			{
				(*mError_Callback)(CRVI::FusionAPI::ErrorCode::BadAllocation);
			}		
		}
		catch (const std::exception& /*e*/)
		{ 
			if(mError_Callback != nullptr)
			{
				(*mError_Callback)(CRVI::FusionAPI::ErrorCode::GeneralException);
			}		
		}

	#ifdef HAVE_OPENCV
		for (size_t t = 0; t < app.image_view_.views_.size (); ++t)
		{
			if (t == 0)
			{
				cout << "Saving depth map of first view." << endl;
				cv::imwrite ("./depthmap_1stview.png", app.image_view_.views_[0]);
				cout << "Saving sequence of (" << app.image_view_.views_.size () << ") views." << endl;
			}
			char buf[4096];
			sprintf (buf, "./%06d.png", (int)t);
			cv::imwrite (buf, app.image_view_.views_[t]);
			printf ("writing: %s\n", buf);
		}
	#endif
	}

	if(mKinFuApp)
	{
		delete mKinFuApp;
		mKinFuApp = nullptr;
	}

	mBoostData->runMutex.lock();	
	mIsThreadRunning = false;
	mBoostData->runMutex.unlock();
}

