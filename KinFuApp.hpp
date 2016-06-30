#ifndef __KINFUAPP_
#define __KINFUAPP_

/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
*  Author: Anatoly Baskeheev, Itseez Ltd, (myname.mysurname@mycompany.com)
*/

#define _CRT_SECURE_NO_DEPRECATE

#include <iostream>
#include <vector>

#include <crvi/gpu/kinfu/kinfu.h>
#include <crvi/gpu/kinfu/raycaster.h>
#include <crvi/gpu/kinfu/marching_cubes.h>

#include <pcl/gpu/containers/initialization.h>
#include <pcl/common/time.h>
#include <pcl/common/angles.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/exceptions.h>

#include <string>
#include <limits>

#include <OpenNI.h>

#include "tsdf_volume.h"
#include "tsdf_volume.hpp"
#include "camera_pose.h"

#include <opencv2\core.hpp>
#include <opencv2\highgui.hpp>

#ifdef HAVE_OPENCV  
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include "video_recorder.h"
#endif

typedef pcl::ScopeTime ScopeTimeT;

#include "../CRVI_Gpu_Kinfu/crvi_gpu_kinfu/src/internal.h"

using namespace std;
using namespace pcl;
using namespace pcl::gpu;
using namespace Eigen;

boost::mutex kinFuAppMutex;	

namespace pcl
{
	namespace gpu
	{
		void paint3DView (const KinfuTracker::View& rgb24, KinfuTracker::View& view, float colors_weight = 0.5f);
		void mergePointNormal (const DeviceArray<PointXYZ>& cloud, const DeviceArray<Normal>& normals, DeviceArray<PointNormal>& output);
	}

	namespace visualization
	{
		//////////////////////////////////////////////////////////////////////////////////////
		/** \brief RGB handler class for colors. Uses the data present in the "rgb" or "rgba"
		* fields from an additional cloud as the color at each point.
		* \author Anatoly Baksheev
		* \ingroup visualization
		*/
		template <typename PointT>
		class PointCloudColorHandlerRGBCloud : public PointCloudColorHandler<PointT>
		{
			using PointCloudColorHandler<PointT>::capable_;
			using PointCloudColorHandler<PointT>::cloud_;

			typedef typename PointCloudColorHandler<PointT>::PointCloud::ConstPtr PointCloudConstPtr;
			typedef typename pcl::PointCloud<RGB>::ConstPtr RgbCloudConstPtr;

		public:
			typedef boost::shared_ptr<PointCloudColorHandlerRGBCloud<PointT> > Ptr;
			typedef boost::shared_ptr<const PointCloudColorHandlerRGBCloud<PointT> > ConstPtr;

			/** \brief Constructor. */
			PointCloudColorHandlerRGBCloud (const PointCloudConstPtr& cloud, const RgbCloudConstPtr& colors)
				: rgb_ (colors)
			{
				cloud_  = cloud;
				capable_ = true;
			}

			/** \brief Obtain the actual color for the input dataset as vtk scalars.
			* \param[out] scalars the output scalars containing the color for the dataset
			* \return true if the operation was successful (the handler is capable and 
			* the input cloud was given as a valid pointer), false otherwise
			*/
			virtual bool
				getColor (vtkSmartPointer<vtkDataArray> &scalars) const
			{
				if (!capable_ || !cloud_)
					return (false);

				if (!scalars)
					scalars = vtkSmartPointer<vtkUnsignedCharArray>::New ();
				scalars->SetNumberOfComponents (3);

				vtkIdType nr_points = vtkIdType (cloud_->points.size ());
				reinterpret_cast<vtkUnsignedCharArray*>(&(*scalars))->SetNumberOfTuples (nr_points);
				unsigned char* colors = reinterpret_cast<vtkUnsignedCharArray*>(&(*scalars))->GetPointer (0);

				// Color every point
				if (nr_points != int (rgb_->points.size ()))
					std::fill (colors, colors + nr_points * 3, static_cast<unsigned char> (0xFF));
				else
					for (vtkIdType cp = 0; cp < nr_points; ++cp)
					{
						int idx = cp * 3;
						colors[idx + 0] = rgb_->points[cp].r;
						colors[idx + 1] = rgb_->points[cp].g;
						colors[idx + 2] = rgb_->points[cp].b;
					}
					return (true);
			}

		private:
			virtual std::string 
				getFieldName () const { return ("additional rgb"); }
			virtual std::string 
				getName () const { return ("PointCloudColorHandlerRGBCloud"); }

			RgbCloudConstPtr rgb_;
		};
	}
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct SampledScopeTime : public StopWatch
{          
	enum { EACH = 33 };
	SampledScopeTime(int& time_ms) : time_ms_(time_ms) {}
	~SampledScopeTime()
	{
		static int i_ = 0;
		static boost::posix_time::ptime starttime_ = boost::posix_time::microsec_clock::local_time();
		time_ms_ += getTime ();
		if (i_ % EACH == 0 && i_)
		{
			boost::posix_time::ptime endtime_ = boost::posix_time::microsec_clock::local_time();
			cout << "Average frame time = " << time_ms_ / EACH << "ms ( " << 1000.f * EACH / time_ms_ << "fps )"
				<< "( real: " << 1000.f * EACH / (endtime_-starttime_).total_milliseconds() << "fps )"  << endl;
			time_ms_ = 0;
			starttime_ = endtime_;
		}
		++i_;
	}
private:    
	int& time_ms_;
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void
	setViewerPose (visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose)
{
	Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f (0, 0, 0);
	Eigen::Vector3f look_at_vector = viewer_pose.rotation () * Eigen::Vector3f (0, 0, 1) + pos_vector;
	Eigen::Vector3f up_vector = viewer_pose.rotation () * Eigen::Vector3f (0, -1, 0);
	viewer.setCameraPosition (pos_vector[0], pos_vector[1], pos_vector[2],
		look_at_vector[0], look_at_vector[1], look_at_vector[2],
		up_vector[0], up_vector[1], up_vector[2]);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Eigen::Affine3f 
	getViewerPose (visualization::PCLVisualizer& viewer)
{
	Eigen::Affine3f pose = viewer.getViewerPose();
	Eigen::Matrix3f rotation = pose.linear();

	Matrix3f axis_reorder;  
	axis_reorder << 0,  0,  1,
		-1,  0,  0,
		0, -1,  0;

	rotation = rotation * axis_reorder;
	pose.linear() = rotation;
	return pose;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template<typename CloudT> void
	writeCloudFile (int format, const CloudT& cloud);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
	writePolygonMeshFile (int format, const pcl::PolygonMesh& mesh);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template<typename MergedT, typename PointT>
typename PointCloud<MergedT>::Ptr merge(const PointCloud<PointT>& points, const PointCloud<RGB>& colors)
{    
	typename PointCloud<MergedT>::Ptr merged_ptr(new PointCloud<MergedT>());

	pcl::copyPointCloud (points, *merged_ptr);      
	for (size_t i = 0; i < colors.size (); ++i)
		merged_ptr->points[i].rgba = colors.points[i].rgba;

	return merged_ptr;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

boost::shared_ptr<pcl::PolygonMesh> convertToMesh(const DeviceArray<PointXYZ>& triangles)
{ 
	if (triangles.empty())
		return boost::shared_ptr<pcl::PolygonMesh>();

	pcl::PointCloud<pcl::PointXYZ> cloud;
	cloud.width  = (int)triangles.size();
	cloud.height = 1;
	triangles.download(cloud.points);

	boost::shared_ptr<pcl::PolygonMesh> mesh_ptr( new pcl::PolygonMesh() ); 
	pcl::toPCLPointCloud2(cloud, mesh_ptr->cloud);

	mesh_ptr->polygons.resize (triangles.size() / 3);
	for (size_t i = 0; i < mesh_ptr->polygons.size (); ++i)
	{
		pcl::Vertices v;
		v.vertices.push_back(i*3+0);
		v.vertices.push_back(i*3+2);
		v.vertices.push_back(i*3+1);              
		mesh_ptr->polygons[i] = v;
	}    
	return mesh_ptr;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct CurrentFrameCloudView
{
	CurrentFrameCloudView(int viz) : cloud_device_ (480, 640)
	{
		cloud_ptr_ = PointCloud<PointXYZ>::Ptr (new PointCloud<PointXYZ>);

		if(viz)
		{
			cloud_viewer_ = pcl::visualization::PCLVisualizer::Ptr( new pcl::visualization::PCLVisualizer("Frame Cloud Viewer") );

			cloud_viewer_->setBackgroundColor (0, 0, 0.15);
			cloud_viewer_->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 1);
			cloud_viewer_->addCoordinateSystem (1.0, "global");
			cloud_viewer_->initCameraParameters ();
			cloud_viewer_->setPosition (500, 500);
			cloud_viewer_->setSize (640, 480);
			cloud_viewer_->setCameraClipDistances (0.01, 10.01);
		}
	}

	void
		generateCurrentCloud (const KinfuTracker& kinfu)
	{
		kinfu.getLastFrameCloud (cloud_device_);

		int c;
		cloud_device_.download (cloud_ptr_->points, c);
		cloud_ptr_->width = cloud_device_.cols ();
		cloud_ptr_->height = cloud_device_.rows ();
		cloud_ptr_->is_dense = false;
	}

	void
		showCurrentCloud ()
	{
		cloud_viewer_->removeAllPointClouds ();
		cloud_viewer_->addPointCloud<PointXYZ>(cloud_ptr_);
		cloud_viewer_->spinOnce ();
	}

	void
		setViewerPose (const Eigen::Affine3f& viewer_pose) {
			::setViewerPose (*cloud_viewer_, viewer_pose);
	}

	PointCloud<PointXYZ>::Ptr cloud_ptr_;
	DeviceArray2D<PointXYZ> cloud_device_;	
	visualization::PCLVisualizer::Ptr cloud_viewer_;
			
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct ImageView
{
	ImageView(int viz) : viz_(viz), paint_image_ (false), accumulate_views_ (false)
	{
		if(viz_)
		{
			viewerScene_ = pcl::visualization::ImageViewer::Ptr(new pcl::visualization::ImageViewer);
			viewerScene_->setWindowTitle ("View3D from ray tracing");
			viewerScene_->setPosition (0, 0);
		
			viewerDepth_ = pcl::visualization::ImageViewer::Ptr(new pcl::visualization::ImageViewer);			
			viewerDepth_->setWindowTitle ("Kinect Depth stream");
			viewerDepth_->setPosition (640, 0);

			//viewerColor_.setWindowTitle ("Kinect RGB stream");
		}
	}

void
	generateScene (KinfuTracker& kinfu, const PtrStepSz<const KinfuTracker::PixelRGB>& rgb24, bool registration, Eigen::Affine3f* pose_ptr = 0)
	{
		if (pose_ptr)
		{
			raycaster_ptr_->run(kinfu.volume(), *pose_ptr);
			raycaster_ptr_->generateSceneView(view_device_);
		}
		else
			kinfu.getImage (view_device_);

		if (paint_image_ && registration && !pose_ptr)
		{
			colors_device_.upload (rgb24.data, rgb24.step, rgb24.rows, rgb24.cols);
			paint3DView (colors_device_, view_device_);
		}

		int cols;
		view_device_.download (view_host_, cols);		
	}

void
		showScene ()
	{	
		if (viz_)
		{			
			viewerScene_->showRGBImage (reinterpret_cast<unsigned char*> (&view_host_[0]), view_device_.cols (), view_device_.rows ()); 

			//viewerColor_.showRGBImage ((unsigned char*)&rgb24.data, rgb24.cols, rgb24.rows);
		}

#ifdef HAVE_OPENCV
		if (accumulate_views_)
		{
			views_.push_back (cv::Mat ());
			cv::cvtColor (cv::Mat (480, 640, CV_8UC3, (void*)&view_host_[0]), views_.back (), CV_RGB2GRAY);
			//cv::copy(cv::Mat(480, 640, CV_8UC3, (void*)&view_host_[0]), views_.back());
		}
#endif
	}

	void
		showDepth (const PtrStepSz<const unsigned short>& depth) 
	{ 
		if (viz_)
			viewerDepth_->showShortImage (depth.data, depth.cols, depth.rows, 0, 5000, true); 
	}

	void
		showGeneratedDepth (KinfuTracker& kinfu, const Eigen::Affine3f& pose)
	{            
		raycaster_ptr_->run(kinfu.volume(), pose);
		raycaster_ptr_->generateDepthImage(generated_depth_);    

		int c;
		vector<unsigned short> data;
		generated_depth_.download(data, c);

		if (viz_)
			viewerDepth_->showShortImage (&data[0], generated_depth_.cols(), generated_depth_.rows(), 0, 5000, true);
	}

	void
		toggleImagePaint()
	{
		paint_image_ = !paint_image_;
		cout << "Paint image: " << (paint_image_ ? "On   (requires registration mode)" : "Off") << endl;
	}
	
	int viz_;

	bool paint_image_;
	bool accumulate_views_;

	visualization::ImageViewer::Ptr viewerScene_;
	visualization::ImageViewer::Ptr viewerDepth_;
	//visualization::ImageViewer viewerColor_;

	KinfuTracker::View view_device_;
	KinfuTracker::View colors_device_;
	vector<KinfuTracker::PixelRGB> view_host_;

	RayCaster::Ptr raycaster_ptr_;

	KinfuTracker::DepthMap generated_depth_;

#ifdef HAVE_OPENCV
	vector<cv::Mat> views_;
#endif
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct SceneCloudView
{
	enum { GPU_Connected6 = 0, CPU_Connected6 = 1, CPU_Connected26 = 2 };

	SceneCloudView(int viz) : viz_(viz), extraction_mode_ (GPU_Connected6), compute_normals_ (false), valid_combined_ (false), cube_added_(false)
	{
		cloud_ptr_ = PointCloud<PointXYZ>::Ptr (new PointCloud<PointXYZ>);
		normals_ptr_ = PointCloud<Normal>::Ptr (new PointCloud<Normal>);
		combined_ptr_ = PointCloud<PointNormal>::Ptr (new PointCloud<PointNormal>);
		point_colors_ptr_ = PointCloud<RGB>::Ptr (new PointCloud<RGB>);

		if (viz_)
		{
			cloud_viewer_ = pcl::visualization::PCLVisualizer::Ptr( new pcl::visualization::PCLVisualizer("Scene Cloud Viewer") );

			cloud_viewer_->setBackgroundColor (0.5, 0.5, 0);
			cloud_viewer_->addCoordinateSystem (1.0, "global");
			cloud_viewer_->initCameraParameters ();
			cloud_viewer_->setPosition (0, 500);
			cloud_viewer_->setSize (640, 480);
			cloud_viewer_->setCameraClipDistances (0.01, 10.01);

			cloud_viewer_->addText ("H: print help", 2, 15, 20, 34, 135, 246);
		}
	}

	void
		show (KinfuTracker& kinfu, bool integrate_colors)
	{
		viewer_pose_ = kinfu.getCameraPose();

		ScopeTimeT time ("PointCloud Extraction");
		cout << "\nGetting cloud... " << flush;

		valid_combined_ = false;

		if (extraction_mode_ != GPU_Connected6)     // So use CPU
		{
			kinfu.volume().fetchCloudHost (*cloud_ptr_, extraction_mode_ == CPU_Connected26);
		}
		else
		{
			DeviceArray<PointXYZ> extracted = kinfu.volume().fetchCloud (cloud_buffer_device_);             

			if (compute_normals_)
			{
				kinfu.volume().fetchNormals (extracted, normals_device_);
				pcl::gpu::mergePointNormal (extracted, normals_device_, combined_device_);
				combined_device_.download (combined_ptr_->points);
				combined_ptr_->width = (int)combined_ptr_->points.size ();
				combined_ptr_->height = 1;

				valid_combined_ = true;
			}
			else
			{
				extracted.download (cloud_ptr_->points);
				cloud_ptr_->width = (int)cloud_ptr_->points.size ();
				cloud_ptr_->height = 1;
			}

			if (integrate_colors)
			{
				kinfu.colorVolume().fetchColors(extracted, point_colors_device_);
				point_colors_device_.download(point_colors_ptr_->points);
				point_colors_ptr_->width = (int)point_colors_ptr_->points.size ();
				point_colors_ptr_->height = 1;
			}
			else
				point_colors_ptr_->points.clear();
		}
		size_t points_size = valid_combined_ ? combined_ptr_->points.size () : cloud_ptr_->points.size ();
		cout << "Done.  Cloud size: " << points_size / 1000 << "K" << endl;

		if (viz_)
		{
			cloud_viewer_->removeAllPointClouds ();
			if (valid_combined_)
			{
				visualization::PointCloudColorHandlerRGBCloud<PointNormal> rgb(combined_ptr_, point_colors_ptr_);
				cloud_viewer_->addPointCloud<PointNormal> (combined_ptr_, rgb, "Cloud");
				cloud_viewer_->addPointCloudNormals<PointNormal>(combined_ptr_, 50);
			}
			else
			{
				visualization::PointCloudColorHandlerRGBCloud<PointXYZ> rgb(cloud_ptr_, point_colors_ptr_);
				cloud_viewer_->addPointCloud<PointXYZ> (cloud_ptr_, rgb);
			}
		}
	}

	void
		toggleCube(const Eigen::Vector3f& size)
	{
		if (!viz_)
			return;

		if (cube_added_)
			cloud_viewer_->removeShape("cube");
		else
			cloud_viewer_->addCube(size*0.5, Eigen::Quaternionf::Identity(), size(0), size(1), size(2));

		cube_added_ = !cube_added_;
	}

	void
		toggleExtractionMode ()
	{
		extraction_mode_ = (extraction_mode_ + 1) % 3;

		switch (extraction_mode_)
		{
		case 0: cout << "Cloud exctraction mode: GPU, Connected-6" << endl; break;
		case 1: cout << "Cloud exctraction mode: CPU, Connected-6    (requires a lot of memory)" << endl; break;
		case 2: cout << "Cloud exctraction mode: CPU, Connected-26   (requires a lot of memory)" << endl; break;
		}
		;
	}

	void
		toggleNormals ()
	{
		compute_normals_ = !compute_normals_;
		cout << "Compute normals: " << (compute_normals_ ? "On" : "Off") << endl;
	}

	void
		clearClouds (bool print_message = false)
	{
		if (!viz_)
			return;

		cloud_viewer_->removeAllPointClouds ();
		cloud_ptr_->points.clear ();
		normals_ptr_->points.clear ();    
		if (print_message)
			cout << "Clouds/Meshes were cleared" << endl;
	}
	
	void
		scanMesh(KinfuTracker& kinfu, bool /*integrate_colors*/)
	{
		ScopeTimeT time ("Mesh Extraction");
		cout << "\nGetting mesh... " << flush;

		if (!marching_cubes_)
			marching_cubes_ = MarchingCubes::Ptr( new MarchingCubes() );

		DeviceArray<PointXYZ> triangles_device = marching_cubes_->run(kinfu.volume(), triangles_buffer_device_);    
		mesh_ptr_ = convertToMesh(triangles_device);

		cout << "Done.  Triangles number: " << triangles_device.size() / MarchingCubes::POINTS_PER_TRIANGLE / 1000 << "K" << endl;
	}

	void
		showMesh()
	{	
		cloud_viewer_->removeAllPointClouds ();
		if (mesh_ptr_)
		{
			cloud_viewer_->addPolygonMesh(*mesh_ptr_);
		}
	}

	int viz_;
	int extraction_mode_;
	bool compute_normals_;
	bool valid_combined_;
	bool cube_added_;

	Eigen::Affine3f viewer_pose_;

	visualization::PCLVisualizer::Ptr cloud_viewer_;

	PointCloud<PointXYZ>::Ptr cloud_ptr_;
	PointCloud<Normal>::Ptr normals_ptr_;

	DeviceArray<PointXYZ> cloud_buffer_device_;
	DeviceArray<Normal> normals_device_;

	PointCloud<PointNormal>::Ptr combined_ptr_;
	DeviceArray<PointNormal> combined_device_;  

	DeviceArray<RGB> point_colors_device_; 
	PointCloud<RGB>::Ptr point_colors_ptr_;

	MarchingCubes::Ptr marching_cubes_;
	DeviceArray<PointXYZ> triangles_buffer_device_;

	boost::shared_ptr<pcl::PolygonMesh> mesh_ptr_;
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct KinFuApp
{
	enum { PCD_BIN = 1, PCD_ASCII = 2, PLY = 3, MESH_PLY = 7, MESH_VTK = 8 };

	KinFuApp(CRVI::VirtualGrabber *virtualGrabber, int icp, int viz, bool enableCurrentFrameCloudView, boost::shared_ptr<CameraPoseProcessor> pose_processor=boost::shared_ptr<CameraPoseProcessor> ()) : virtualGrabber_(virtualGrabber), exit_ (false), scan_ (false), scan_mesh_(false), scan_volume_ (false), independent_camera_ (false),
		registration_ (false), integrate_colors_ (false), focal_length_(-1.f), time_ms_(0), icp_(icp), viz_(viz), pose_processor_ (pose_processor),
		mesh_ready_callback_(nullptr), error_callback_(nullptr), toggle_cube_requested_(false), clear_clouds_requested_(false), reset_requested_(false),		
		scene_cloud_view_(viz), image_view_(viz), current_depth_frame_callback_(nullptr), current_color_frame_callback_(nullptr), current_point_cloud_callback_(nullptr), 
		raytracer_3dview_callback_(nullptr), rayTracer3DView_(nullptr), currentPointCloud_(nullptr), mesh_generated_(false), pause_(false)
	{ 
		////     //Eigen::Vector3f volume_size = Vector3f::Constant (vsz/*meters*/);//origin    

		//Eigen::Vector3f volume_size = Vector3f(1.0f,2.0f,1.0f);         //bounding box define bounding boxxxx
		//if(CRVI::Globals::systemMode == CRVI::Globals::SystemMode::FreeRunning)
			//const Vector3f volume_size = Vector3f(1.0f,2.0f,1.0f);            //bounding box.....bounding box....
			//volume_size = Vector3f::Constant ( CRVI::Globals::volume_Size);  //volume change VTN
		Eigen::Vector3f volume_size = Vector3f(CRVI::Globals::volume_Size_X,CRVI::Globals::volume_Size_Y,CRVI::Globals::volume_Size_Z);
		kinfu_.volume().setSize (volume_size);

		Eigen::Matrix3f R = Eigen::Matrix3f::Identity ();   // * AngleAxisf( pcl::deg2rad(-30.f), Vector3f::UnitX());
		//Eigen::Vector3f t = volume_size * 0.5f - Vector3f (0, 0, volume_size (2) / 2 * 1.2f);    // origin   
		//Eigen::Vector3f t = Vector3f (volume_size(0)*0.5f, volume_size(1)*0.5f, volume_size(2)*0.5f-1.0f);  //1.2f for vol3
		Eigen::Vector3f t = Vector3f (volume_size(0)*0.5f, volume_size(1)*0.5f, volume_size(2)*0.5f-CRVI::Globals::dist);  
		
      
      //if(Globals::gSystemMode == 1)
		//     t = volume_size * 0.5f - Vector3f (0, 0, volume_size (2) / 2 * 1.2f);  //VTN... orign

		//if(CRVI::Globals::systemMode == CRVI::Globals::SystemMode::FreeRunning)
		//	//t = Vector3f (volume_size(0)*0.5f, volume_size(1)*0.5f, volume_size(2)*0.5f-1.0f);  
		//	//t = volume_size * 0.5f - Vector3f (0, 0, volume_size (2) / 2 * 1.2f);  //VTN... orign
		//{
		//	float test=sqrt(pow(volume_size(1)*0.5f,2)+pow(volume_size(2)*0.5f,2));
		//	t = Vector3f (volume_size(0)*0.5f, volume_size(1)*0.5f,volume_size(2)*0.5f-test);  //VTN... test 2016
		//}
		//Eigen::Vector3f t = Vector3f (volume_size (1)*0.5f,volume_size (2)*0.5f, -0.5f);  // VTN MODIFIED
		//Eigen::Vector3f t = Vector3f (0,0,0);  // VTN MODIFIED

		Eigen::Affine3f pose = Eigen::Translation3f (t) * Eigen::AngleAxisf (R);

		kinfu_.setInitalCameraPose (pose);
		//kinfu_.volume().setTsdfTruncDist (0.03f/*meters*/);       //origin 0.030f  
		kinfu_.volume().setTsdfTruncDist (volume_size(0) / 100.0f);    
		kinfu_.setIcpCorespFilteringParams (0.1f/*meters*/, sin ( pcl::deg2rad(20.f) ));   //VTN modifie, 0.1f origin
		//kinfu_.setDepthTruncationForICP(2.0f/*meters*/);  /// we can change this for our system:: original 5.f in commnent VTN
		kinfu_.setCameraMovementThreshold(0.001f);

		if (!icp)
			kinfu_.disableIcp();

		//Init KinfuApp            
		tsdf_cloud_ptr_ = pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI>);
		image_view_.raycaster_ptr_ = RayCaster::Ptr( new RayCaster(kinfu_.rows (), kinfu_.cols ()) );
			
		if(enableCurrentFrameCloudView)
		{
			initCurrentFrameView();
		}

		//initialize raytracer3dview buffer
		rayTracer3DView_ = new CRVI::ColorFrame(kinfu_.cols (), kinfu_.rows (), 3);
	}

	~KinFuApp()
	{	
		//cleanup
		if(rayTracer3DView_)
			delete rayTracer3DView_;

		if(currentPointCloud_)
			delete currentPointCloud_;
	}

	void
		initCurrentFrameView ()
	{
		current_frame_cloud_view_ = boost::shared_ptr<CurrentFrameCloudView>(new CurrentFrameCloudView (viz_));		
		if(viz_)
		{
			current_frame_cloud_view_->setViewerPose (kinfu_.getCameraPose ());
		}
		
		//initialize currentpointcloud buffer
		currentPointCloud_ = new CRVI::PointCloud(current_frame_cloud_view_->cloud_device_.cols(), current_frame_cloud_view_->cloud_device_.rows(), false);
	}
	
	void
		toggleIndependentCamera()
	{
		independent_camera_ = !independent_camera_;
		cout << "Camera mode: " << (independent_camera_ ?  "Independent" : "Bound to Kinect pose") << endl;
	}
		
	void execute(const PtrStepSz<const unsigned short>& depth, const PtrStepSz<const KinfuTracker::PixelRGB>& rgb24, bool has_data)
	{     
		bool has_image = false;

		//does the kinfu integration of the grabbed depth and color frame
		if (has_data)
		{			

         if(kinfu_.IsTracking())
         {
            depth_device_.upload (depth.data, depth.step, depth.rows, depth.cols);
			   if (integrate_colors_)
				   image_view_.colors_device_.upload (rgb24.data, rgb24.step, rgb24.rows, rgb24.cols);

            {
				   SampledScopeTime fps(time_ms_);
            
               //run kinfu gpu algorithm
				   if (integrate_colors_)
					   has_image = kinfu_ (depth_device_, image_view_.colors_device_);
				   else
					   has_image = kinfu_ (depth_device_);  
            }

			   // process camera pose
			   if (pose_processor_)
			   {
				   pose_processor_->processPose (kinfu_.getCameraPose ());
			   }
         }

			image_view_.showDepth (depth);
			//image_view_.showGeneratedDepth(kinfu_, kinfu_.getCameraPose());
		}
		


		//scans the tsdf cube data
		if (scan_)
		{
			scan_ = false;
			scene_cloud_view_.show (kinfu_, integrate_colors_);

			if (scan_volume_)
			{                
				cout << "Downloading TSDF volume from device ... " << flush;
				kinfu_.volume().downloadTsdfAndWeighs (tsdf_volume_.volumeWriteable (), tsdf_volume_.weightsWriteable ());
				tsdf_volume_.setHeader (Eigen::Vector3i (CRVI::Globals::resolution_X, CRVI::Globals::resolution_Y, CRVI::Globals::resolution_Z), kinfu_.volume().getSize ()); //Patrick modified
				cout << "done [" << tsdf_volume_.size () << " voxels]" << endl << endl;

				cout << "Converting volume to TSDF cloud ... " << flush;
				tsdf_volume_.convertToTsdfCloud (tsdf_cloud_ptr_);
				cout << "done [" << tsdf_cloud_ptr_->size () << " points]" << endl << endl;        
			}
			else
				cout << "[!] tsdf volume download is disabled" << endl << endl;
		}

		//scans the final mesh and shows it through the visualizer and sends data to the registered callback
		if (scan_mesh_)
		{
			scan_mesh_ = false;
			scene_cloud_view_.scanMesh(kinfu_, integrate_colors_);
			
			if(scene_cloud_view_.mesh_ptr_)
			{
				if(viz_)
				{
					scene_cloud_view_.showMesh();
				}
			
				if(mesh_ready_callback_ != nullptr)
				{
					CRVI::Mesh* mesh = nullptr;
					
					//converts the ros msg (pcl::PCLPointCloud2) into a PointCloud (easier to work with)
					//does not affect the number of points and the order of the points				
					pcl::PointCloud<pcl::PointXYZRGB> pointCloud;
					pcl::fromPCLPointCloud2(scene_cloud_view_.mesh_ptr_->cloud, pointCloud);	

					int nbPoints = pointCloud.size();
					int nbTriangles = scene_cloud_view_.mesh_ptr_->polygons.size();				

					if(nbPoints != nbTriangles * 3)
					{
						if(error_callback_ != nullptr)
						{
							(*error_callback_)(CRVI::FusionAPI::ErrorCode::MeshDataError);
						}						
					}
					else
					{
						//creates the mesh according to the data size
						mesh = new CRVI::Mesh(nbTriangles);

						//copies the triangle indexes data
						int index=0;
						for(std::vector<pcl::Vertices>::iterator it = scene_cloud_view_.mesh_ptr_->polygons.begin(); it != scene_cloud_view_.mesh_ptr_->polygons.end(); ++it)
						{
							*(mesh->triangles + index) = it->vertices[0];
							index++;
							*(mesh->triangles + index) = it->vertices[1];
							index++;
							*(mesh->triangles + index) = it->vertices[2];
							index++;
						}

						//copies original pointdata to destination pointdata				
						index=0;
						for(pcl::PointCloud<pcl::PointXYZRGB>::iterator it_src = pointCloud.begin(); it_src != pointCloud.end(); ++it_src)
						{					
							(*(mesh->points + index)).x = it_src->x;
							(*(mesh->points + index)).y = it_src->y;
							(*(mesh->points + index)).z = it_src->z;
							(*(mesh->points + index)).r = it_src->r;
							(*(mesh->points + index)).g = it_src->g;
							(*(mesh->points + index)).b = it_src->b;
							index++;
						}				

					}				

					if(mesh)
					{
						if(mesh->IsValid())
						{
							(*mesh_ready_callback_)(mesh);
						}
						delete mesh;
					}
				}

				mesh_generated_ = true;
			}			
		}

		//toggle the bounding box in the scend cloud viewer
		if(viz_ && toggle_cube_requested_)
		{
			scene_cloud_view_.toggleCube(kinfu_.volume().getSize());
			toggle_cube_requested_ = false;
		}

		//clears the scene cloud viewer memory
		if(viz_ && clear_clouds_requested_)
		{
			scene_cloud_view_.clearClouds (true);
			clear_clouds_requested_ = false;
		}
		
		//shows the 3dview of the raytracer projection to the visualizer and sends data to the registered callback
		if (has_image && (viz_ || raytracer_3dview_callback_ != nullptr))
		{
			if(viz_)
			{
				//gets viewer pose from the scene cloud viewer and generate raytracer 3d view from this viewer pose
				Eigen::Affine3f viewer_pose = getViewerPose(*scene_cloud_view_.cloud_viewer_);
				image_view_.generateScene (kinfu_, rgb24, registration_, independent_camera_ ? &viewer_pose : 0);
				image_view_.showScene ();				
			}
			else
			{
				//generate raytracer 3d view from the current camera position
				image_view_.generateScene (kinfu_, rgb24, registration_, 0);
			}

			//sends the generated scene data to the callback
			if(raytracer_3dview_callback_ != nullptr)
			{		
				memcpy(rayTracer3DView_->Data, reinterpret_cast<unsigned char*> (&image_view_.view_host_[0]), rayTracer3DView_->Width * rayTracer3DView_->Height * 3);
				if(rayTracer3DView_->IsValid())
				{
					(*raytracer_3dview_callback_)(rayTracer3DView_);
				}
			}							
		}    

		//generate and show the current frame pointcloud to the visualizer and sends data to the registered callback
		if (current_frame_cloud_view_ && (viz_ || current_point_cloud_callback_ != nullptr))
		{
			current_frame_cloud_view_->generateCurrentCloud (kinfu_);

			if(viz_)
			{
				current_frame_cloud_view_->showCurrentCloud ();    
			}
			
			if(current_point_cloud_callback_ != nullptr)
			{	
				//copies the cloud content in the cloud buffer
				currentPointCloud_->isDense = current_frame_cloud_view_->cloud_ptr_->is_dense;

				int index=0;
				for(pcl::PointCloud<pcl::PointXYZ>::iterator it_src = current_frame_cloud_view_->cloud_ptr_->begin(); it_src != current_frame_cloud_view_->cloud_ptr_->end(); ++it_src)
				{					
					(*(currentPointCloud_->points + index)).x = it_src->x;
					(*(currentPointCloud_->points + index)).y = it_src->y;
					(*(currentPointCloud_->points + index)).z = it_src->z;
					index++;
				}
				
				if(currentPointCloud_->IsValid())
				{
					(*current_point_cloud_callback_)(currentPointCloud_);	
				}
			}	
		}
				
		//sets the viewerpose of the scene cloud viewer according to the current camera position
		if (viz_ && !independent_camera_)
			setViewerPose (*scene_cloud_view_.cloud_viewer_, kinfu_.getCameraPose());		

		//resets the kinfutracker
		if(reset_requested_)
		{
			if(viz_)
			{
				scene_cloud_view_.clearClouds (true);				
			}
			kinfu_.reset();
			reset_requested_ = false;
			kinfu_.SetIsTracking(true);
			mesh_generated_ = false;
		}
	}	

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void
		startMainLoop ()
	{   	  
		signed int grabberError = 0;
		int depthWidth=0, depthHeight=0, colorWidth=0, colorHeight=0;
		CRVI::ColorFrame *colorFrame=nullptr;
		CRVI::DepthFrame *depthFrame=nullptr;

		bool need_colors = integrate_colors_ || registration_;
		bool has_depth_data;
		bool has_color_data;
		bool scene_view_not_stopped = viz_ ? !scene_cloud_view_.cloud_viewer_->wasStopped () : true;
		bool image_view_not_stopped = viz_ ? !image_view_.viewerScene_->wasStopped () : true;		
		
		//init depth frame
		depthWidth = virtualGrabber_->GetDepthStreamWidth();
		depthHeight = virtualGrabber_->GetDepthStreamHeight();
		depthFrame = new CRVI::DepthFrame(depthWidth,depthHeight);
				
		//init color frame
		if(need_colors && virtualGrabber_->IsColorEnabled())
		{
			colorWidth = virtualGrabber_->GetColorStreamWidth();
			colorHeight = virtualGrabber_->GetColorStreamHeight();
			colorFrame = new CRVI::ColorFrame(colorWidth, colorHeight, 3);		   
		}		 

		//kinfuapp main loop
		while (!exit_ && scene_view_not_stopped && image_view_not_stopped)
		{  			
			if(pause_) //do nothing
			{
				boost::this_thread::sleep(boost::posix_time::milliseconds(100));
			}
			else //do the kinfuApp code
			{
				has_depth_data = false;
				has_color_data = false;
			
				//grab current depth frame
				grabberError = virtualGrabber_->GetDepthFrame(depthFrame);
			
				if(grabberError == 0 && depthFrame->IsValid())
				{
					//sends depth frame to callback
					if(current_depth_frame_callback_ != nullptr)
					{
						(*current_depth_frame_callback_)(depthFrame);
					}

					depth_.cols = depthFrame->Width;
					depth_.rows = depthFrame->Height;
					depth_.step = depthFrame->Step;			
							
					depth_.data = (const unsigned short*)(depthFrame->Data);    
					has_depth_data = true;
				}

			
				if(need_colors && virtualGrabber_->IsColorEnabled())
				{
					//grab current color frame
					grabberError = virtualGrabber_->GetColorFrame(colorFrame); 

					if(grabberError == 0 && colorFrame->IsValid())
					{
						//sends color frame to callback
						if(current_color_frame_callback_ != nullptr)
						{
							(*current_color_frame_callback_)(colorFrame);
						}

						rgb24_.cols = colorFrame->Width;
						rgb24_.rows = colorFrame->Height;
						rgb24_.step = colorFrame->Step;					 
						rgb24_.data = (const pcl::gpu::KinfuTracker::PixelRGB *)(colorFrame->Data); 
						has_color_data = true;
					}
				}
						
				//executes the kinfuapp main function with the grabbed frames			
				if(grabberError == 0)
				{						
					try 
					{ 				
						boost::mutex::scoped_lock lock(kinFuAppMutex);	
						//main function of the kinfu app
						this->execute (depth_, rgb24_, (!need_colors && has_depth_data) || (need_colors && has_depth_data && has_color_data)); 				
					}
					catch (const std::bad_alloc& /*e*/) 
					{ 
						if(error_callback_ != nullptr)
						{
							(*error_callback_)(CRVI::FusionAPI::ErrorCode::BadAllocation);
						}
						break;
					}
					catch (const std::exception& /*e*/)
					{ 
						if(error_callback_ != nullptr)
						{
							(*error_callback_)(CRVI::FusionAPI::ErrorCode::GeneralException);
						}
						break;
					}

					//updates the scene cloud viewer
					if (viz_)
						scene_cloud_view_.cloud_viewer_->spinOnce (3);
				}
				else
				{
					if(error_callback_ != nullptr)
					{
						(*error_callback_)(CRVI::FusionAPI::ErrorCode::AcquisitionError);
					}		
				}	
			}
		}

		//cleanup
		if (depthFrame)
			delete depthFrame;

		if (colorFrame)
			delete colorFrame;
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void
		writeCloud (int format) const
	{      
		const SceneCloudView& view = scene_cloud_view_;

		// Points to export are either in cloud_ptr_ or combined_ptr_.
		// If none have points, we have nothing to export.
		if (view.cloud_ptr_->points.empty () && view.combined_ptr_->points.empty ())
		{
			cout << "Not writing cloud: Cloud is empty" << endl;
		}
		else
		{
			if(view.point_colors_ptr_->points.empty()) // no colors
			{
				if (view.valid_combined_)
					writeCloudFile (format, view.combined_ptr_);
				else
					writeCloudFile (format, view.cloud_ptr_);
			}
			else
			{        
				if (view.valid_combined_)
					writeCloudFile (format, merge<PointXYZRGBNormal>(*view.combined_ptr_, *view.point_colors_ptr_));
				else
					writeCloudFile (format, merge<PointXYZRGB>(*view.cloud_ptr_, *view.point_colors_ptr_));
			}
		}
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void
		writeMesh(int format) const
	{
		if (scene_cloud_view_.mesh_ptr_) 
			writePolygonMeshFile(format, *scene_cloud_view_.mesh_ptr_);
	}
		
	CRVI::VirtualGrabber *virtualGrabber_;	

	void(*mesh_ready_callback_)(CRVI::Mesh* mesh); 
	void(*error_callback_)(CRVI::FusionAPI::ErrorCode errorCode);	
	void(*current_depth_frame_callback_)(CRVI::DepthFrame *deptFrame);
	void(*current_color_frame_callback_)(CRVI::ColorFrame *colorFrame);
	void(*current_point_cloud_callback_)(CRVI::PointCloud *pointCloud);
	void(*raytracer_3dview_callback_)(CRVI::ColorFrame *raytracer_3dview);

	CRVI::ColorFrame *rayTracer3DView_;
	CRVI::PointCloud *currentPointCloud_;

	bool exit_;
	bool scan_;
	bool scan_mesh_;
	bool scan_volume_;
	bool mesh_generated_;
	bool toggle_cube_requested_;
	bool clear_clouds_requested_;
	bool reset_requested_;
	bool pause_;

	bool independent_camera_;

	bool registration_;
	bool integrate_colors_;	
	float focal_length_;

	KinfuTracker kinfu_;

	SceneCloudView scene_cloud_view_;
	ImageView image_view_;
	boost::shared_ptr<CurrentFrameCloudView> current_frame_cloud_view_;

	KinfuTracker::DepthMap depth_device_;

	pcl::TSDFVolume<float, short> tsdf_volume_;
	pcl::PointCloud<pcl::PointXYZI>::Ptr tsdf_cloud_ptr_;

	PtrStepSz<const unsigned short> depth_;
	PtrStepSz<const KinfuTracker::PixelRGB> rgb24_;

	int time_ms_;
	int icp_;
	int viz_;

	boost::shared_ptr<CameraPoseProcessor> pose_processor_;		
};


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename CloudPtr> void
	writeCloudFile (int format, const CloudPtr& cloud_prt)
{
	if (format == KinFuApp::PCD_BIN)
	{
		cout << "Saving point cloud to 'cloud_bin.pcd' (binary)... " << flush;
		pcl::io::savePCDFile ("cloud_bin.pcd", *cloud_prt, true);
	}
	else
		if (format == KinFuApp::PCD_ASCII)
		{
			cout << "Saving point cloud to 'cloud.pcd' (ASCII)... " << flush;
			pcl::io::savePCDFile ("cloud.pcd", *cloud_prt, false);
		}
		else   /* if (format == KinFuApp::PLY) */
		{
			cout << "Saving point cloud to 'cloud.ply' (ASCII)... " << flush;
			pcl::io::savePLYFileASCII ("cloud.ply", *cloud_prt);

		}
		cout << "Done" << endl;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void
	writePolygonMeshFile (int format, const pcl::PolygonMesh& mesh)
{
	if (format == KinFuApp::MESH_PLY)
	{
		cout << "Saving mesh to to 'mesh.ply'... " << flush;
		pcl::io::savePLYFile("mesh.ply", mesh);		
	}
	else /* if (format == KinFuApp::MESH_VTK) */
	{
		cout << "Saving mesh to to 'mesh.vtk'... " << flush;
		pcl::io::saveVTKFile("mesh.vtk", mesh);    
	}  
	cout << "Done" << endl;
}

#endif //__KINFUAPP_