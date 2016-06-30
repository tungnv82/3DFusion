#ifndef __FUSIONAPI_DATA_TYPES_
#define __FUSIONAPI_DATA_TYPES_

namespace CRVI
{
	struct ViewPoint
	{	
		float rotation[9]; //3x3 matrix expressed as 1x9 vector with a column-major storage order
		float translation[3]; //1x3
	};

	struct PointXYZ
	{	
		float x,y,z;
	};

	class PointCloud
	{
	public:			 
			int width;
			int height;
			bool isDense; // specifies if all the data in points is finite (true), or whether it might contain Inf/NaN values (false)
			PointXYZ *points;			

			PointCloud(int width_, int height_, bool isDense_)
				: points(nullptr), width(width_), height(height_), isDense(isDense_)
			{
				points = new PointXYZ[width_ * height_];				
			}

			~PointCloud()
			{	
				if (points)
					delete[] points;
			}

			int Size()
			{
				return width * height;
			}

			bool IsValid() const
			{
				if (points && width * height > 0)
					return true;

				return false;
			}
	};		

	class DepthFrame
	{
	public:
		unsigned short * Data;
		unsigned int Width;
		unsigned int Height;	
		unsigned int Step;

		DepthFrame(unsigned int width, unsigned int height)
			: Data(nullptr),
			Width(width),
			Height(height),			
			Step(Width * sizeof(unsigned short))
		{
			Data = new unsigned short[Width*Height];
		}

		~DepthFrame()
		{
			if (Data != nullptr)
				delete[] Data;
		}

		bool IsValid() const
		{
			if ((Data != nullptr) && (Width > 0) && (Height > 0))
				return true;

			return false;
		}
	};

	class ColorFrame
	{
	public:
		unsigned char * Data;
		unsigned int Width;
		unsigned int Height;
		unsigned int Channel;
		unsigned int Step;

		ColorFrame(unsigned int width, unsigned int height, unsigned int channel)
			: Data(nullptr),
			Width(width),
			Height(height),
			Channel(channel),
			Step(Width*Channel)
		{
			Data = new unsigned char[Width*Channel*Height];
		}

		~ColorFrame()
		{
			if (Data != nullptr)
				delete[] Data;
		}

		bool IsValid() const
		{
			if ((Data != nullptr) && (Width > 0) && (Height > 0))
				return true;

			return false;
		}
	};
}

#endif //__FUSIONAPI_DATA_TYPES_

#ifndef __MESH_COMMON_DATATYPES__
#define __MESH_COMMON_DATATYPES__

namespace CRVI
{
	struct PointXYZRGB
	{	
		float x,y,z;
		unsigned char r,g,b;
	};
		
	class Mesh
	{
		public:
			int nbrTriangles; 
			int nbrPoints;
			int* triangles;
			PointXYZRGB *points;

			Mesh(int nbTriangles)
				: triangles(nullptr),
				points(nullptr),
				nbrTriangles(nbTriangles),			
				nbrPoints(nbTriangles * 3)
			{
				points = new PointXYZRGB[nbrPoints];
				triangles = new int[3*nbrTriangles];
			}

			~Mesh()
			{				
				if (triangles)
					delete[] triangles;

				if (points)
					delete[] points;
			}

			bool IsValid() const
			{
				if (points && triangles && nbrPoints > 2 && nbrTriangles > 0)
					return true;

				return false;
			}
	};
}

#endif //__MESH_COMMON_DATATYPES__
