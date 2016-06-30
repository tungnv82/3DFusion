#ifndef __VIRTUAL_GRABBER_
#define __VIRTUAL_GRABBER_

#include "DataTypes.hpp"
#include <string>

namespace CRVI
{
	class VirtualGrabber
	{
	public:
		virtual std::string GetErrorMessage(signed int grabberError) = 0;

		virtual bool IsColorEnabled() = 0;
		virtual signed int GetDepthFrame(CRVI::DepthFrame* &depthFrame) = 0;
		virtual signed int GetColorFrame(CRVI::ColorFrame* &colorFrame) = 0;	
		virtual int GetDepthStreamWidth() = 0;
		virtual int GetDepthStreamHeight() = 0;
		virtual int GetColorStreamWidth() = 0;
		virtual int GetColorStreamHeight() = 0;						
	};
}

#endif //__VIRTUAL_GRABBER_