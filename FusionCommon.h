#ifndef __FUSION_COMMON_
#define __FUSION_COMMON_

#ifdef FUSION_EXPORTS
#define FUSION_API __declspec(dllexport)
#else
#define FUSION_API __declspec(dllimport)
#endif

#endif

