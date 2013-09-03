/*
	Copyright (c) 2012 Hiroshi Matsuike

	This software is provided 'as-is', without any express or implied
	warranty. In no event will the authors be held liable for any damages
	arising from the use of this software.

	Permission is granted to anyone to use this software for any purpose,
	including commercial applications, and to alter it and redistribute it
	freely, subject to the following restrictions:

	1. The origin of this software must not be misrepresented; you must not
	claim that you wrote the original software. If you use this software
	in a product, an acknowledgment in the product documentation would be
	appreciated but is not required.

	2. Altered source versions must be plainly marked as such, and must not be
	misrepresented as being the original software.

	3. This notice may not be removed or altered from any source distribution.
*/

#ifndef EPX_BASE_H
#define EPX_BASE_H

#include <assert.h>
#include <stdio.h>
#include <float.h>
#include <windows.h>
#include <malloc.h>

#if _MSC_VER >= 1600
	#include <stdint.h>
#endif

#include "../extra/vectormath/vectormath_aos.h"

#define EPX_PI		3.14159265358979f
#define EPX_EPSILON	1e-5f
#define EPX_FLT_MAX FLT_MAX

#define EPX_MIN(a,b) (((a)<(b))?(a):(b))
#define EPX_MAX(a,b) (((a)>(b))?(a):(b))
#define EPX_CLAMP(v,a,b) EPX_MAX(a,EPX_MIN(v,b))

namespace EasyPhysics {

#if _MSC_VER < 1600
	typedef signed char         		EpxInt8;
	typedef unsigned char       		EpxUInt8;
	typedef signed short        		EpxInt16;
	typedef unsigned short      		EpxUInt16;
	typedef signed int          		EpxInt32;
	typedef unsigned int        		EpxUInt32;
	typedef signed long long    		EpxInt64;
	typedef unsigned long long  		EpxUInt64;
	typedef float						EpxFloat;
	typedef bool						EpxBool;	
#else
	typedef int8_t             			EpxInt8;
	typedef uint8_t            			EpxUInt8;
	typedef int16_t            			EpxInt16;
	typedef uint16_t           			EpxUInt16;
	typedef int32_t            			EpxInt32;
	typedef uint32_t           			EpxUInt32;
	typedef int64_t            			EpxInt64;
	typedef uint64_t           			EpxUInt64;
	typedef float						EpxFloat;
	typedef bool						EpxBool;
#endif

typedef Vectormath::Aos::Point3     EpxPoint3;
typedef Vectormath::Aos::Vector3    EpxVector3;
typedef Vectormath::Aos::Vector4    EpxVector4;
typedef Vectormath::Aos::Quat       EpxQuat;
typedef Vectormath::Aos::Matrix3    EpxMatrix3;
typedef Vectormath::Aos::Matrix4    EpxMatrix4;
typedef Vectormath::Aos::Transform3 EpxTransform3;

static inline void epxPrintf(const char *str, ...)
{
    char strDebug[1024]={0};
    va_list argList;
    va_start(argList, str);
    vsprintf_s(strDebug,str,argList);
	OutputDebugStringA(strDebug);
    va_end(argList);
}

// Assert
#ifdef _DEBUG
	#define EPX_ASSERT(test) {if(!(test)){EasyPhysics::epxPrintf("Assert " __FILE__ ":%d ("#test")\n", __LINE__);__debugbreak();}}
	#define EPX_ASSERT_MSG(test,msg) {if(!(test)){EasyPhysics::epxPrintf("Assert " msg " " __FILE__ ":%d ("#test")\n",__LINE__);__debugbreak();}}
#else
	#define EPX_ASSERT(test)
	#define EPX_ASSERT_MSG(test,msg)
#endif

#define EPX_ALWAYS_ASSERT(test) {if(!(test)){EasyPhysics::epxPrintf("Assert " __FILE__ ":%d ("#test")\n", __LINE__);__debugbreak();}}
#define EPX_ALWAYS_ASSERT_MSG(test,msg) {if(!(test)){EasyPhysics::epxPrintf("Assert:" msg " " __FILE__ ":%d ("#test")\n",__LINE__);__debugbreak();}}

// Utility functions

static inline
EpxUInt32 epxConvertToUInt32(EpxFloat flt)
{
	EpxUInt32 iflt = *((EpxUInt32*)&flt);
	EpxInt32 msb = (EpxInt32)(iflt>>31);
	EpxUInt32 msbMask = -msb;
	EpxUInt32 maskSignFlip = msbMask | 0x80000000;
	return iflt ^ maskSignFlip;
}

static inline
void *epxAlloca(size_t bytes)
{
	return _alloca(bytes);
}

} // namespace EasyPhysics

#endif // EPX_BASE_H
