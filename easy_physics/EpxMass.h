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

#ifndef EPX_MASS_H
#define EPX_MASS_H

#include "EpxBase.h"

namespace EasyPhysics {

static inline 
EpxFloat epxCalcMassBox(EpxFloat density,const EpxVector3 &halfExtent)
{
	return density * halfExtent[0] * halfExtent[1] * halfExtent[2] * 8;
}

static inline 
EpxMatrix3 epxCalcInertiaBox(const EpxVector3 &halfExtent,EpxFloat mass)
{
	EpxVector3 sqrSz = halfExtent * 2.0f;
	sqrSz = mulPerElem(sqrSz,sqrSz);
	EpxMatrix3 inertia = EpxMatrix3::identity();
	inertia[0][0] = (mass*(sqrSz[1]+sqrSz[2]))/12.0f;
	inertia[1][1] = (mass*(sqrSz[0]+sqrSz[2]))/12.0f;
	inertia[2][2] = (mass*(sqrSz[0]+sqrSz[1]))/12.0f;
	return inertia;
}

static inline 
EpxFloat epxCalcMassSphere(EpxFloat density,EpxFloat radius)
{
	return (4.0f/3.0f) * EPX_PI * radius * radius * radius * density;
}

static inline 
EpxMatrix3 epxCalcInertiaSphere(EpxFloat radius,EpxFloat mass)
{
	EpxMatrix3 inertia = EpxMatrix3::identity();
	inertia[0][0] = inertia[1][1] = inertia[2][2] = 0.4f * mass * radius * radius;
	return inertia;
}

} // namespace EasyPhysics

#endif // EPX_MASS_H
