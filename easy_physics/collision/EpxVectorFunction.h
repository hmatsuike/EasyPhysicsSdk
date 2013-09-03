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

#ifndef EPX_VECTOR_FUNCTION_H
#define EPX_VECTOR_FUNCTION_H

namespace EasyPhysics {

static inline void epxCalcTangentVector(
	const EpxVector3 &normal,
	EpxVector3 &tangent1,
	EpxVector3 &tangent2)
{
	if(fabs(normal[2]) > 0.707f) {
		// y-z plane
		EpxFloat a(normal[1]*normal[1] + normal[2]*normal[2]);
		EpxFloat k = 1.0f/sqrtf(a);
		tangent1 = EpxVector3(0.0f,-normal[2],normal[1]) * k;
		tangent2 = cross(normal,tangent1);
	}
	else {
		// x-y plane
		EpxFloat a(normal[0]*normal[0] + normal[1]*normal[1]);
		EpxFloat k = 1.0f/sqrtf(a);
		tangent1 = EpxVector3(-normal[1],normal[0],0.0f) * k;
		tangent2 = cross(normal,tangent1);
	}
}

static inline EpxTransform3 epxIntegrateTransform(
	EpxFloat lambda,
	const EpxTransform3 &transform,
	const EpxVector3 &linearVelocity,
	const EpxVector3 &angularVelocity)
{
	EpxQuat orientation(transform.getUpper3x3());
	EpxQuat dq = EpxQuat(angularVelocity,0.0f) * orientation * 0.5f;
	return EpxTransform3(
		normalize(orientation + dq * lambda),
		transform.getTranslation() + linearVelocity * lambda);
}

} // namespace EasyPhysics

#endif // EPX_VECTOR_FUNCTION_H
