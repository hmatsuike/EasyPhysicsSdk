/*
	Copyright (c) 2013 Hiroshi Matsuike

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

#include "EpxConvexSphereClosest.h"
#include "EpxClosestFunction.h"

namespace EasyPhysics {

EpxBool epxConvexSphereClosest(
	const EpxConvexMesh &convexA,const EpxTransform3 &transformA,
	const EpxSphere &sphereB,const EpxTransform3 &transformB,
	EpxVector3 &normal,
	EpxFloat &distance,
	EpxVector3 &closestPointA,
	EpxVector3 &closestPointB)
{
	EpxTransform3 transformAB,transformBA;
	EpxMatrix3 matrixAB,matrixBA;
	EpxVector3 offsetAB,offsetBA;
	
	// Bローカル→Aローカルへの変換
	transformAB = orthoInverse(transformA) * transformB;
	matrixAB = transformAB.getUpper3x3();
	offsetAB = transformAB.getTranslation();
	
	// Aローカル→Bローカルへの変換
	transformBA = orthoInverse(transformAB);
	matrixBA = transformBA.getUpper3x3();
	offsetBA = transformBA.getTranslation();
	
	EpxFloat closestMinSqr = EPX_FLT_MAX;
	EpxVector3 closestMinPointA;
	EpxVector3 closestMinNormal;
	
	for(EpxUInt32 f=0;f<convexA.m_numFacets;f++) {
		const EpxFacet &facetA = convexA.m_facets[f];
		
		EpxVector3 triangleA[3] = {
			convexA.m_vertices[facetA.vertId[0]],
			convexA.m_vertices[facetA.vertId[1]],
			convexA.m_vertices[facetA.vertId[2]],
		};
		
		if(dot(offsetAB-triangleA[0],facetA.normal) > 0.0f) {
			EpxVector3 s;
			epxGetClosestPointTriangle(offsetAB,triangleA[0],triangleA[1],triangleA[2],facetA.normal,s);
			EpxFloat dSqr = lengthSqr(offsetAB-s);
			if(dSqr < closestMinSqr) {
				closestMinSqr = dSqr;
				closestMinPointA = s;
				closestMinNormal = facetA.normal;
			}
		}
	}
	
	if(closestMinSqr == EPX_FLT_MAX) {
		return false;
	}
	
	if(closestMinSqr > EPX_EPSILON) {
		EpxFloat distAB = sqrtf(closestMinSqr);
		distance = distAB - sphereB.m_radius;
		normal = normalize(closestMinPointA - offsetAB);
	}
	else {
		distance = - sphereB.m_radius;
		normal = -closestMinNormal;
	}
	
#if 0
	// distanceが負のとき、falseを返す
	if(distance < 0.0f) {
		return false;
	}
#endif

	closestPointA = closestMinPointA;
	EpxVector3 closestPointB_ = offsetAB + sphereB.m_radius * normal;
	closestPointB = offsetBA + matrixBA * closestPointB_;
	
	normal = transformA.getUpper3x3() * normal;

	return true;
}

} // namespace EasyPhysics
