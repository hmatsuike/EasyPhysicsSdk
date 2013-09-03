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

#include "EpxCollisionDetection.h"
#include "../collision/EpxConvexConvexContact.h"
#include "../collision/EpxConvexSphereClosest.h"

namespace EasyPhysics {

void epxDetectCollision(
	const EpxState *states,
	const EpxCollidable *collidables,
	EpxUInt32 numRigidBodies,
	const EpxPair *pairs,
	EpxUInt32 numPairs)
{
	EPX_ALWAYS_ASSERT(states);
	EPX_ALWAYS_ASSERT(collidables);
	EPX_ALWAYS_ASSERT(pairs);
	
	for(EpxUInt32 i=0;i<numPairs;i++) {
		const EpxPair &pair = pairs[i];
		
		EPX_ALWAYS_ASSERT(pair.contact);

		const EpxState &stateA = states[pair.rigidBodyA];
		const EpxState &stateB = states[pair.rigidBodyB];
		const EpxCollidable &collA = collidables[pair.rigidBodyA];
		const EpxCollidable &collB = collidables[pair.rigidBodyB];
		
		EpxTransform3 transformA(stateA.m_orientation,stateA.m_position);
		EpxTransform3 transformB(stateB.m_orientation,stateB.m_position);
		
		pair.contact->m_numClosestPoints = 0;

		for(EpxUInt32 j=0;j<collA.m_numShapes;j++) {
			const EpxShape &shapeA = collA.m_shapes[j];
			EpxTransform3 offsetTransformA(shapeA.m_offsetOrientation,shapeA.m_offsetPosition);
			EpxTransform3 worldTransformA = transformA * offsetTransformA;
			
			for(EpxUInt32 k=0;k<collB.m_numShapes;k++) {
				const EpxShape &shapeB = collB.m_shapes[k];
				EpxTransform3 offsetTransformB(shapeB.m_offsetOrientation,shapeB.m_offsetPosition);
				EpxTransform3 worldTransformB = transformB * offsetTransformB;
				
				EpxVector3 contactPointA;
				EpxVector3 contactPointB;
				EpxVector3 normal;
				EpxFloat penetrationDepth;
				
				if(epxConvexConvexContact(
					shapeA.m_geometry,worldTransformA,
					shapeB.m_geometry,worldTransformB,
					normal,penetrationDepth,
					contactPointA,contactPointB) && penetrationDepth < 0.0f) {
					
					// 衝突点を剛体の座標系へ変換し、コンタクトへ追加する。
					pair.contact->addContactPoint(
						penetrationDepth,normal,
						offsetTransformA.getTranslation() + offsetTransformA.getUpper3x3() * contactPointA,
						offsetTransformB.getTranslation() + offsetTransformB.getUpper3x3() * contactPointB);
				}
			}
		}

		//continue; // Disable CCD

		for(EpxUInt32 j=0;j<collA.m_numCoreSpheres;j++) {
			EpxTransform3 worldTransformA = transformA * EpxTransform3::translation(collA.m_coreSpheres[j].m_offsetPosition);
			EpxSphere sphereA;
			sphereA.m_radius = collA.m_coreSpheres[j].m_radius;

			for(EpxUInt32 k=0;k<collB.m_numShapes;k++) {
				const EpxShape &shapeB = collB.m_shapes[k];
				EpxTransform3 offsetTransformB(shapeB.m_offsetOrientation,shapeB.m_offsetPosition);
				EpxTransform3 worldTransformB = transformB * offsetTransformB;

				EpxVector3 closestPointA;
				EpxVector3 closestPointB;
				EpxVector3 normal;
				EpxFloat distance;

				if(epxConvexSphereClosest(
					shapeB.m_geometry,worldTransformB,
					sphereA,worldTransformA,
					normal,	distance,
					closestPointB,closestPointA)) {
					// 最近接点を剛体の座標系へ変換し、コンタクトへ追加する。
					pair.contact->addClosestPoint(
						distance,-normal,
						collA.m_coreSpheres[j].m_offsetPosition + closestPointA,
						offsetTransformB.getTranslation() + offsetTransformB.getUpper3x3() * closestPointB);
				}
			}
		}

		for(EpxUInt32 j=0;j<collB.m_numCoreSpheres;j++) {
			EpxTransform3 worldTransformB = transformB * EpxTransform3::translation(collB.m_coreSpheres[j].m_offsetPosition);
			EpxSphere sphereB;
			sphereB.m_radius = collB.m_coreSpheres[j].m_radius;

			for(EpxUInt32 k=0;k<collA.m_numShapes;k++) {
				const EpxShape &shapeA = collA.m_shapes[k];
				EpxTransform3 offsetTransformA(shapeA.m_offsetOrientation,shapeA.m_offsetPosition);
				EpxTransform3 worldTransformA = transformA * offsetTransformA;

				EpxVector3 closestPointA;
				EpxVector3 closestPointB;
				EpxVector3 normal;
				EpxFloat distance;

				if(epxConvexSphereClosest(
					shapeA.m_geometry,worldTransformA,
					sphereB,worldTransformB,
					normal,	distance,
					closestPointA,closestPointB)) {
					// 最近接点を剛体の座標系へ変換し、コンタクトへ追加する。
					pair.contact->addClosestPoint(
						distance,normal,
						offsetTransformA.getTranslation() + offsetTransformA.getUpper3x3() * closestPointA,
						collB.m_coreSpheres[j].m_offsetPosition + closestPointB);
				}
			}
		}
	}
}

} // namespace EasyPhysics
