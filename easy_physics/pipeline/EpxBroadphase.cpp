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

#include "EpxBroadphase.h"
#include "EpxSort.h"
#include "../collision/EpxVectorFunction.h"

#define EPX_AABB_EXPAND 0.01f

namespace EasyPhysics {

struct EpxSortData {
	EpxUInt32 key;
	EpxUInt32 data;
};

struct EpxBroadPhaseProxy {
	EpxFloat aabb[6];
};

static inline 
EpxBool epxIntersectAABB(const EpxVector3 &centerA,const EpxVector3 &halfA,const EpxVector3 &centerB,const EpxVector3 &halfB)
{
	if(fabs(centerA[0] - centerB[0]) > halfA[0] + halfB[0]) return false;
	if(fabs(centerA[1] - centerB[1]) > halfA[1] + halfB[1]) return false;
	if(fabs(centerA[2] - centerB[2]) > halfA[2] + halfB[2]) return false;
	return true;
}

void epxBroadPhase(
	const EpxState *states,
	const EpxCollidable *collidables,
	EpxUInt32 numRigidBodies,
	const EpxPair *oldPairs,
	const EpxUInt32 numOldPairs,
	EpxPair *newPairs,
	EpxUInt32 &numNewPairs,
	const EpxUInt32 maxPairs,
	EpxFloat timeStep,
	EpxAllocator *allocator,
	EpxStackAllocator *stackAllocator,
	void *userData,
	epxBroadPhaseCallback callback)
{
	EPX_ALWAYS_ASSERT(states);
	EPX_ALWAYS_ASSERT(collidables);
	EPX_ALWAYS_ASSERT(oldPairs);
	EPX_ALWAYS_ASSERT(newPairs);
	EPX_ALWAYS_ASSERT(allocator);
	EPX_ALWAYS_ASSERT(stackAllocator);

	numNewPairs = 0;
	
	// AABB交差ペアを見つける
#if 1 // sort and sweep
	EpxBroadPhaseProxy *proxies = (EpxBroadPhaseProxy*)stackAllocator->allocate(sizeof(EpxBroadPhaseProxy)*numRigidBodies);
	EpxSortData *sortData = (EpxSortData*)stackAllocator->allocate(sizeof(EpxSortData)*numRigidBodies);

	// 剛体が最も分散している軸を見つける
	int axis = 0;
	{
		EpxVector3 s(0.0f),s2(0.0f);
		for(EpxUInt32 i=0;i<numRigidBodies;i++) {
			EpxVector3 c = states[i].m_position;
			s += c;
			s2 += mulPerElem(c,c);
		}
		EpxVector3 v = s2 - mulPerElem(s,s) / (float)numRigidBodies;
		if(v[1] > v[0]) axis = 1;
		if(v[2] > v[axis]) axis = 2;
	}
	
	// プロキシを軸上に並べる
	for(int i=0;i<numRigidBodies;i++) {
		const EpxState &state = states[i];
		const EpxCollidable &collidable = collidables[i];
		
		EpxVector3 aabbMin,aabbMax;
		
		if(collidable.m_numCoreSpheres == 0) {
			EpxMatrix3 orientation(state.m_orientation);
			EpxVector3 center = state.m_position + orientation * collidable.m_center;
			EpxVector3 extent = absPerElem(orientation) * ( collidable.m_half + EpxVector3(EPX_AABB_EXPAND) );// AABBサイズを若干拡張
			aabbMin = center - extent;
			aabbMax = center + extent;
		}
		else {
			// Core Sphereが含まれる場合、AABBを進む距離に応じて拡大
			EpxTransform3 tr0(state.m_orientation,state.m_position);
			EpxTransform3 tr1 = epxIntegrateTransform(timeStep,tr0,state.m_linearVelocity,state.m_angularVelocity);
			EpxVector3 center0 = tr0.getTranslation() + tr0.getUpper3x3() * collidable.m_center;
			EpxVector3 center1 = tr1.getTranslation() + tr1.getUpper3x3() * collidable.m_center;
			EpxVector3 radius(length(collidable.m_half) + EPX_AABB_EXPAND);
			aabbMin = minPerElem(minPerElem(minPerElem(center0 + radius,center0 - radius),center1 + radius),center1 - radius);
			aabbMax = maxPerElem(maxPerElem(maxPerElem(center0 + radius,center0 - radius),center1 + radius),center1 - radius);
		}
		
		proxies[i].aabb[0] = aabbMin[0];
		proxies[i].aabb[1] = aabbMin[1];
		proxies[i].aabb[2] = aabbMin[2];
		proxies[i].aabb[3] = aabbMax[0];
		proxies[i].aabb[4] = aabbMax[1];
		proxies[i].aabb[5] = aabbMax[2];
		
		sortData[i].key = epxConvertToUInt32(aabbMin[axis]);
		sortData[i].data = i;
	}
	
	// プロキシをソート
	{
		EpxSortData *sortBuff = (EpxSortData*)stackAllocator->allocate(sizeof(EpxSortData)*numRigidBodies);
		epxSort<EpxSortData>(sortData,sortBuff,numRigidBodies);
		stackAllocator->deallocate(sortBuff);
	}
	
	// 交差ペア検出
	for(EpxUInt32 i=0;i<numRigidBodies;i++) {
		for(EpxUInt32 j=i+1;j<numRigidBodies;j++) {
			EpxUInt32 idA = sortData[i].data;
			EpxUInt32 idB = sortData[j].data;
			EpxBroadPhaseProxy &proxyA = proxies[idA];
			EpxBroadPhaseProxy &proxyB = proxies[idB];
			
			if(proxyA.aabb[axis+3] < proxyB.aabb[axis]) {
				break;
			}
			
			if(proxyA.aabb[3] < proxyB.aabb[0] || proxyA.aabb[0] > proxyB.aabb[3]) continue;
			if(proxyA.aabb[4] < proxyB.aabb[1] || proxyA.aabb[1] > proxyB.aabb[4]) continue;
			if(proxyA.aabb[5] < proxyB.aabb[2] || proxyA.aabb[2] > proxyB.aabb[5]) continue;
			
			if(callback && !callback(idA,idB,userData)) {
				continue;
			}
			
			if(numNewPairs < maxPairs) {
				EpxPair &newPair = newPairs[numNewPairs++];
				newPair.rigidBodyA = idA<idB?idA:idB;
				newPair.rigidBodyB = idA<idB?idB:idA;
				newPair.contact = NULL;
			}
		}
	}
	
	stackAllocator->deallocate(sortData);
	stackAllocator->deallocate(proxies);
	
#else // brute force
	for(EpxUInt32 i=0;i<numRigidBodies;i++) {
		for(EpxUInt32 j=i+1;j<numRigidBodies;j++) {
			const EpxState &stateA = states[i];
			const EpxCollidable &collidableA = collidables[i];
			const EpxState &stateB = states[j];
			const EpxCollidable &collidableB = collidables[j];

			if(callback && !callback(i,j,userData)) {
				continue;
			}
			
			EpxMatrix3 orientationA(stateA.m_orientation);
			EpxVector3 centerA = stateA.m_position + orientationA * collidableA.m_center;
			EpxVector3 halfA = absPerElem(orientationA) * ( collidableA.m_half + EpxVector3(EPX_AABB_EXPAND) );// AABBサイズを若干拡張
			
			EpxMatrix3 orientationB(stateB.m_orientation);
			EpxVector3 centerB = stateB.m_position + orientationB * collidableB.m_center;
			EpxVector3 halfB = absPerElem(orientationB) * ( collidableB.m_half + EpxVector3(EPX_AABB_EXPAND) );// AABBサイズを若干拡張
			
			if(epxIntersectAABB(centerA,halfA,centerB,halfB) && numNewPairs < maxPairs) {
				EpxPair &newPair = newPairs[numNewPairs++];
				
				newPair.rigidBodyA = i<j?i:j;
				newPair.rigidBodyB = i<j?j:i;
				newPair.contact = NULL;
			}
		}
	}
#endif
	
	// ソート
	{
		EpxPair *sortBuff = (EpxPair*)stackAllocator->allocate(sizeof(EpxPair)*numNewPairs);
		epxSort<EpxPair>(newPairs,sortBuff,numNewPairs);
		stackAllocator->deallocate(sortBuff);
	}
	
	// 新しく検出したペアと過去のペアを比較
	EpxPair *outNewPairs = (EpxPair*)stackAllocator->allocate(sizeof(EpxPair)*numNewPairs);
	EpxPair *outKeepPairs = (EpxPair*)stackAllocator->allocate(sizeof(EpxPair)*numOldPairs);
	EPX_ASSERT(outNewPairs);
	EPX_ASSERT(outKeepPairs);
	
	EpxUInt32 nNew = 0;
	EpxUInt32 nKeep = 0;
	
	EpxUInt32 oldId = 0,newId = 0;
	
	while(oldId<numOldPairs&&newId<numNewPairs) {
		if(newPairs[newId].key > oldPairs[oldId].key) {
			// remove
			allocator->deallocate(oldPairs[oldId].contact);
			oldId++;
		}
		else if(newPairs[newId].key == oldPairs[oldId].key) {
			// keep
			EPX_ASSERT(nKeep<=numOldPairs);
			outKeepPairs[nKeep] = oldPairs[oldId];
			nKeep++;
			oldId++;
			newId++;
		}
		else {
			// new
			EPX_ASSERT(nNew<=numNewPairs);
			outNewPairs[nNew] = newPairs[newId];
			nNew++;
			newId++;
		}
	};
	
	if(newId<numNewPairs) {
		// all new
		for(;newId<numNewPairs;newId++,nNew++) {
			EPX_ASSERT(nNew<=numNewPairs);
			outNewPairs[nNew] = newPairs[newId];
		}
	}
	else if(oldId<numOldPairs) {
		// all remove
		for(;oldId<numOldPairs;oldId++) {
			allocator->deallocate(oldPairs[oldId].contact);
		}
	}
	
	for(EpxUInt32 i=0;i<nNew;i++) {
		outNewPairs[i].contact = (EpxContact*)allocator->allocate(sizeof(EpxContact));
		outNewPairs[i].contact->reset();
	}
	
	for(EpxUInt32 i=0;i<nKeep;i++) {
		outKeepPairs[i].contact->refresh(
			states[outKeepPairs[i].rigidBodyA].m_position,
			states[outKeepPairs[i].rigidBodyA].m_orientation,
			states[outKeepPairs[i].rigidBodyB].m_position,
			states[outKeepPairs[i].rigidBodyB].m_orientation);
	}
	
	numNewPairs = 0;
	for(EpxUInt32 i=0;i<nKeep;i++) {
		outKeepPairs[i].type = EpxPairTypeKeep;
		newPairs[numNewPairs++] = outKeepPairs[i];
	}
	for(EpxUInt32 i=0;i<nNew&&numNewPairs<maxPairs;i++) {
		outNewPairs[i].type = EpxPairTypeNew;
		newPairs[numNewPairs++] = outNewPairs[i];
	}
	
	stackAllocator->deallocate(outKeepPairs);
	stackAllocator->deallocate(outNewPairs);
	
	// ソート
	{
		EpxPair *sortBuff = (EpxPair*)stackAllocator->allocate(sizeof(EpxPair)*numNewPairs);
		epxSort<EpxPair>(newPairs,sortBuff,numNewPairs);
		stackAllocator->deallocate(sortBuff);
	}
}

} // namespace EasyPhysics
