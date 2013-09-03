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

#include "EpxConvexConvexContact.h"
#include "EpxClosestFunction.h"

namespace EasyPhysics {

#ifndef EPX_DOXYGEN_SKIP

enum EpxSatType {
	EpxSatTypePointAFacetB,
	EpxSatTypePointBFacetA,
	EpxSatTypeEdgeEdge,
};

/*
#define EPX_CHECK_MINMAX(axis,AMin,AMax,BMin,BMax,type) \
{\
	satCount++;\
	EpxFloat d1 = AMin - BMax;\
	EpxFloat d2 = BMin - AMax;\
	if(d1 >= 0.0f || d2 >= 0.0f) {\
		return false;\
	}\
	if(distanceMin < d1) {\
		distanceMin = d1;\
		axisMin = axis;\
		satType = type;\
		axisFlip = false;\
	}\
	if(distanceMin < d2) {\
		distanceMin = d2;\
		axisMin = -axis;\
		satType = type;\
		axisFlip = true;\
	}\
}
*/
EpxBool isValidEdge(const EpxVector3 &eA0,const EpxVector3 &eA1,const EpxVector3 &eB0,const EpxVector3 &eB1)
{
	EpxVector3 eAX = cross(eA1,eA0);
	EpxVector3 eBX = cross(eB1,eB0);
	EpxFloat a = dot(eB0,eAX);
	EpxFloat b = dot(eB1,eAX);
	EpxFloat c = dot(eA0,eBX);
	EpxFloat d = dot(eA1,eBX);
	return a * b < 0.0f && c * d < 0.0f && a * d > 0.0f;
}

EpxBool epxConvexConvexContact_local(
	const EpxConvexMesh &convexA,const EpxTransform3 &transformA,
	const EpxConvexMesh &convexB,const EpxTransform3 &transformB,
	EpxVector3 &normal,
	EpxFloat &penetrationDepth,
	EpxVector3 &contactPointA,
	EpxVector3 &contactPointB)
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
	
	// 最も浅い貫通深度とそのときの分離軸
	EpxFloat distanceMin = -EPX_FLT_MAX;
	EpxVector3 axisMin(0.0f);
	EpxSatType satType = EpxSatTypeEdgeEdge;
	EpxBool axisFlip;
	
	//----------------------------------------------------------------------------
	// 分離軸判定
	
	int satCount = 0;

	// ConvexAの面法線を分離軸とする
	for(EpxUInt32 f=0;f<convexA.m_numFacets;f++) {
		const EpxFacet &facet = convexA.m_facets[f];
		const EpxVector3 separatingAxis = facet.normal;
		
#if 1
		EpxVector3 axisB = matrixBA * separatingAxis;
		EpxVector3 pA = offsetBA + matrixBA * convexA.m_vertices[facet.vertId[0]];
		EpxFloat minB = EPX_FLT_MAX;
		for(EpxUInt32 i=0;i<convexB.m_numVertices;i++) {
			EpxFloat prj = dot(axisB,convexB.m_vertices[i] - pA);
			minB = EPX_MIN(minB,prj);
		}

		satCount++;
		if(minB >= 0.0f) {
			return false;
		}
		if(distanceMin < minB) {
			distanceMin = minB;
			axisMin = -separatingAxis;
			satType = EpxSatTypePointBFacetA;
			axisFlip = true;
		}
#else
		// ConvexAを分離軸に投影
		EpxFloat minA,maxA;
		epxGetProjection(minA,maxA,&convexA,separatingAxis);
		
		// ConvexBを分離軸に投影
		EpxFloat minB,maxB;
		epxGetProjection(minB,maxB,&convexB,matrixBA * facet.normal);
		EpxFloat offset = dot(offsetAB,separatingAxis);
		minB += offset;
		maxB += offset;
		
		// 判定
		EPX_CHECK_MINMAX(separatingAxis,minA,maxA,minB,maxB,EpxSatTypePointBFacetA);
#endif
	}
		
	// ConvexBの面法線を分離軸とする
	for(EpxUInt32 f=0;f<convexB.m_numFacets;f++) {
		const EpxFacet &facet = convexB.m_facets[f];
		const EpxVector3 separatingAxis = matrixAB * facet.normal;
		
#if 1
		EpxVector3 pB = offsetAB + matrixAB * convexB.m_vertices[facet.vertId[0]];
		EpxFloat minA = EPX_FLT_MAX;
		for(EpxUInt32 i=0;i<convexA.m_numVertices;i++) {
			EpxFloat prj = dot(separatingAxis,convexA.m_vertices[i] - pB);
			minA = EPX_MIN(minA,prj);
		}

		satCount++;
		if(minA >= 0.0f) {
			return false;
		}
		if(distanceMin < minA) {
			distanceMin = minA;
			axisMin = separatingAxis;
			satType = EpxSatTypePointAFacetB;
			axisFlip = false;
		}
#else
		// ConvexAを分離軸に投影
		EpxFloat minA,maxA;
		epxGetProjection(minA,maxA,&convexA,separatingAxis);
		
		// ConvexBを分離軸に投影
		EpxFloat minB,maxB;
		epxGetProjection(minB,maxB,&convexB,facet.normal);
		EpxFloat offset = dot(offsetAB,separatingAxis);
		minB += offset;
		maxB += offset;
		
		// 判定
		EPX_CHECK_MINMAX(separatingAxis,minA,maxA,minB,maxB,EpxSatTypePointAFacetB);
#endif
	}
		
	// ConvexAとConvexBの外積を分離軸とする
	for(EpxUInt32 eA=0;eA<convexA.m_numEdges;eA++) {
		const EpxEdge &edgeA = convexA.m_edges[eA];
		if(edgeA.type != EpxEdgeTypeConvex) continue;

		const EpxVector3 edgeVecA = convexA.m_vertices[edgeA.vertId[1]] - convexA.m_vertices[edgeA.vertId[0]];
		
		for(EpxUInt32 eB=0;eB<convexB.m_numEdges;eB++) {
			const EpxEdge &edgeB = convexB.m_edges[eB];
			if(edgeB.type != EpxEdgeTypeConvex) continue;
				
			const EpxVector3 edgeVecB = matrixAB * (convexB.m_vertices[edgeB.vertId[1]] - convexB.m_vertices[edgeB.vertId[0]]);

			// Gauss map algorithm from physics tutorial at GDC 2013
			EpxVector3 eA0 = convexA.m_facets[edgeA.facetId[0]].normal;
			EpxVector3 eA1 = convexA.m_facets[edgeA.facetId[1]].normal;
			EpxVector3 eB0 = -matrixAB * convexB.m_facets[edgeB.facetId[0]].normal;
			EpxVector3 eB1 = -matrixAB * convexB.m_facets[edgeB.facetId[1]].normal;
			if(!isValidEdge(eA0,eA1,eB0,eB1)) continue;
			
			EpxVector3 separatingAxis = cross(edgeVecA,edgeVecB);
			if(lengthSqr(separatingAxis) < EPX_EPSILON*EPX_EPSILON) continue;
			
			separatingAxis = normalize(separatingAxis);
		
#if 1
			EpxVector3 pA = convexA.m_vertices[edgeA.vertId[0]];
			EpxVector3 pB = offsetAB + matrixAB * convexB.m_vertices[edgeB.vertId[0]];

			if(dot(separatingAxis,pA) > 0.0f) { // 原点は必ずConvexの内側に存在すること
				separatingAxis = -separatingAxis;
			}
			
			EpxFloat d = dot(separatingAxis,pA - pB);
			
			satCount++;
			if(d >= 0.0f) {
				return false;
			}
			if(distanceMin < d) {
				distanceMin = d;
				axisMin = separatingAxis;
				satType = EpxSatTypeEdgeEdge;
			}
#else			
			EpxVector3 pA = convexA.m_vertices[edgeA.vertId[0]];
			EpxVector3 pB = offsetAB + matrixAB * convexB.m_vertices[edgeB.vertId[1]];
			
			EpxFloat chk1 = dot(separatingAxis,pA);
			EpxFloat dd1 = dot(separatingAxis,pA - pB);
			EpxFloat dd2 = dot(separatingAxis,pB - pA);

			// ConvexAを分離軸に投影
			EpxFloat minA,maxA;
			epxGetProjection(minA,maxA,&convexA,separatingAxis);
				
			// ConvexBを分離軸に投影
			EpxFloat minB,maxB;
			epxGetProjection(minB,maxB,&convexB,matrixBA * separatingAxis);
			EpxFloat offset = dot(offsetAB,separatingAxis);
			minB += offset;
			maxB += offset;
			
			// 判定
			EPX_CHECK_MINMAX(separatingAxis,minA,maxA,minB,maxB,EpxSatTypeEdgeEdge);
#endif
		}
	}
	
	// ここまで到達したので、２つの凸メッシュは交差している。
	// また、反発ベクトル(axisMin)と貫通深度(distanceMin)が求まった。
	// 反発ベクトルはＡを押しだす方向をプラスにとる。
	
	//int satTotal = convexA.m_numFacets + convexB.m_numFacets + convexA.m_numEdges * convexB.m_numEdges;
	//epxPrintf("sat check count %d / %d\n",satCount,satTotal);
	
	//----------------------------------------------------------------------------
	// 衝突座標検出
	
	int collCount = 0;

	EpxFloat closestMinSqr = EPX_FLT_MAX;
	EpxVector3 closestPointA,closestPointB;
	EpxVector3 separation = 1.1f * fabs(distanceMin) * axisMin;

	for(EpxUInt32 fA=0;fA<convexA.m_numFacets;fA++) {
		const EpxFacet &facetA = convexA.m_facets[fA];

		EpxFloat checkA = dot(facetA.normal,-axisMin);
		if(satType == EpxSatTypePointBFacetA && checkA < 0.99f && axisFlip) {
			// 判定軸が面Aの法線のとき、向きの違うAの面は判定しない
			continue;
		}
			
		if(checkA < 0.0f) {
			// 衝突面と逆に向いている面は判定しない
			continue;
		}
			
		for(EpxUInt32 fB=0;fB<convexB.m_numFacets;fB++) {
			const EpxFacet &facetB = convexB.m_facets[fB];

			EpxFloat checkB = dot(facetB.normal,matrixBA * axisMin);
			if(satType == EpxSatTypePointAFacetB && checkB < 0.99f && !axisFlip) {
				// 判定軸が面Bの法線のとき、向きの違うBの面は判定しない
				continue;
			}
			
			if(checkB < 0.0f) {
				// 衝突面と逆に向いている面は判定しない
				continue;
			}

			collCount++;

			// 面Ａと面Ｂの最近接点を求める
			EpxVector3 triangleA[3] = {
				separation + convexA.m_vertices[facetA.vertId[0]],
				separation + convexA.m_vertices[facetA.vertId[1]],
				separation + convexA.m_vertices[facetA.vertId[2]],
			};
			
			EpxVector3 triangleB[3] = {
				offsetAB + matrixAB * convexB.m_vertices[facetB.vertId[0]],
				offsetAB + matrixAB * convexB.m_vertices[facetB.vertId[1]],
				offsetAB + matrixAB * convexB.m_vertices[facetB.vertId[2]],
			};
			
			// エッジ同士の最近接点算出
			switch(satType) {
				case EpxSatTypeEdgeEdge:
				for(int i=0;i<3;i++) {
					if(convexA.m_edges[facetA.edgeId[i]].type != EpxEdgeTypeConvex) continue;
				
					for(int j=0;j<3;j++) {
						if(convexB.m_edges[facetB.edgeId[j]].type != EpxEdgeTypeConvex) continue;

						EpxVector3 sA,sB;
						epxGetClosestTwoSegments(
							triangleA[i],triangleA[(i+1)%3],
							triangleB[j],triangleB[(j+1)%3],
							sA,sB);
						
						EpxFloat dSqr = lengthSqr(sA-sB);
						if(dSqr < closestMinSqr) {
							closestMinSqr = dSqr;
							closestPointA = sA;
							closestPointB = sB;
						}
					}
				}
				break;
				
				case EpxSatTypePointAFacetB:
				case EpxSatTypePointBFacetA:
				// 頂点Ａ→面Ｂの最近接点算出
				for(int i=0;i<3;i++) {
					EpxVector3 s;
					epxGetClosestPointTriangle(triangleA[i],triangleB[0],triangleB[1],triangleB[2],matrixAB * facetB.normal,s);
					EpxFloat dSqr = lengthSqr(triangleA[i]-s);
					if(dSqr < closestMinSqr) {
						closestMinSqr = dSqr;
						closestPointA = triangleA[i];
						closestPointB = s;
					}
				}
				// 頂点Ｂ→面Ａの最近接点算出
				for(int i=0;i<3;i++) {
					EpxVector3 s;
					epxGetClosestPointTriangle(triangleB[i],triangleA[0],triangleA[1],triangleA[2],facetA.normal,s);
					EpxFloat dSqr = lengthSqr(triangleB[i]-s);
					if(dSqr < closestMinSqr) {
						closestMinSqr = dSqr;
						closestPointA = s;
						closestPointB = triangleB[i];
					}
				}
				break;
			}
		}
	}

	//epxPrintf("intersection check count %d\n",collCount);

	normal = transformA.getUpper3x3() * axisMin;
	penetrationDepth = distanceMin;
	contactPointA = closestPointA - separation;
	contactPointB = offsetBA + matrixBA * closestPointB;

	return true;
}

#endif // EPX_DOXYGEN_SKIP

EpxBool epxConvexConvexContact(
	const EpxConvexMesh &convexA,const EpxTransform3 &transformA,
	const EpxConvexMesh &convexB,const EpxTransform3 &transformB,
	EpxVector3 &normal,
	EpxFloat &penetrationDepth,
	EpxVector3 &contactPointA,
	EpxVector3 &contactPointB)
{
	// 座標系変換の回数を減らすため、面数の多い方を座標系の基準とする

	EpxBool ret;
	if(convexA.m_numFacets >= convexB.m_numFacets) {
		ret = epxConvexConvexContact_local(
			convexA,transformA,
			convexB,transformB,
			normal,penetrationDepth,contactPointA,contactPointB);
	}
	else {
		ret = epxConvexConvexContact_local(
			convexB,transformB,
			convexA,transformA,
			normal,penetrationDepth,contactPointB,contactPointA);
		normal = -normal;
	}

	return ret;
}

} // namespace EasyPhysics
