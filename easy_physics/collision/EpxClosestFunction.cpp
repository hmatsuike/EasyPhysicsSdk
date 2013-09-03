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

#include "EpxClosestFunction.h"

namespace EasyPhysics {

void epxGetClosestTwoSegments(
	const EpxVector3 &segmentPointA0,const EpxVector3 &segmentPointA1,
	const EpxVector3 &segmentPointB0,const EpxVector3 &segmentPointB1,
	EpxVector3 &closestPointA,EpxVector3 &closestPointB)
{
	EpxVector3 v1 = segmentPointA1 - segmentPointA0;
	EpxVector3 v2 = segmentPointB1 - segmentPointB0;
	EpxVector3 r  = segmentPointA0 - segmentPointB0;

	EpxFloat a = dot(v1,v1);
	EpxFloat b = dot(v1,v2);
	EpxFloat c = dot(v2,v2);
	EpxFloat d = dot(v1,r);
	EpxFloat e = dot(v2,r);
	EpxFloat det = -a*c+b*b;
	EpxFloat s=0.0f,t=0.0f;

	// 逆行列の存在をチェック
	if(det*det > EPX_EPSILON) {
		s = (c*d-b*e)/det;
	}

	// 線分A上の最近接点を決めるパラメータsを0.0～1.0でクランプ
	s = EPX_CLAMP(s,0.0f,1.0f);
	
	// 線分Bのtを求める
	//t = dot((segmentPointA0+s*v1) - segmentPointB0,v2) / dot(v2,v2);
	t = (e+s*b)/c;

	// 線分B上の最近接点を決めるパラメータtを0.0～1.0でクランプ
	t = EPX_CLAMP(t,0.0f,1.0f);

	// 再度、線分A上の点を求める
	//s = dot((segmentPointB0+t*v2) - segmentPointA0,v1) / dot(v1,v1);
	s = (-d+t*b)/a;
	s = EPX_CLAMP(s,0.0f,1.0f);

	closestPointA = segmentPointA0 + s * v1;
	closestPointB = segmentPointB0 + t * v2;
}

#ifndef EPX_DOXYGEN_SKIP

inline
void epxGetClosestPointLine(
	const EpxVector3 &linePoint,
	const EpxVector3 &lineDirection,
	const EpxVector3 &point,
	EpxVector3 &closestPoint)
{
	EpxFloat s = dot(point - linePoint,lineDirection) / dot(lineDirection,lineDirection);
	closestPoint = linePoint + s * lineDirection;
}

#endif // EPX_DOXYGEN_SKIP

void epxGetClosestPointTriangle(
	const EpxVector3 &point,
	const EpxVector3 &trianglePoint0,
	const EpxVector3 &trianglePoint1,
	const EpxVector3 &trianglePoint2,
	const EpxVector3 &triangleNormal,
	EpxVector3 &closestPoint)
{
	// ３角形面上の投影点
	EpxVector3 proj = point - dot(triangleNormal,point - trianglePoint0) * triangleNormal;
	
	// エッジP0,P1のボロノイ領域
	EpxVector3 edgeP01 = trianglePoint1 - trianglePoint0;
	EpxVector3 edgeP01_normal = cross(edgeP01,triangleNormal);
	
	EpxFloat voronoiEdgeP01_check1 = dot(proj - trianglePoint0,edgeP01_normal);
	EpxFloat voronoiEdgeP01_check2 = dot(proj - trianglePoint0,edgeP01);
	EpxFloat voronoiEdgeP01_check3 = dot(proj - trianglePoint1,-edgeP01);
	
	if(voronoiEdgeP01_check1 > 0.0f && voronoiEdgeP01_check2 > 0.0f && voronoiEdgeP01_check3 > 0.0f) {
		epxGetClosestPointLine(trianglePoint0,edgeP01,proj,closestPoint);
		return;
	}
	
	// エッジP1,P2のボロノイ領域
	EpxVector3 edgeP12 = trianglePoint2 - trianglePoint1;
	EpxVector3 edgeP12_normal = cross(edgeP12,triangleNormal);
	
	EpxFloat voronoiEdgeP12_check1 = dot(proj - trianglePoint1,edgeP12_normal);
	EpxFloat voronoiEdgeP12_check2 = dot(proj - trianglePoint1,edgeP12);
	EpxFloat voronoiEdgeP12_check3 = dot(proj - trianglePoint2,-edgeP12);
	
	if(voronoiEdgeP12_check1 > 0.0f && voronoiEdgeP12_check2 > 0.0f && voronoiEdgeP12_check3 > 0.0f) {
		epxGetClosestPointLine(trianglePoint1,edgeP12,proj,closestPoint);
		return;
	}
	
	// エッジP2,P0のボロノイ領域
	EpxVector3 edgeP20 = trianglePoint0 - trianglePoint2;
	EpxVector3 edgeP20_normal = cross(edgeP20,triangleNormal);
	
	EpxFloat voronoiEdgeP20_check1 = dot(proj - trianglePoint2,edgeP20_normal);
	EpxFloat voronoiEdgeP20_check2 = dot(proj - trianglePoint2,edgeP20);
	EpxFloat voronoiEdgeP20_check3 = dot(proj - trianglePoint0,-edgeP20);
	
	if(voronoiEdgeP20_check1 > 0.0f && voronoiEdgeP20_check2 > 0.0f && voronoiEdgeP20_check3 > 0.0f) {
		epxGetClosestPointLine(trianglePoint2,edgeP20,proj,closestPoint);
		return;
	}
	
	// ３角形面の内側
	if(voronoiEdgeP01_check1 <= 0.0f && voronoiEdgeP12_check1 <= 0.0f && voronoiEdgeP20_check1 <= 0.0f) {
		closestPoint = proj;
		return;
	}
	
	// 頂点P0のボロノイ領域
	if(voronoiEdgeP01_check2 <= 0.0f && voronoiEdgeP20_check3 <= 0.0f) {
		closestPoint = trianglePoint0;
		return;
	}
	
	// 頂点P1のボロノイ領域
	if(voronoiEdgeP01_check3 <= 0.0f && voronoiEdgeP12_check2 <= 0.0f) {
		closestPoint = trianglePoint1;
		return;
	}
	
	// 頂点P2のボロノイ領域
	if(voronoiEdgeP20_check2 <= 0.0f && voronoiEdgeP12_check3 <= 0.0f) {
		closestPoint = trianglePoint2;
		return;
	}
}

} // namespace EasyPhysics