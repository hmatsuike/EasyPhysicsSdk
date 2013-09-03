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

#ifndef EPX_CLOSEST_FUNCTION_H
#define EPX_CLOSEST_FUNCTION_H

#include "../EpxBase.h"

namespace EasyPhysics {

/// ２つの線分の最近接点検出
/// @param segmentPointA0 線分Aの始点
/// @param segmentPointA1 線分Aの終点
/// @param segmentPointB0 線分Bの始点
/// @param segmentPointB1 線分Bの終点
/// @param[out] closestPointA 線分A上の最近接点
/// @param[out] closestPointB 線分B上の最近接点
void epxGetClosestTwoSegments(
	const EpxVector3 &segmentPointA0,const EpxVector3 &segmentPointA1,
	const EpxVector3 &segmentPointB0,const EpxVector3 &segmentPointB1,
	EpxVector3 &closestPointA,EpxVector3 &closestPointB);

/// 頂点から３角形面への最近接点検出
/// @param point 頂点
/// @param trianglePoint0 ３角形面の頂点0
/// @param trianglePoint1 ３角形面の頂点1
/// @param trianglePoint2 ３角形面の頂点2
/// @param triangleNormal ３角形面の法線ベクトル
/// @param[out] closestPoint ３角形面上の最近接点
void epxGetClosestPointTriangle(
	const EpxVector3 &point,
	const EpxVector3 &trianglePoint0,
	const EpxVector3 &trianglePoint1,
	const EpxVector3 &trianglePoint2,
	const EpxVector3 &triangleNormal,
	EpxVector3 &closestPoint);

} // namespace EasyPhysics

#endif // EPX_CLOSEST_FUNCTION_H
