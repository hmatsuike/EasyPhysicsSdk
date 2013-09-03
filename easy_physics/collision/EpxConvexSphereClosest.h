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

#ifndef EPX_CONVEX_SPHERE_CLOSEST_H
#define EPX_CONVEX_SPHERE_CLOSEST_H

#include "../EpxBase.h"
#include "../elements/EpxConvexMesh.h"
#include "../elements/EpxSphere.h"

namespace EasyPhysics {

/// 凸形状と球の最近接点検出
/// @param convexA 凸メッシュA
/// @param transformA Aのワールド変換行列
/// @param sphereB 球B
/// @param transformB Bのワールド変換行列
/// @param[out] normal 最近接点の法線ベクトル（ワールド座標系）
/// @param[out] distance 最近接距離
/// @param[out] closestPointA 最近接点（剛体Aのローカル座標系）
/// @param[out] closestPointB 最近接点（剛体Bのローカル座標系）
/// @return 最近接点を検出した場合はtrueを返す。
EpxBool epxConvexSphereClosest(
	const EpxConvexMesh &convexA,const EpxTransform3 &transformA,
	const EpxSphere &sphereB,const EpxTransform3 &transformB,
	EpxVector3 &normal,
	EpxFloat &distance,
	EpxVector3 &closestPointA,
	EpxVector3 &closestPointB);

} // namespace EasyPhysics

#endif // EPX_CONVEX_SPHERE_CLOSEST_H
