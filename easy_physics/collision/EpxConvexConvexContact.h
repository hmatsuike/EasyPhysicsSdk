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

#ifndef EPX_CONVEX_CONVEX_CONTACT_H
#define EPX_CONVEX_CONVEX_CONTACT_H

#include "../EpxBase.h"
#include "../elements/EpxConvexMesh.h"

namespace EasyPhysics {

/// ２つの凸メッシュの衝突検出
/// @param convexA 凸メッシュA
/// @param transformA Aのワールド変換行列
/// @param convexB 凸メッシュB
/// @param transformB Bのワールド変換行列
/// @param[out] normal 衝突点の法線ベクトル（ワールド座標系）
/// @param[out] penetrationDepth 貫通深度
/// @param[out] contactPointA 衝突点（剛体Aのローカル座標系）
/// @param[out] contactPointB 衝突点（剛体Bのローカル座標系）
/// @return 衝突を検出した場合はtrueを返す。
EpxBool epxConvexConvexContact(
	const EpxConvexMesh &convexA,const EpxTransform3 &transformA,
	const EpxConvexMesh &convexB,const EpxTransform3 &transformB,
	EpxVector3 &normal,
	EpxFloat &penetrationDepth,
	EpxVector3 &contactPointA,
	EpxVector3 &contactPointB);

} // namespace EasyPhysics

#endif // EPX_CONVEX_CONVEX_CONTACT_H
