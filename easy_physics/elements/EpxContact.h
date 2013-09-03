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

#ifndef EPX_CONTACT_H
#define EPX_CONTACT_H

#include "../EpxBase.h"
#include "EpxConstraint.h"

#define EPX_NUM_CONTACT_POINTS 4
#define EPX_NUM_CLOSEST_POINTS 16

namespace EasyPhysics {

/// 衝突点
struct EpxContactPoint {
	EpxFloat distance; ///< 貫通深度
	EpxVector3 pointA; ///< 衝突点（剛体Aのローカル座標系）
	EpxVector3 pointB; ///< 衝突点（剛体Bのローカル座標系）
	EpxVector3 normal; ///< 衝突点の法線ベクトル（ワールド座標系）
	EpxConstraint constraints[3]; ///< 拘束
	
	/// 初期化
	void reset()
	{
		constraints[0].accumImpulse = 0.0f;
		constraints[1].accumImpulse = 0.0f;
		constraints[2].accumImpulse = 0.0f;
	}
};

/// 最近接点
struct EpxClosestPoint {
	EpxFloat distance; ///< 最近接距離
	EpxVector3 pointA; ///< 最近接点（剛体Aのローカル座標系）
	EpxVector3 pointB; ///< 最近接点（剛体Bのローカル座標系）
	EpxVector3 normal; ///< 最近接点の法線ベクトル（ワールド座標系）
	EpxConstraint constraint; ///< 拘束
	
	/// 初期化
	void reset()
	{
		constraint.accumImpulse = 0.0f;
	}
};

/// 衝突情報
struct EpxContact {
	EpxFloat m_friction; ///< 摩擦
	EpxUInt32 m_numContactPoints; ///< 衝突の数
	EpxUInt32 m_numClosestPoints; ///< 最近接点の数
	EpxContactPoint m_contactPoints[EPX_NUM_CONTACT_POINTS]; ///< 衝突点の配列
	EpxClosestPoint m_closestPoints[EPX_NUM_CLOSEST_POINTS]; ///< 最近接点の配列

	/// 同一衝突点を探索する
	/// @param newPointA 衝突点（剛体Aのローカル座標系）
	/// @param newPointB 衝突点（剛体Bのローカル座標系）
	/// @param newNormal 衝突点の法線ベクトル（ワールド座標系）
	/// @return 同じ衝突点を見つけた場合はそのインデックスを返す。
	/// もし見つからなかった場合は-1を返す。
	int findNearestContactPoint(const EpxVector3 &newPointA,const EpxVector3 &newPointB,const EpxVector3 &newNormal);
	
	/// 衝突点を入れ替える
	/// @param newPoint 衝突点（剛体Aのローカル座標系）
	/// @param newDistance 貫通深度
	/// @return 破棄する衝突点のインデックスを返す。
	int sort4ContactPoints(const EpxVector3 &newPoint,EpxFloat newDistance);
	
	/// 衝突点を破棄する
	/// @param i 破棄する衝突点のインデックス
	void removeContactPoint(int i);
	
	/// 初期化
	void reset();
	
	/// 衝突点をリフレッシュする
	/// @param pA 剛体Aの位置
	/// @param qA 剛体Aの姿勢
	/// @param pB 剛体Bの位置
	/// @param qB 剛体Bの姿勢
	void refresh(const EpxVector3 &pA,const EpxQuat &qA,const EpxVector3 &pB,const EpxQuat &qB);
	
	/// 衝突点をマージする
	/// @param contact 合成する衝突情報
	void merge(const EpxContact &contact);
	
	/// 衝突点を追加する
	/// @param penetrationDepth 貫通深度
	/// @param normal 衝突点の法線ベクトル（ワールド座標系）
	/// @param contactPointA 衝突点（剛体Aのローカル座標系）
	/// @param contactPointB 衝突点（剛体Bのローカル座標系）
	void addContactPoint(
		EpxFloat penetrationDepth,
		const EpxVector3 &normal,
		const EpxVector3 &contactPointA,
		const EpxVector3 &contactPointB);

	/// 最近接点を追加する
	/// @param distance 最近接距離
	/// @param normal 最近突点の法線ベクトル（ワールド座標系）
	/// @param closestPointA 最近突点（剛体Aのローカル座標系）
	/// @param closestPointB 最近突点（剛体Bのローカル座標系）
	void addClosestPoint(
		EpxFloat distance,
		const EpxVector3 &normal,
		const EpxVector3 &closestPointA,
		const EpxVector3 &closestPointB);
};

} // namespace EasyPhysics

#endif // EPX_CONTACT_H
