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

#ifndef EPX_JOINT_H
#define EPX_JOINT_H

#include "../EpxBase.h"
#include "EpxConstraint.h"
#include "EpxState.h"

namespace EasyPhysics {

/// 拘束ロックの種類
enum EpxLockType {
	EpxLockTypeFree,	///< 自由
	EpxLockTypeLimit,	///< 制限
	EpxLockTypeFix,		///< 固定
};

/// ジョイント拘束
struct EpxJointConstraint {
	EpxLockType lockType; ///< 拘束のロック種類
	EpxFloat bias; ///< 拘束の強さの調整値
	EpxFloat damping; ///< 拘束のダンピング
	EpxFloat lowerLimit; ///< 可動範囲の下限
	EpxFloat upperLimit; ///< 可動範囲の上限
	EpxConstraint constraint; ///< 拘束
	
	/// 初期化
	void reset()
	{
		lockType = EpxLockTypeFix;
		bias = 0.1f;
		damping = 0.0f;
		lowerLimit = 0.0f;
		upperLimit = 0.0f;
		constraint.accumImpulse = 0.0f;
	}
};

/// ジョイント
struct EpxJoint {
	EpxUInt16 rigidBodyIdA; ///< 剛体Aへのインデックス
	EpxUInt16 rigidBodyIdB; ///< 剛体Bへのインデックス
	EpxUInt32 numConstraints; ///< 拘束の数（<=6）
	EpxVector3 anchorA; ///< 剛体Aのローカル座標系における接続点
	EpxVector3 anchorB; ///< 剛体Bのローカル座標系における接続点
	EpxMatrix3 frameA; ///< 剛体Aのローカル座標系におけるジョイント座標系
	EpxMatrix3 frameB; ///< 剛体Bのローカル座標系におけるジョイント座標系
	EpxJointConstraint jointConstraints[6]; ///< ジョイント拘束

	void (*calcAngleFunc)(
		const EpxMatrix3 &worldFrameA,
		const EpxMatrix3 &worldFrameB,
		EpxVector3 &angle3,
		EpxMatrix3 &axis3);

	/// 初期化
	void reset()
	{
		jointConstraints[0].reset();
		jointConstraints[1].reset();
		jointConstraints[2].reset();
		jointConstraints[3].reset();
		jointConstraints[4].reset();
		jointConstraints[5].reset();
		calcAngleFunc = NULL;
	}
};

/// ボールジョイントとして初期化する
/// @param[out] joint ジョイント
/// @param rigidBodyIdA 剛体Aのインデックス
/// @param rigidBodyIdB 剛体Bのインデックス
/// @param stateA 剛体Aの状態
/// @param stateB 剛体Bの状態
/// @param anchor 接続点（ワールド座標系）
/// @param bias バイアス
/// @param damping ダンピング
void epxInitializeJointAsBall(
	EpxJoint &joint,
	EpxUInt32 rigidBodyIdA,
	EpxUInt32 rigidBodyIdB,
	const EpxState &stateA,
	const EpxState &stateB,
	const EpxVector3 &anchor,
	EpxFloat bias,
	EpxFloat damping);

/// ヒンジジョイントとして初期化する
/// @param[out] joint ジョイント
/// @param rigidBodyIdA 剛体Aのインデックス
/// @param rigidBodyIdB 剛体Bのインデックス
/// @param stateA 剛体Aの状態
/// @param stateB 剛体Bの状態
/// @param anchor 接続点（ワールド座標系）
/// @param direction 回転軸（ワールド座標系）
/// @param bias バイアス
/// @param damping ダンピング
/// @param lowerAngle 回転角の下限
/// @param upperAngle 回転角の上限
void epxInitializeJointAsHinge(
	EpxJoint &joint,
	EpxUInt32 rigidBodyIdA,
	EpxUInt32 rigidBodyIdB,
	const EpxState &stateA,
	const EpxState &stateB,
	const EpxVector3 &anchor,
	const EpxVector3 &direction,
	EpxFloat bias,
	EpxFloat damping,
	EpxFloat lowerAngle,
	EpxFloat upperAngle);

/// 固定ジョイントとして初期化する
/// @param[out] joint ジョイント
/// @param rigidBodyIdA 剛体Aのインデックス
/// @param rigidBodyIdB 剛体Bのインデックス
/// @param stateA 剛体Aの状態
/// @param stateB 剛体Bの状態
/// @param anchor 接続点（ワールド座標系）
/// @param bias バイアス
void epxInitializeJointAsFix(
	EpxJoint &joint,
	EpxUInt32 rigidBodyIdA,
	EpxUInt32 rigidBodyIdB,
	const EpxState &stateA,
	const EpxState &stateB,
	const EpxVector3 &anchor,
	EpxFloat bias);

/// スイングツイストジョイントとして初期化する
/// @param[out] joint ジョイント
/// @param rigidBodyIdA 剛体Aのインデックス
/// @param rigidBodyIdB 剛体Bのインデックス
/// @param stateA 剛体Aの状態
/// @param stateB 剛体Bの状態
/// @param anchor 接続点（ワールド座標系）
/// @param direction ツイスト軸（ワールド座標系）
/// @param bias バイアス
/// @param damping ダンピング
/// @param twistLowerAngle ツイスト回転角の下限
/// @param twistUpperAngle ツイスト回転角の上限
/// @param swingLowerAngle スイング回転角の下限
/// @param swingUpperAngle スイング回転角の上限
void epxInitializeJointAsSwingTwist(
	EpxJoint &joint,
	EpxUInt32 rigidBodyIdA,
	EpxUInt32 rigidBodyIdB,
	const EpxState &stateA,
	const EpxState &stateB,
	const EpxVector3 &anchor,
	const EpxVector3 &direction,
	EpxFloat bias,
	EpxFloat damping,
	EpxFloat twistLowerAngle,
	EpxFloat twistUpperAngle,
	EpxFloat swingLowerAngle,
	EpxFloat swingUpperAngle);

/// スライダジョイントとして初期化する
/// @param[out] joint ジョイント
/// @param rigidBodyIdA 剛体Aのインデックス
/// @param rigidBodyIdB 剛体Bのインデックス
/// @param stateA 剛体Aの状態
/// @param stateB 剛体Bの状態
/// @param anchor 接続点（ワールド座標系）
/// @param direction スライド軸（ワールド座標系）
/// @param bias バイアス
/// @param damping ダンピング
/// @param lowerDistance スライド可動距離の下限
/// @param upperDistance スライド可動距離の上限
void epxInitializeJointAsSlider(
	EpxJoint &joint,
	EpxUInt32 rigidBodyIdA,
	EpxUInt32 rigidBodyIdB,
	const EpxState &stateA,
	const EpxState &stateB,
	const EpxVector3 &anchor,
	const EpxVector3 &direction,
	EpxFloat bias,
	EpxFloat damping,
	EpxFloat lowerDistance,
	EpxFloat upperDistance);

} // namespace EasyPhysics

#endif // EPX_BALL_JOINT_H
