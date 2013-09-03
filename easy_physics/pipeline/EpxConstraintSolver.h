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

#ifndef EPX_CONSTRAINT_SOLVER_H
#define EPX_CONSTRAINT_SOLVER_H

#include "../EpxBase.h"
#include "../elements/EpxState.h"
#include "../elements/EpxRigidBody.h"
#include "../elements/EpxPair.h"
#include "../elements/EpxJoint.h"
#include "EpxAllocator.h"

namespace EasyPhysics {

/// 拘束ソルバー
/// @param states 剛体の状態の配列
/// @param bodies 剛体の属性の配列
/// @param numRigidBodies 剛体の数
/// @param pairs ペア配列
/// @param numPairs ペア数
/// @param joints ジョイント配列
/// @param numJoints ジョイントの数
/// @param iteration 計算の反復回数
/// @param bias 位置補正のバイアス
/// @param slop 貫通許容誤差
/// @param timeStep タイムステップ
/// @param stackAllocator スタックアロケータ
void epxSolveConstraints(
	EpxState *states,
	const EpxRigidBody *bodies,
	EpxUInt32 numRigidBodies,
	const EpxPair *pairs,
	EpxUInt32 numPairs,
	EpxJoint *joints,
	EpxUInt32 numJoints,
	EpxUInt32 iteration,
	EpxFloat bias,
	EpxFloat slop,
	EpxFloat timeStep,
	EpxStackAllocator *stackAllocator
	);

} // namespace EasyPhysics

#endif // EPX_CONSTRAINT_SOLVER_H
