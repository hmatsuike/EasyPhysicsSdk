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

#ifndef EPX_STATE_H
#define EPX_STATE_H

#include "../EpxBase.h"

#define EPX_MAX_LINEAR_VELOCITY		340.0f
#define EPX_MAX_ANGULAR_VELOCITY	(EPX_PI * 30.0f)

namespace EasyPhysics {

/// モーションタイプ
enum EpxMotionType {
	EpxMotionTypeActive, ///< アクティブ
	EpxMotionTypeStatic ///< 固定
};

/// 剛体の状態
struct EpxState {
	EpxVector3	m_position; ///< 位置
	EpxQuat		m_orientation; ///< 姿勢
	EpxVector3	m_linearVelocity; ///< 並進速度
	EpxVector3	m_angularVelocity; ///< 回転速度
	EpxMotionType m_motionType; ///< ふるまい
	
	/// 初期化
	void reset()
	{
		m_position = EpxVector3(0.0f);
		m_orientation = EpxQuat::identity();
		m_linearVelocity = EpxVector3(0.0f);
		m_angularVelocity = EpxVector3(0.0f);
		m_motionType = EpxMotionTypeActive;
	}
};

} // namespace EasyPhysics

#endif // EPX_STATE_H
