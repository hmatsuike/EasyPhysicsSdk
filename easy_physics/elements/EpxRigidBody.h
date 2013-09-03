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

#ifndef EPX_RIGIDBODY_H
#define EPX_RIGIDBODY_H

#include "../EpxBase.h"

namespace EasyPhysics {

/// 剛体の属性
struct EpxRigidBody {
	EpxMatrix3     m_inertia; ///< 慣性テンソル
	EpxFloat       m_mass; ///< 質量
	EpxFloat       m_restitution; ///< 反発係数
	EpxFloat       m_friction; ///< 摩擦係数
	
	/// 初期化
	void reset()
	{
		m_mass = 1.0f;
		m_inertia = EpxMatrix3::identity();
		m_restitution = 0.2f;
		m_friction = 0.6f;
	}
};

} // namespace EasyPhysics

#endif // EPX_RIGIDBODY_H
