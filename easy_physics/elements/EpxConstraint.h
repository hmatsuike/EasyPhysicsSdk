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

#ifndef EPX_CONSTRAINT_H
#define EPX_CONSTRAINT_H

#include "../EpxBase.h"

namespace EasyPhysics {

/// 拘束
struct EpxConstraint {
	EpxVector3 axis; ///< 拘束軸
	EpxFloat jacDiagInv; ///< 拘束式の分母
	EpxFloat rhs; ///< 初期拘束力
	EpxFloat lowerLimit; ///< 拘束力の下限
	EpxFloat upperLimit; ///< 拘束力の上限
	EpxFloat accumImpulse; ///< 蓄積される拘束力
};

} // namespace EasyPhysics

#endif // EPX_CONSTRAINT_H
