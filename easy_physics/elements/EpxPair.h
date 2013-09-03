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

#ifndef EPX_PAIR_H
#define EPX_PAIR_H

#include "../EpxBase.h"
#include "EpxContact.h"

namespace EasyPhysics {

/// ペアの種類
enum EpxPairType {
	EpxPairTypeNew, ///< 新規
	EpxPairTypeKeep, ///< 維持
};

/// ペア
struct EpxPair {
	EpxPairType type; ///< 種類
	union {
		EpxUInt32 key;///< ユニークなキー
		struct {
			EpxUInt16 rigidBodyA; ///< 剛体Aのインデックス
			EpxUInt16 rigidBodyB; ///< 剛体Bのインデックス
		};
	};
	EpxContact *contact; ///< 衝突情報
};

} // namespace EasyPhysics

#endif // EPX_PAIR_H
