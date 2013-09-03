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

#ifndef EPX_ALLOCATOR_H
#define EPX_ALLOCATOR_H

#include "../EpxBase.h"

namespace EasyPhysics {

/// アロケータ
class EpxAllocator {
public:
	/// メモリ確保時に呼ばれるメソッド。
	/// @param bytes 確保するメモリのバイト数
	/// @return 確保したメモリのポインタ
	/// @return メモリの確保に失敗した場合はNULLを返す。
	virtual void *allocate(size_t bytes) = 0;

	/// メモリ解放時に呼ばれるメソッド。
	/// @param p 解放するメモリのポインタ
	virtual void deallocate(void *p) = 0;
};

/// スタックアロケータ
class EpxStackAllocator {
public:
	/// 初期化メソッド。指定されたバッファをヒープとして扱います。
	/// @param heapBuff ヒープメモリのポインタ
	/// @param heapBytes ヒープメモリのバイト数
	virtual void initialize(void *heapBuff,size_t heapBytes) = 0;

	/// クリアメソッド。スタックを初期状態に戻します。
	virtual void clear() = 0;

	/// メモリ確保時に呼ばれるメソッド。
	/// @param bytes 確保するメモリのバイト数
	/// @return 確保したメモリのポインタ
	/// @return メモリの確保に失敗した場合はNULLを返す。
	virtual void *allocate(size_t bytes) = 0;

	/// メモリ解放時に呼ばれるメソッド。
	/// @param p 解放するメモリのポインタ
	virtual void deallocate(void *p) = 0;
};

} // namespace EasyPhysics

#endif // EPX_ALLOCATOR_H
