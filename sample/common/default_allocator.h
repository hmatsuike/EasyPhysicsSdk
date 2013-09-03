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

#ifndef __DEFAULT_ALLOCATOR_H__
#define __DEFAULT_ALLOCATOR_H__

#include "../../easy_physics/EpxInclude.h"

class DefaultAllocator : public EasyPhysics::EpxAllocator
{
public:
	void *allocate(size_t bytes)
	{
		return malloc(bytes);
	}
	
	void deallocate(void *p)
	{
		free(p);
	}
};

class StackAllocator : public EasyPhysics::EpxStackAllocator
{
private:
	enum {
		HEAP_BYTES = 1 * 1024 * 1024,
		STACK_SIZE = 64,
		ALLOC_MIN = 4,
		ALIGNMENT = 4,
	};
	EasyPhysics::EpxUInt8 *m_heapBuff;
	EasyPhysics::EpxUInt32 m_poolStack[STACK_SIZE];
	EasyPhysics::EpxInt32 m_heapBytes;
	EasyPhysics::EpxInt32 m_curStack;

public:
	StackAllocator()
	{
		void *p = malloc(HEAP_BYTES);
		EPX_ALWAYS_ASSERT(p);
		initialize(p,HEAP_BYTES);
	}
	
	~StackAllocator()
	{
		free(m_heapBuff);
	}
	
	void initialize(void *heapBuff,size_t heapBytes)
	{
		m_heapBuff = (EasyPhysics::EpxUInt8*)heapBuff;
		m_heapBytes = heapBytes;
	}

	void clear()
	{
		m_poolStack[0] = 0;
		m_curStack = 0;
	}

	void *allocate(size_t bytes)
	{
		EPX_ALWAYS_ASSERT(m_curStack<STACK_SIZE-1);

		bytes = EPX_MAX(bytes,ALLOC_MIN);

		uintptr_t p = (uintptr_t)(m_heapBuff + m_poolStack[m_curStack]);
		
		uintptr_t tmp = ALIGNMENT - 1;
		
		p = (p+tmp) & ~tmp;
		bytes = (bytes+tmp) & ~tmp;
		
		EPX_ALWAYS_ASSERT((p + bytes) <= (uintptr_t)(m_heapBuff + m_heapBytes)); // Memory overflow
		
		m_poolStack[++m_curStack] = (EasyPhysics::EpxUInt32)((p + bytes) - (uintptr_t)m_heapBuff);
		
		return (void*)p;
	}
	
	void deallocate(void *p)
	{
		(void) p;
		m_curStack--;
	}
};

#endif /* __DEFAULT_ALLOCATOR_H__ */
