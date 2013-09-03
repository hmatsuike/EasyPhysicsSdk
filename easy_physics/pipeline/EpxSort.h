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

#ifndef EPX_SORT_H
#define EPX_SORT_H

#include "../EpxBase.h"

#define Key(a) ((a).key)

namespace EasyPhysics {

#if 1 // Radix sort

/// ソート
/// @param[in,out] d ソートするデータの配列
/// @param buff ソート用のバッファ（入力データと同サイズ）
/// @param n データの数
template <class SortData>
void epxSort(SortData *data,SortData *buff,int n)
{
	const EpxUInt32 bits = 8;
	const EpxUInt32 nSize = 256;
	const EpxUInt32 nLoop = 4;
	EpxInt32 *count = (EpxInt32*)epxAlloca(sizeof(EpxInt32) * nSize); // 1KB

	SortData *sbuff[2] = {data,buff};

	int sw = 0;

	for(EpxUInt32 pass=0;pass<nLoop;pass++) {
		for(EpxUInt32 j=0;j<nSize;j++) {
			count[j] = 0;
		}
		for(int i=0;i<n;i++) {
			count[((Key(sbuff[sw][i])>>(pass*bits))&0xff)]++;
		}

		EpxInt32 sum = 0;
		for(EpxUInt32 j=0;j<nSize;j++) {
			EpxInt32 temp = count[j] + sum;
			count[j] = sum;
			sum = temp;
		}

		for(int i=0;i<n;i++) {
			EpxInt32 id = (Key(sbuff[sw][i])>>(pass*bits))&0xff;
			EpxInt32 dst = count[id]++;
			sbuff[1-sw][dst] = sbuff[sw][i];
		}
		sw = 1 - sw;
	}
}

#else // Merge sort
#ifndef EPX_DOXYGEN_SKIP

template <class SortData>
void epxMergeTwoBuffers(SortData* d1,unsigned int n1,SortData* d2,unsigned int n2,SortData *buff)
{
	unsigned int i=0,j=0;

	while(i<n1&&j<n2) {
		if(Key(d1[i]) < Key(d2[j])) {
			buff[i+j] = d1[i++];
		}
		else {
			buff[i+j] = d2[j++];
		}
	}
	
	if(i<n1) {
		while(i<n1) {
			buff[i+j] = d1[i++];
		}
	}
	else if(j<n2) {
		while(j<n2) {
			buff[i+j] = d2[j++];
		}
	}

	for(unsigned int k=0;k<(n1+n2);k++) {
		d1[k] = buff[k];
	}
}

#endif // EPX_DOXYGEN_SKIP

/// ソート
/// @param[in,out] d ソートするデータの配列
/// @param buff ソート用のバッファ（入力データと同サイズ）
/// @param n データの数
template <class SortData>
void epxSort(SortData *d,SortData *buff,int n)
{
	int n1 = n>>1;
	int n2 = n-n1;
	if(n1>1) epxSort(d,buff,n1);
	if(n2>1) epxSort(d+n1,buff,n2);
	epxMergeTwoBuffers(d,n1,d+n1,n2,buff);
}
#endif

} // namespace EasyPhysics

#endif // EPX_SORT_H
