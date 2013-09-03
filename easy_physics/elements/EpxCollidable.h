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

#ifndef EPX_COLLIDABLE_H
#define EPX_COLLIDABLE_H

#include "../EpxBase.h"
#include "EpxShape.h"
#include "EpxCoreSphere.h"

#define EPX_NUM_SHAPES 5

namespace EasyPhysics {

/// 形状コンテナ
struct EpxCollidable {
	EpxUInt8 m_numShapes; ///< 保持する形状数
	EpxUInt8 m_numCoreSpheres; ///< 保持するCore Sphere数
	EpxShape m_shapes[EPX_NUM_SHAPES]; ///< 形状の配列
	EpxCoreSphere m_coreSpheres[EPX_NUM_SHAPES]; ///< Core Sphere
	EpxVector3 m_center; ///< AABBの中心
	EpxVector3 m_half; ///< AABBのサイズの半分
	
	/// 初期化
	void reset()
	{
		m_numShapes = 0;
		m_numCoreSpheres = 0;
		m_center = EpxVector3(0.0f);
		m_half = EpxVector3(0.0f);
	}
	
	/// 形状を登録する。
	/// 空きがなければ無視される
	/// @param shape 形状
	void addShape(const EpxShape &shape)
	{
		if(m_numShapes < EPX_NUM_SHAPES) {
			m_shapes[m_numShapes++] = shape;
		}
	}

	/// Core Sphereを登録する。
	/// 空きがなければ無視される
	/// @param sphere Core Sphere
	void addCoreSphere(const EpxCoreSphere &sphere)
	{
		if(m_numCoreSpheres < EPX_NUM_SHAPES) {
			m_coreSpheres[m_numCoreSpheres++] = sphere;
		}
	}

	/// 形状登録の完了を通知する。
	/// 全ての形状を登録した後に呼び出し、全体を囲むAABBを作成する。
	void finish()
	{
		// AABBを計算する
		EpxVector3 aabbMax(-EPX_FLT_MAX),aabbMin(EPX_FLT_MAX);
		for(EpxUInt32 i=0;i<m_numShapes;i++) {
			const EpxConvexMesh &mesh = m_shapes[i].m_geometry;
		
			for(EpxUInt32 v=0;v<mesh.m_numVertices;v++) {
				aabbMax = maxPerElem(aabbMax,m_shapes[i].m_offsetPosition + rotate(m_shapes[i].m_offsetOrientation,mesh.m_vertices[v]));
				aabbMin = minPerElem(aabbMin,m_shapes[i].m_offsetPosition + rotate(m_shapes[i].m_offsetOrientation,mesh.m_vertices[v]));
			}
		}

		for(EpxUInt32 i=0;i<m_numCoreSpheres;i++) {
			const EpxCoreSphere &sphere = m_coreSpheres[i];

			aabbMax = maxPerElem(aabbMax,sphere.m_offsetPosition + EpxVector3(sphere.m_radius));
			aabbMin = minPerElem(aabbMin,sphere.m_offsetPosition - EpxVector3(sphere.m_radius));
		}

		m_center = (aabbMax + aabbMin) * 0.5f;
		m_half = (aabbMax - aabbMin) * 0.5f;
	}
};

} // namespace EasyPhysics

#endif // EPX_COLLIDABLE_H
