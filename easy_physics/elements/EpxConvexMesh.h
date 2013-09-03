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

#ifndef EPX_CONVEX_MESH_H
#define EPX_CONVEX_MESH_H

#include "../EpxBase.h"

#define EPX_CONVEX_MESH_MAX_VERTICES	34
#define EPX_CONVEX_MESH_MAX_EDGES		96
#define EPX_CONVEX_MESH_MAX_FACETS		64

namespace EasyPhysics {

/// エッジの種類
enum EpxEdgeType {
	EpxEdgeTypeConvex,	///< 凸エッジ
	EpxEdgeTypeConcave,	///< 凹エッジ
	EpxEdgeTypeFlat,	///< 平坦エッジ
};

/// エッジ
struct EpxEdge {
	EpxUInt8 type; ///< エッジの種類
	EpxUInt8 vertId[2]; ///< 端点の頂点インデックス
	EpxUInt8 facetId[2]; ///< 共有する面インデックス
};

/// ３角形面
struct EpxFacet {
	EpxUInt8 vertId[3]; ///< 頂点インデックス
	EpxUInt8 edgeId[3]; ///< エッジインデックス
	EpxVector3 normal; ///< 面法線ベクトル
};

/// 凸メッシュ
struct EpxConvexMesh {
	EpxUInt8 m_numVertices; ///< 頂点数
	EpxUInt8 m_numFacets; ///< 面数
	EpxUInt8 m_numEdges; ///< エッジ数
	EpxVector3 m_vertices[EPX_CONVEX_MESH_MAX_VERTICES]; ///< 頂点配列
	EpxEdge m_edges[EPX_CONVEX_MESH_MAX_EDGES]; ///< エッジ配列
	EpxFacet m_facets[EPX_CONVEX_MESH_MAX_FACETS]; ///< 面配列
	
	/// 初期化
	void reset()
	{
		m_numVertices = 0;
		m_numFacets = 0;
		m_numEdges = 0;
	}
};

/// 軸上に凸メッシュを投影し、最小値と最大値を得る
/// @param[out] pmin 投影領域の最小値
/// @param[out] pmax 投影領域の最大値
/// @param convexMesh 凸メッシュ
/// @param axis 投影軸
void epxGetProjection(
	EpxFloat &pmin,EpxFloat &pmax,
	const EpxConvexMesh *convexMesh,
	const EpxVector3 &axis);

/// 凸メッシュを作成する<br>
/// ・入力データが既に凸包になっていること。<br>
/// ・原点は必ずConvexの内側に存在すること。<br>
/// ・３平面から共有されるエッジ、穴あき面は禁止。<br>
/// ・縮退面は自動的に削除される。
/// @param[out] convexMesh 凸メッシュ
/// @param vertices 頂点配列
/// @param numVertices 頂点数
/// @param indices 面インデックス配列
/// @param numIndices 面インデックス数
/// @param scale スケール
/// @return 凸メッシュの作成に成功した場合はtrueを返す。
EpxBool epxCreateConvexMesh(EpxConvexMesh *convexMesh,
	const EpxFloat *vertices,EpxUInt32 numVertices,
	const EpxUInt16 *indices,EpxUInt32 numIndices,
	const EpxVector3 &scale=EpxVector3(1.0f));

} // namespace EasyPhysics

#endif // EPX_CONVEX_MESH_H
