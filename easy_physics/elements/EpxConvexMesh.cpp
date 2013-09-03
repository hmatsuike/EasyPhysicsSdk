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

#include <string.h>
#include "EpxConvexMesh.h"

namespace EasyPhysics {

void epxGetProjection(
	EpxFloat &pmin,EpxFloat &pmax,
	const EpxConvexMesh *convexMesh,
	const EpxVector3 &axis)
{
	EPX_ASSERT(convexMesh);
	
	EpxFloat pmin_ = EPX_FLT_MAX;
	EpxFloat pmax_ = -EPX_FLT_MAX;
	
	for(EpxUInt32 i=0;i<convexMesh->m_numVertices;i++) {
		EpxFloat prj = dot(axis,convexMesh->m_vertices[i]);
		pmin_ = EPX_MIN(pmin_,prj);
		pmax_ = EPX_MAX(pmax_,prj);
	}
	
	pmin = pmin_;
	pmax = pmax_;
}

EpxBool epxCreateConvexMesh(EpxConvexMesh *convexMesh,
	const EpxFloat *vertices,EpxUInt32 numVertices,
	const EpxUInt16 *indices,EpxUInt32 numIndices,
	const EpxVector3 &scale)
{
	EPX_ASSERT(convexMesh);
	EPX_ASSERT(vertices);
	EPX_ASSERT(indices);
	EPX_ASSERT(dot(scale,scale)>0.0f);
	
	if(numVertices > EPX_CONVEX_MESH_MAX_VERTICES || numIndices > EPX_CONVEX_MESH_MAX_FACETS*3) {
		return false;
	}

	memset(convexMesh,0,sizeof(EpxConvexMesh));
	
	// 頂点バッファ作成
	for(EpxUInt32 i=0;i<numVertices;i++) {
		convexMesh->m_vertices[i][0] = vertices[i*3  ];
		convexMesh->m_vertices[i][1] = vertices[i*3+1];
		convexMesh->m_vertices[i][2] = vertices[i*3+2];
		convexMesh->m_vertices[i] = mulPerElem(scale,convexMesh->m_vertices[i]);
	}
	convexMesh->m_numVertices = numVertices;
	
	// 面バッファ作成
	EpxUInt32 nf=0;
	for(EpxUInt32 i=0;i<numIndices/3;i++) {
		EpxVector3 p[3] = {
			convexMesh->m_vertices[indices[i*3  ]],
			convexMesh->m_vertices[indices[i*3+1]],
			convexMesh->m_vertices[indices[i*3+2]]
		};
		
		EpxVector3 normal = cross(p[1]-p[0],p[2]-p[0]);
		EpxFloat areaSqr = lengthSqr(normal); // 面積
		if(areaSqr > EPX_EPSILON * EPX_EPSILON) {// 縮退面は登録しない
			convexMesh->m_facets[nf].vertId[0] = (EpxUInt8)indices[i*3  ];
			convexMesh->m_facets[nf].vertId[1] = (EpxUInt8)indices[i*3+1];
			convexMesh->m_facets[nf].vertId[2] = (EpxUInt8)indices[i*3+2];
			convexMesh->m_facets[nf].normal = normal / sqrtf(areaSqr);
			nf++;
		}
	}
	convexMesh->m_numFacets = nf;
	
	// エッジバッファ作成
	EpxUInt8 edgeIdTable[EPX_CONVEX_MESH_MAX_VERTICES*EPX_CONVEX_MESH_MAX_VERTICES/2];
	memset(edgeIdTable,0xff,sizeof(edgeIdTable));
	
	EpxUInt32 ne=0;
	for(EpxUInt32 i=0;i<convexMesh->m_numFacets;i++) {
		EpxFacet &facet = convexMesh->m_facets[i];
		for(EpxUInt32 e=0;e<3;e++) {
			EpxUInt32 vertId0 = EPX_MIN(facet.vertId[e%3],facet.vertId[(e+1)%3]);
			EpxUInt32 vertId1 = EPX_MAX(facet.vertId[e%3],facet.vertId[(e+1)%3]);
			EpxUInt32 tableId = vertId1*(vertId1-1)/2+vertId0;
			if(edgeIdTable[tableId] == 0xff) {
				// 初回時は登録のみ
				convexMesh->m_edges[ne].facetId[0] = i;
				convexMesh->m_edges[ne].facetId[1] = i;
				convexMesh->m_edges[ne].vertId[0] = (EpxUInt8)vertId0;
				convexMesh->m_edges[ne].vertId[1] = (EpxUInt8)vertId1;
				convexMesh->m_edges[ne].type = EpxEdgeTypeConvex; // 凸エッジで初期化
				facet.edgeId[e] = ne;
				edgeIdTable[tableId] = ne;
				ne++;
				if(ne > EPX_CONVEX_MESH_MAX_EDGES) {
					return false;
				}
			}
			else {
				// 共有面を見つけたので、エッジの角度を判定
				EPX_ASSERT(edgeIdTable[tableId] < EPX_CONVEX_MESH_MAX_EDGES);
				EpxEdge &edge = convexMesh->m_edges[edgeIdTable[tableId]];
				EpxFacet &facetB = convexMesh->m_facets[edge.facetId[0]];

				EPX_ASSERT(edge.facetId[0] == edge.facetId[1]);
				
				// エッジに含まれないＡ面の頂点がB面の表か裏かで判断
				EpxVector3 s = convexMesh->m_vertices[facet.vertId[(e+2)%3]];
				EpxVector3 q = convexMesh->m_vertices[facetB.vertId[0]];
				EpxFloat d = dot(s-q,facetB.normal);
				
				if(d < -EPX_EPSILON) {
					edge.type = EpxEdgeTypeConvex;
				}
				else if(d > EPX_EPSILON) {
					// 本来ここに来てはいけない
					edge.type = EpxEdgeTypeConcave;
				}
				else {
					edge.type = EpxEdgeTypeFlat;
				}
				
				edge.facetId[1] = i;
			}
		}
	}
	convexMesh->m_numEdges = ne;

	return true;
}

} // namespace EasyPhysics
