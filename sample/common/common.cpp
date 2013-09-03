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

#include "common.h"
#include "render_func.h"

using namespace EasyPhysics;

int createRenderMesh(EpxConvexMesh *convexMesh)
{
	EPX_ASSERT(convexMesh);
	
	EpxFloat* verts = new EpxFloat[convexMesh->m_numVertices * 3];
	EpxFloat* nmls = new EpxFloat[convexMesh->m_numVertices * 3];
	EpxUInt16* idxs	= new EpxUInt16[convexMesh->m_numFacets * 3];

	for(EpxUInt32 c=0; c<convexMesh->m_numVertices;c++) {
		verts[c*3+0] = convexMesh->m_vertices[c][0];
		verts[c*3+1] = convexMesh->m_vertices[c][1];
		verts[c*3+2] = convexMesh->m_vertices[c][2];
	}

	for (EpxUInt32 c=0; c<convexMesh->m_numVertices;c++) {
		EpxVector3 normal(0.0f);
		int facetCount = 0;
		for(EpxUInt32 f=0;f<convexMesh->m_numFacets;f++) {
			EpxFacet &facet = convexMesh->m_facets[f];
			if(facet.vertId[0] == c || facet.vertId[1] == c || facet.vertId[2] == c) {
				const EpxVector3& v0 = convexMesh->m_vertices[facet.vertId[0]];
				const EpxVector3& v1 = convexMesh->m_vertices[facet.vertId[1]];
				const EpxVector3& v2 = convexMesh->m_vertices[facet.vertId[2]];
				normal += cross(v1-v0,v2-v0);
				facetCount++;
			}
		}
		normal = normalize(normal/(EpxFloat)facetCount);
		
		nmls[c*3+0] = normal[0];
		nmls[c*3+1] = normal[1];
		nmls[c*3+2] = normal[2];
	}

	for (EpxUInt32 c=0; c<convexMesh->m_numFacets;c++) {
		idxs[c*3+0] = convexMesh->m_facets[c].vertId[0];
		idxs[c*3+1] = convexMesh->m_facets[c].vertId[1];
		idxs[c*3+2] = convexMesh->m_facets[c].vertId[2];
	}

	int renderMeshId = renderInitMesh(
		verts,	sizeof(EpxFloat)*3,
		nmls,	sizeof(EpxFloat)*3,
		idxs,	sizeof(EpxUInt16)*3,
		convexMesh->m_numVertices, convexMesh->m_numFacets);
	
	delete [] idxs;
	delete [] nmls;
	delete [] verts;
	
	return renderMeshId;
}

unsigned long long perfGetCount()
{
	LARGE_INTEGER cnt;
	QueryPerformanceCounter(&cnt);
	return cnt.QuadPart;
}

float perfGetTimeMillisecond(unsigned long long time1,unsigned long long time2)
{
	LARGE_INTEGER freq;
	QueryPerformanceFrequency(&freq);
	return (time2 - time1) / (float)freq.QuadPart * 1000.0f;
}
