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

#ifndef __RENDER_FUNC_H__
#define __RENDER_FUNC_H__

#include "common.h"

#define	DISPLAY_WIDTH			640
#define	DISPLAY_HEIGHT			480

///////////////////////////////////////////////////////////////////////////////
// Draw Primitives

void renderInit(const char *title=NULL);
void renderRelease();
void renderBegin();
void renderEnd();

int renderInitMesh(
	const float *vtx,unsigned int vtxStrideBytes,
	const float *nml,unsigned int nmlStrideBytes,
	const unsigned short *tri,unsigned int triStrideBytes,
	int numVtx,int numTri);

void renderReleaseMeshAll();

void renderMesh(
	const EasyPhysics::EpxTransform3 &transform,
	const EasyPhysics::EpxVector3 &color,
	int meshId);

///////////////////////////////////////////////////////////////////////////////
// Debug Drawing

void renderDebugBegin();
void renderDebugEnd();

void renderDebugPoint(
	const EasyPhysics::EpxVector3 &position,
	const EasyPhysics::EpxVector3 &color);

void renderDebugLine(
	const EasyPhysics::EpxVector3 &position1,
	const EasyPhysics::EpxVector3 &position2,
	const EasyPhysics::EpxVector3 &color);

void renderDebugAabb(
	const EasyPhysics::EpxVector3 &center,
	const EasyPhysics::EpxVector3 &extent,
	const EasyPhysics::EpxVector3 &color);

void renderDebugSphere(
	const EasyPhysics::EpxVector3 &position,
	const EasyPhysics::EpxFloat radius,
	const EasyPhysics::EpxVector3 &color);

///////////////////////////////////////////////////////////////////////////////
// 2D 

void renderDebug2dBegin();
void renderDebug2dEnd();

///////////////////////////////////////////////////////////////////////////////
// Render Parameter

void renderGetViewAngle(float &angleX,float &angleY,float &radius);
void renderSetViewAngle(float angleX,float angleY,float radius);
void renderResize(int width,int height);

void renderGetViewTarget(EasyPhysics::EpxVector3 &targetPos);
void renderSetViewTarget(const EasyPhysics::EpxVector3 &targetPos);
void renderGetViewRadius(float &radius);
void renderSetViewRadius(float radius);
void renderLookAtTarget(const EasyPhysics::EpxVector3 &viewPos,const EasyPhysics::EpxVector3 &viewTarget);

void renderGetScreenSize(int &width,int &height);

EasyPhysics::EpxVector3 renderGetWorldPosition(const EasyPhysics::EpxVector3 &screenPos);
EasyPhysics::EpxVector3 renderGetScreenPosition(const EasyPhysics::EpxVector3 &worldPos);

void renderWait();

#endif /* __RENDER_FUNC_H__ */
