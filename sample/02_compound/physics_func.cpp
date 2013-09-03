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

#include <stdlib.h>
#include "../common/common.h"
#include "../common/geometry_data.h"
#include "../common/default_allocator.h"
#include "physics_func.h"

using namespace EasyPhysics;

///////////////////////////////////////////////////////////////////////////////
// シミュレーション定義

const int maxRigidBodies	= 500;
const int maxJoints			= 100;
const int maxPairs			= 5000;
const float timeStep		= 0.016f;
const float contactBias		= 0.1f;
const float contactSlop		= 0.001f;
const int iteration			= 10;
const EpxVector3 gravity(0.0f,-9.8f,0.0f);

///////////////////////////////////////////////////////////////////////////////
// シミュレーションデータ

// 剛体
EpxState states[maxRigidBodies];
EpxRigidBody bodies[maxRigidBodies];
EpxCollidable collidables[maxRigidBodies];
EpxUInt32 numRigidBodies = 0;

// ジョイント
EpxJoint joints[maxJoints];
EpxUInt32 numJoints = 0;

// ペア
unsigned int pairSwap;
EpxUInt32 numPairs[2];
EpxPair pairs[2][maxPairs];

static int frame = 0;

///////////////////////////////////////////////////////////////////////////////
// メモリアロケータ

DefaultAllocator allocator;

StackAllocator stackAllocator;

///////////////////////////////////////////////////////////////////////////////
// シミュレーション関数

struct Perf {
	unsigned long long count;
	int frame;

	void setFrame(int f)
	{
		frame = f;
	}

	void begin()
	{
		count = perfGetCount();
	}

	void end(const char *msg)
	{
		unsigned long long count2 = perfGetCount();
		float msec = perfGetTimeMillisecond(count,count2);
		if(frame % 100 == 0) {
			epxPrintf("%s : %.2f msec\n",msg,msec);
		}
	}
};

void physicsSimulate()
{
	Perf perf;
	perf.setFrame(frame);

	pairSwap = 1 - pairSwap;
	
	perf.begin();
	for(EpxUInt32 i=0;i<numRigidBodies;i++) {
		EpxVector3 externalForce = gravity * bodies[i].m_mass;
		EpxVector3 externalTorque(0.0f);
		epxApplyExternalForce(states[i],bodies[i],externalForce,externalTorque,timeStep);
	}
	perf.end("apply force");
	
	perf.begin();
	epxBroadPhase(
		states,collidables,numRigidBodies,
		pairs[1-pairSwap],numPairs[1-pairSwap],
		pairs[pairSwap],numPairs[pairSwap],
		maxPairs,timeStep,&allocator,&stackAllocator,NULL,NULL);
	perf.end("broadphase");

	perf.begin();
	epxDetectCollision(
		states,collidables,numRigidBodies,
		pairs[pairSwap],numPairs[pairSwap]);
	perf.end("collision");
	
	perf.begin();
	epxSolveConstraints(
		states,bodies,numRigidBodies,
		pairs[pairSwap],numPairs[pairSwap],
		joints,numJoints,
		iteration,contactBias,contactSlop,timeStep,&stackAllocator);
	perf.end("solver");
	
	perf.begin();
	epxIntegrate(states,numRigidBodies,timeStep);
	perf.end("integrate");

	//epxPrintf("--- frame %d -------------\n",frame);
	//for(int i=0;i<numPairs[pairSwap];i++) {
	//	EpxPair &p = pairs[pairSwap][i];
	//	epxPrintf("RB %u,%u CP %u\n",p.rigidBodyA,p.rigidBodyB,p.contact->m_numContactPoints);
	//}

	frame++;
}

///////////////////////////////////////////////////////////////////////////////
// シーンの作成

static int fireRigidBodyId;

void createFireBody()
{
	fireRigidBodyId = numRigidBodies++;

	EpxVector3 scale(0.5f);		

	states[fireRigidBodyId].reset();
	states[fireRigidBodyId].m_motionType = EpxMotionTypeStatic;
	states[fireRigidBodyId].m_position = EpxVector3(999.0f);
	bodies[fireRigidBodyId].reset();
	bodies[fireRigidBodyId].m_mass = 1.0f;
	bodies[fireRigidBodyId].m_inertia = epxCalcInertiaBox(scale,1.0f);
	collidables[fireRigidBodyId].reset();

	EpxShape shape;
	shape.reset();

	epxCreateConvexMesh(&shape.m_geometry,sphere_vertices,sphere_numVertices,sphere_indices,sphere_numIndices,scale);
	shape.userData = (void*)createRenderMesh(&shape.m_geometry);
	
	EpxCoreSphere sphere;
	sphere.reset();
	sphere.m_radius = 0.25f;
	
	collidables[fireRigidBodyId].addShape(shape);
	collidables[fireRigidBodyId].addCoreSphere(sphere);
	collidables[fireRigidBodyId].finish();
}

void createSceneCompound()
{
	// 地面
	{
		int id = numRigidBodies++;
		
		EpxVector3 scale(10.0f,1.0f,10.0f);
		
		states[id].reset();
		states[id].m_motionType = EpxMotionTypeStatic;
		states[id].m_position = EpxVector3(0.0f,-scale[1],0.0f);
		bodies[id].reset();
		collidables[id].reset();
		
		EpxShape shape;
		shape.reset();
		
		epxCreateConvexMesh(&shape.m_geometry,box_vertices,box_numVertices,box_indices,box_numIndices,scale);
		
		shape.userData = (void*)createRenderMesh(&shape.m_geometry);
		
		collidables[id].addShape(shape);
		collidables[id].finish();
	}
	
	// 複合形状
	for(int i=0;i<5;i++) {
		int id = numRigidBodies++;
		
		states[id].reset();
		states[id].m_position = EpxVector3(0.0f,3.0f+2.0f*i,0.0f);
		states[id].m_orientation = EpxQuat::rotationY(0.5f*i);
		bodies[id].reset();
		bodies[id].m_mass = 1.0f;
		bodies[id].m_inertia = epxCalcInertiaBox(EpxVector3(1.0f,1.0f,0.5f),1.0f);
		collidables[id].reset();
		
		{
			EpxVector3 scale(1.0f,0.125f,0.125f);
			EpxShape shape;
			shape.reset();
			epxCreateConvexMesh(&shape.m_geometry,box_vertices,box_numVertices,box_indices,box_numIndices,scale);
			shape.userData = (void*)createRenderMesh(&shape.m_geometry);
			collidables[id].addShape(shape);
		}

		{
			EpxVector3 scale(0.5f);
			EpxShape shape;
			shape.reset();
			shape.m_offsetPosition = EpxVector3(-1.0f,0.0f,0.0f);
			epxCreateConvexMesh(&shape.m_geometry,sphere_vertices,sphere_numVertices,sphere_indices,sphere_numIndices,scale);
			shape.userData = (void*)createRenderMesh(&shape.m_geometry);
			collidables[id].addShape(shape);
		}

		{
			EpxVector3 scale(1.0f,0.25f,0.25f);
			EpxShape shape;
			shape.reset();
			shape.m_offsetPosition = EpxVector3(1.0f,0.0f,0.0f);
			shape.m_offsetOrientation = EpxQuat::rotationZ(0.5f*EPX_PI);
			epxCreateConvexMesh(&shape.m_geometry,cylinder_vertices,cylinder_numVertices,cylinder_indices,cylinder_numIndices,scale);
			shape.userData = (void*)createRenderMesh(&shape.m_geometry);
			collidables[id].addShape(shape);
		}
		collidables[id].finish();
	}
}

void createSceneDaruma()
{
	// 地面
	{
		int id = numRigidBodies++;
		
		EpxVector3 scale(10.0f,1.0f,10.0f);
		
		states[id].reset();
		states[id].m_motionType = EpxMotionTypeStatic;
		states[id].m_position = EpxVector3(0.0f,-scale[1],0.0f);
		bodies[id].reset();
		collidables[id].reset();
		
		EpxShape shape;
		shape.reset();
		
		epxCreateConvexMesh(&shape.m_geometry,box_vertices,box_numVertices,box_indices,box_numIndices,scale);
		
		shape.userData = (void*)createRenderMesh(&shape.m_geometry);
		
		collidables[id].addShape(shape);
		collidables[id].finish();
	}
	
	// ダルマ複合形状
	for(int i=0;i<=5;i++) {
		int id = numRigidBodies++;
		
		states[id].reset();
		states[id].m_position = EpxVector3(-5.0f+i*2.0f,3.0f,0.0f);
		states[id].m_angularVelocity = EpxVector3(rand()%5,rand()%5,rand()%5);
		bodies[id].reset();
		bodies[id].m_mass = 3.0f;
		bodies[id].m_inertia = epxCalcInertiaSphere(1.5f,3.0f);
		collidables[id].reset();
		
		{
			EpxVector3 scale(1.0f);
			EpxShape shape;
			shape.reset();
			shape.m_offsetPosition = EpxVector3(0.0f,1.0f,0.0f);
			epxCreateConvexMesh(&shape.m_geometry,sphere_vertices,sphere_numVertices,sphere_indices,sphere_numIndices,scale);
			shape.userData = (void*)createRenderMesh(&shape.m_geometry);
			collidables[id].addShape(shape);
		}

		{
			EpxVector3 scale(0.5f);
			EpxShape shape;
			shape.reset();
			shape.m_offsetPosition = EpxVector3(0.0f,2.0f,0.0f);
			epxCreateConvexMesh(&shape.m_geometry,sphere_vertices,sphere_numVertices,sphere_indices,sphere_numIndices,scale);
			shape.userData = (void*)createRenderMesh(&shape.m_geometry);
			collidables[id].addShape(shape);
		}

		collidables[id].finish();
	}
}

void createSceneStackingPole()
{
	// 地面
	{
		int id = numRigidBodies++;

		EpxVector3 scale(10.0f,1.0f,10.0f);		

		states[id].reset();
		states[id].m_motionType = EpxMotionTypeStatic;
		states[id].m_position = EpxVector3(0.0f,-scale[1],0.0f);
		bodies[id].reset();
		collidables[id].reset();
		
		EpxShape shape;
		shape.reset();
		
		epxCreateConvexMesh(&shape.m_geometry,box_vertices,box_numVertices,box_indices,box_numIndices,scale);
		
		shape.userData = (void*)createRenderMesh(&shape.m_geometry);
		
		collidables[id].addShape(shape);
		collidables[id].finish();
	}

	// ボックスを積み上げる
	int renderMeshId = -1;
	const EpxVector3 brickScale(0.5f);

	for(int i=0;i<7;i++) {
		int id = numRigidBodies++;

		states[id].reset();
		states[id].m_position = EpxVector3(0.0f,brickScale[1]+i,0.0f);
		bodies[id].reset();
		bodies[id].m_mass = 1.0f;
		bodies[id].m_inertia = epxCalcInertiaBox(brickScale,1.0f);
		collidables[id].reset();

		EpxShape shape;
		shape.reset();

		epxCreateConvexMesh(&shape.m_geometry,box_vertices,box_numVertices,box_indices,box_numIndices,brickScale);
		shape.userData = (void*)createRenderMesh(&shape.m_geometry);
		
		collidables[id].addShape(shape);
		collidables[id].finish();
	}
}

void createSceneStackingWall()
{
	// 地面
	{
		int id = numRigidBodies++;
		
		EpxVector3 scale(10.0f,1.0f,10.0f);		

		states[id].reset();
		states[id].m_motionType = EpxMotionTypeStatic;
		states[id].m_position = EpxVector3(0.0f,-scale[1],0.0f);
		bodies[id].reset();
		collidables[id].reset();
		
		EpxShape shape;
		shape.reset();
		
		epxCreateConvexMesh(&shape.m_geometry,box_vertices,box_numVertices,box_indices,box_numIndices,scale);
		
		shape.userData = (void*)createRenderMesh(&shape.m_geometry);
		
		collidables[id].addShape(shape);
		collidables[id].finish();
	}

	// ボックスを積み上げる
	const EpxVector3 brickScale(0.5f);
	const EpxVector3 offsetPosition(0.0f,brickScale[1],0.0f);
	const EpxFloat diff = brickScale[1] * 1.1f;
	int stackSize = 10;
	EpxFloat offset = -stackSize * (diff * 2.0f) * 0.5f;
	EpxVector3 pos(0.0f, diff, 0.0f);
	
	int renderMeshId = -1;

	while(stackSize) {
		for(int i=0;i<stackSize;i++) {
			int id = numRigidBodies++;
			
			pos[0] = offset + i * diff * 2.0f;
			
			states[id].reset();
			states[id].m_position = offsetPosition + pos;
			bodies[id].reset();
			bodies[id].m_mass = 1.0f;
			bodies[id].m_inertia = epxCalcInertiaBox(brickScale,1.0f);
			collidables[id].reset();

			EpxShape shape;
			shape.reset();

			epxCreateConvexMesh(&shape.m_geometry,box_vertices,box_numVertices,box_indices,box_numIndices,brickScale);
			shape.userData = (void*)createRenderMesh(&shape.m_geometry);

			collidables[id].addShape(shape);
			collidables[id].finish();
		}
		offset += diff;
		pos[1] += (diff * 2.0f);
		stackSize--;
	}
}

void createSceneTower()
{
	// 地面
	{
		int id = numRigidBodies++;
		
		EpxVector3 scale(10.0f,1.0f,10.0f);		

		states[id].reset();
		states[id].m_motionType = EpxMotionTypeStatic;
		states[id].m_position = EpxVector3(0.0f,-scale[1],0.0f);
		bodies[id].reset();
		collidables[id].reset();
		
		EpxShape shape;
		shape.reset();
		
		epxCreateConvexMesh(&shape.m_geometry,box_vertices,box_numVertices,box_indices,box_numIndices,scale);
		
		shape.userData = (void*)createRenderMesh(&shape.m_geometry);
		
		collidables[id].addShape(shape);
		collidables[id].finish();
	}

	// タワー
	const int stackSize = 10;
	const int rotSize = 24;
	const EpxVector3 boxSize(0.5f);
	const EpxVector3 offsetPosition(0.0f);
	
	EpxFloat radius = 1.3f * rotSize * boxSize[0] / EPX_PI;
	
	EpxQuat rotY = EpxQuat::identity();
	EpxFloat posY = boxSize[1];

	for(int i=0;i<stackSize;i++) {
		for(int j=0;j<rotSize;j++) {
			int id = numRigidBodies++;
			
			EpxVector3 pos = rotate(rotY,EpxVector3(0.0f , posY, radius));
			
			states[id].reset();
			states[id].m_position = offsetPosition + pos;
			states[id].m_orientation = rotY;
			bodies[id].reset();
			bodies[id].m_mass = 1.0f;
			bodies[id].m_inertia = epxCalcInertiaBox(boxSize,1.0f);
			collidables[id].reset();

			EpxShape shape;
			shape.reset();

			epxCreateConvexMesh(&shape.m_geometry,box_vertices,box_numVertices,box_indices,box_numIndices,boxSize);
			shape.userData = (void*)createRenderMesh(&shape.m_geometry);

			collidables[id].addShape(shape);
			collidables[id].finish();
			
			rotY *= EpxQuat::rotationY(EPX_PI/(rotSize*0.5f));
		}

		posY += boxSize[1] * 2.0f;
		rotY *= EpxQuat::rotationY(EPX_PI/(EpxFloat)rotSize);
	}
}

static const int maxScenes = 5;
static const char titles[][32] = {
	"compound shape",
	"daruma",
	"stacking 1",
	"stacking 2",
	"tower",
};

const char *physicsGetSceneTitle(int i)
{
	return titles[i%maxScenes];
}

void physicsCreateScene(int sceneId)
{
	frame = 0;
	
	numRigidBodies = 0;
	numJoints = 0;
	numPairs[0] = numPairs[1] = 0;
	pairSwap = 0;
	
	switch(sceneId%maxScenes) {
		case 0:
		createSceneCompound();
		break;

		case 1:
		createSceneDaruma();
		break;

		case 2:
		createSceneStackingPole();
		break;

		case 3:
		createSceneStackingWall();
		break;
		
		case 4:
		createSceneTower();
		break;
	}

	epxPrintf("numRigidBodies %u\n",numRigidBodies);

	createFireBody();
}

///////////////////////////////////////////////////////////////////////////////
// 初期化、解放

bool physicsInit()
{
	return true;
}

void physicsRelease()
{
}

///////////////////////////////////////////////////////////////////////////////
// 外部から物理データへのアクセス

int physicsGetNumRigidbodies()
{
	return numRigidBodies;
}

const EpxState &physicsGetState(int i)
{
	return states[i];
}

const EpxRigidBody &physicsGetRigidBody(int i)
{
	return bodies[i];
}

const EpxCollidable &physicsGetCollidable(int i)
{
	return collidables[i];
}

int physicsGetNumContacts()
{
	return numPairs[pairSwap];
}

const EasyPhysics::EpxContact &physicsGetContact(int i)
{
	return *pairs[pairSwap][i].contact;
}

EpxUInt32 physicsGetRigidBodyAInContact(int i)
{
	return pairs[pairSwap][i].rigidBodyA;
}

EpxUInt32 physicsGetRigidBodyBInContact(int i)
{
	return pairs[pairSwap][i].rigidBodyB;
}

void physicsFire(const EasyPhysics::EpxVector3 &position,const EasyPhysics::EpxVector3 &velocity)
{
	states[fireRigidBodyId].m_motionType = EpxMotionTypeActive;
	states[fireRigidBodyId].m_position = position;
	states[fireRigidBodyId].m_linearVelocity = velocity;
	states[fireRigidBodyId].m_angularVelocity = EpxVector3(0.0f);
}
