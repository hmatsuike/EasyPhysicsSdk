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
const int iteration			= 5;
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
// ブロードフェーズコールバック

EpxBool broadPhaseCallback(EpxUInt32 rigidBodyIdA,EpxUInt32 rigidBodyIdB,void *userData)
{
	// ジョイントで接続された剛体のペアは作成しない
	for(EpxUInt32 i=0;i<numJoints;i++) {
		if((joints[i].rigidBodyIdA == rigidBodyIdA && joints[i].rigidBodyIdB == rigidBodyIdB) || 
		   (joints[i].rigidBodyIdA == rigidBodyIdB && joints[i].rigidBodyIdB == rigidBodyIdA)) {
			return false;
		}
	}
	return true;
}

///////////////////////////////////////////////////////////////////////////////
// ベース剛体を回す

static int sid;
static int baseId;
static void animateBody()
{
	if(sid == 0) {
		states[baseId].m_orientation *= EpxQuat::rotationX(0.01f);
	}
}

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
	epxIntegrate(states,numRigidBodies,timeStep);
	perf.end("integrate");
	
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
		maxPairs,timeStep,&allocator,&stackAllocator,NULL,broadPhaseCallback);
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
	
	animateBody();
	
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
	bodies[fireRigidBodyId].m_mass = 10.0f;
	bodies[fireRigidBodyId].m_inertia = epxCalcInertiaBox(scale,10.0f);
	bodies[fireRigidBodyId].m_restitution = 0.8f;
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

void createSceneVarietyOfJoints()
{
	// ベース
	{
		int id = numRigidBodies++;
		
		EpxVector3 scale(5.0f,0.25f,0.25f);
		
		states[id].reset();
		states[id].m_motionType = EpxMotionTypeStatic;
		states[id].m_position = EpxVector3(0.0f,3.0f,0.0f);
		bodies[id].reset();
		collidables[id].reset();
		
		EpxShape shape;
		shape.reset();
		
		epxCreateConvexMesh(&shape.m_geometry,cylinder_vertices,cylinder_numVertices,cylinder_indices,cylinder_numIndices,scale);
		
		shape.userData = (void*)createRenderMesh(&shape.m_geometry);
		
		collidables[id].addShape(shape);
		collidables[id].finish();
		
		baseId = id;
	}
	
	// 共通の形状
	EpxVector3 scale(0.25f,0.25f,0.75f);
	EpxShape shape;
	shape.reset();
	epxCreateConvexMesh(&shape.m_geometry,box_vertices,box_numVertices,box_indices,box_numIndices,scale);
	shape.userData = (void*)createRenderMesh(&shape.m_geometry);
	
	// ボールジョイント
	{
		int id = numRigidBodies++;
		states[id].reset();
		states[id].m_position = EpxVector3(-4.0f,3.0f,0.5f+scale[2]);
		bodies[id].reset();
		bodies[id].m_mass = 5.0f;
		bodies[id].m_inertia = epxCalcInertiaBox(scale,5.0f);
		collidables[id].reset();
		collidables[id].addShape(shape);
		collidables[id].finish();
		EpxJoint &joint = joints[numJoints++];
		epxInitializeJointAsBall(joint,
			baseId,id,
			states[baseId],states[id],
			states[id].m_position + EpxVector3(0.0f,0.0f,-scale[2]), // 接続位置
			0.1f,0.0f);
	}
	
	// ヒンジジョイント
	{
		int id = numRigidBodies++;
		states[id].reset();
		states[id].m_position = EpxVector3(-2.0f,3.0f,0.5f+scale[2]);
		bodies[id].reset();
		bodies[id].m_mass = 5.0f;
		bodies[id].m_inertia = epxCalcInertiaBox(scale,5.0f);
		collidables[id].reset();
		collidables[id].addShape(shape);
		collidables[id].finish();
		EpxJoint &joint = joints[numJoints++];
		epxInitializeJointAsHinge(joint,
			baseId,id,
			states[baseId],states[id],
			states[id].m_position + EpxVector3(0.0f,0.0f,-scale[2]), // 接続位置
			EpxVector3(1.0f,0.0f,0.0f),
			0.1f,0.0f,-1.0f,1.0f);
	}

	// 固定ジョイント
	{
		int id = numRigidBodies++;
		states[id].reset();
		states[id].m_position = EpxVector3(-0.0f,3.0f,0.5f+scale[2]);
		bodies[id].reset();
		bodies[id].m_mass = 5.0f;
		bodies[id].m_inertia = epxCalcInertiaBox(scale,5.0f);
		collidables[id].reset();
		collidables[id].addShape(shape);
		collidables[id].finish();
		EpxJoint &joint = joints[numJoints++];
		epxInitializeJointAsFix(joint,
			baseId,id,
			states[baseId],states[id],
			states[id].m_position + EpxVector3(0.0f,0.0f,-scale[2]), // 接続位置
			0.1f);
	}
	
	// スイングツイストジョイント
	{
		int id = numRigidBodies++;
		states[id].reset();
		states[id].m_position = EpxVector3(2.0f,3.0f,0.5f+scale[2]);
		bodies[id].reset();
		bodies[id].m_mass = 5.0f;
		bodies[id].m_inertia = epxCalcInertiaBox(scale,5.0f);
		collidables[id].reset();
		collidables[id].addShape(shape);
		collidables[id].finish();
		EpxJoint &joint = joints[numJoints++];
		epxInitializeJointAsSwingTwist(joint,
			baseId,id,
			states[baseId],states[id],
			states[id].m_position + EpxVector3(0.0f,0.0f,-scale[2]), // 接続位置
			EpxVector3(0.0f,0.0f,1.0f),
			0.1f,0.0f,-0.1f,0.1f,0.0f,0.7f);
	}

	// スライダージョイント
	{
		int id = numRigidBodies++;
		states[id].reset();
		states[id].m_position = EpxVector3(4.0f,3.0f,0.5f+scale[2]);
		bodies[id].reset();
		bodies[id].m_mass = 5.0f;
		bodies[id].m_inertia = epxCalcInertiaBox(scale,5.0f);
		collidables[id].reset();
		collidables[id].addShape(shape);
		collidables[id].finish();
		EpxJoint &joint = joints[numJoints++];
		epxInitializeJointAsSlider(joint,
			baseId,id,
			states[baseId],states[id],
			states[id].m_position + EpxVector3(0.0f,0.0f,-scale[2]), // 接続位置
			EpxVector3(0.0f,0.0f,1.0f),
			0.1f,0.0f,-0.2f,1.0f);
	}
}

void createSceneBallJoint()
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

	{
		int id = numRigidBodies++;

		EpxVector3 scale(0.25f,0.25f,2.0f);

		states[id].reset();
		states[id].m_position = EpxVector3(0.0f,scale[1],0.0f);
		bodies[id].reset();
		bodies[id].m_mass = 1.0f;
		bodies[id].m_inertia = epxCalcInertiaBox(scale,1.0f);
		collidables[id].reset();
	
		EpxShape shape;
		shape.reset();
	
		epxCreateConvexMesh(&shape.m_geometry,box_vertices,box_numVertices,box_indices,box_numIndices,scale);
		shape.userData = (void*)createRenderMesh(&shape.m_geometry);
	
		collidables[id].addShape(shape);
		collidables[id].finish();
	}

	// ボールジョイントにより接続されたボックス
	int rigidBodyIdA,rigidBodyIdB;
	{
		int id = numRigidBodies++;

		EpxVector3 scale(1.0f,0.25f,0.25f);

		states[id].reset();
		states[id].m_position = EpxVector3(-scale[0],3.0f,0.0f);
		bodies[id].reset();
		bodies[id].m_mass = 1.0f;
		bodies[id].m_inertia = epxCalcInertiaBox(scale,1.0f);
		collidables[id].reset();
	
		EpxShape shape;
		shape.reset();
	
		epxCreateConvexMesh(&shape.m_geometry,cylinder_vertices,cylinder_numVertices,cylinder_indices,cylinder_numIndices,scale);
		shape.userData = (void*)createRenderMesh(&shape.m_geometry);
	
		collidables[id].addShape(shape);
		collidables[id].finish();
		
		rigidBodyIdA = id;
	}

	{
		int id = numRigidBodies++;

		EpxVector3 scale(1.0f,0.25f,0.25f);

		states[id].reset();
		states[id].m_position = EpxVector3(scale[0],3.0f,0.0f);
		bodies[id].reset();
		bodies[id].m_mass = 1.0f;
		bodies[id].m_inertia = epxCalcInertiaBox(scale,1.0f);
		collidables[id].reset();
	
		EpxShape shape;
		shape.reset();
	
		epxCreateConvexMesh(&shape.m_geometry,cylinder_vertices,cylinder_numVertices,cylinder_indices,cylinder_numIndices,scale);
		shape.userData = (void*)createRenderMesh(&shape.m_geometry);
	
		collidables[id].addShape(shape);
		collidables[id].finish();

		rigidBodyIdB = id;
	}

	EpxJoint &joint = joints[numJoints++];
	epxInitializeJointAsBall(joint,
		rigidBodyIdA,rigidBodyIdB,
		states[rigidBodyIdA],states[rigidBodyIdB],
		0.5f * (states[rigidBodyIdA].m_position + states[rigidBodyIdB].m_position), // 接続位置
		0.1f,0.0f);
}

void createSceneHingeJoint()
{
	// ヒンジジョイント
	int rigidBodyIdA,rigidBodyIdB;
	
	{
		int id = numRigidBodies++;

		EpxVector3 scale(1.25f,0.25f,0.25f);

		states[id].reset();
		states[id].m_motionType = EpxMotionTypeStatic;
		states[id].m_position = EpxVector3(0.0f,1.0f,0.0f);
		bodies[id].reset();
		collidables[id].reset();
	
		EpxShape shape;
		shape.reset();
		shape.m_offsetOrientation = EpxQuat::rotationZ(0.5f*EPX_PI);
		epxCreateConvexMesh(&shape.m_geometry,cylinder_vertices,cylinder_numVertices,cylinder_indices,cylinder_numIndices,scale);
		shape.userData = (void*)createRenderMesh(&shape.m_geometry);
	
		collidables[id].addShape(shape);
		collidables[id].finish();
		
		rigidBodyIdA = id;
	}

	{
		int id = numRigidBodies++;

		EpxVector3 scale(2.5f,1.0f,0.125f);

		states[id].reset();
		states[id].m_position = EpxVector3(0.0f,1.0f,0.0f);
		//states[id].m_angularVelocity = EpxVector3(0.0f,10.0f,0.0f);
		states[id].m_angularVelocity = EpxVector3(0.0f,-80.0f,0.0f);
		bodies[id].reset();
		bodies[id].m_mass = 10.0f;
		
		// 回転方向に安定するように慣性テンソル計算用の形状を調整
		bodies[id].m_inertia = epxCalcInertiaBox(EpxVector3(0.5f,3.0f,0.5f),10.0f);
		collidables[id].reset();
	
		EpxShape shape;
		shape.reset();
	
		epxCreateConvexMesh(&shape.m_geometry,box_vertices,box_numVertices,box_indices,box_numIndices,scale);
		shape.userData = (void*)createRenderMesh(&shape.m_geometry);
	
		collidables[id].addShape(shape);

		// AABB拡張させるため、CCD属性を与える
		EpxCoreSphere sphere[4];
		sphere[0].reset();
		sphere[1].reset();
		sphere[2].reset();
		sphere[3].reset();
		sphere[0].m_radius = 0.125f;
		sphere[1].m_radius = 0.125f;
		sphere[2].m_radius = 0.125f;
		sphere[3].m_radius = 0.125f;
		sphere[0].m_offsetPosition = EpxVector3( 2.25f, 0.0f,0.0f);
		sphere[1].m_offsetPosition = EpxVector3(-2.25f, 0.0f,0.0f);
		sphere[2].m_offsetPosition = EpxVector3( 1.25f, 0.0f,0.0f);
		sphere[3].m_offsetPosition = EpxVector3(-1.25f, 0.0f,0.0f);
		collidables[id].addCoreSphere(sphere[0]);
		collidables[id].addCoreSphere(sphere[1]);
		collidables[id].addCoreSphere(sphere[2]);
		collidables[id].addCoreSphere(sphere[3]);
		
		collidables[id].finish();
		
		rigidBodyIdB = id;
	}

	EpxJoint &joint = joints[numJoints++];
	epxInitializeJointAsHinge(joint,
		rigidBodyIdA,rigidBodyIdB,
		states[rigidBodyIdA],states[rigidBodyIdB],
		EpxVector3(0.0f,1.0f,0.0f), // 接続位置
		EpxVector3(0.0f,1.0f,0.0f), // ヒンジの回転軸
		0.1f,0.0f,0.0f,0.0f);
}

void createSceneFixedJoint()
{
	// 固定ジョイント
	int rigidBodyIdA,rigidBodyIdB;

	{
		int id = numRigidBodies++;

		EpxVector3 scale(1.0f,2.5f,2.5f);

		states[id].reset();
		states[id].m_motionType = EpxMotionTypeStatic;
		states[id].m_position = EpxVector3(-1.0f,1.0f,0.0f);
		bodies[id].reset();
		collidables[id].reset();
	
		EpxShape shape;
		shape.reset();
		epxCreateConvexMesh(&shape.m_geometry,box_vertices,box_numVertices,box_indices,box_numIndices,scale);
		shape.userData = (void*)createRenderMesh(&shape.m_geometry);
	
		collidables[id].addShape(shape);
		collidables[id].finish();
		
		rigidBodyIdA = id;
	}

	{
		int id = numRigidBodies++;

		EpxVector3 scale(1.0f,0.25f,0.25f);

		states[id].reset();
		states[id].m_position = EpxVector3(1.0f,1.0f,0.0f);
		bodies[id].reset();
		bodies[id].m_mass = 10.0f;
		// 安定するように慣性テンソル計算用の形状を大きめに設定
		bodies[id].m_inertia = epxCalcInertiaBox(10.0f*scale,10.0f);
		collidables[id].reset();
	
		EpxShape shape;
		shape.reset();
	
		epxCreateConvexMesh(&shape.m_geometry,box_vertices,box_numVertices,box_indices,box_numIndices,scale);
		shape.userData = (void*)createRenderMesh(&shape.m_geometry);
	
		collidables[id].addShape(shape);
		collidables[id].finish();
		
		rigidBodyIdB = id;
	}

	// 固定ジョイントが安定するように、接続位置を剛体の重心に配置
	EpxJoint &joint = joints[numJoints++];
	epxInitializeJointAsFix(joint,
		rigidBodyIdA,rigidBodyIdB,
		states[rigidBodyIdA],states[rigidBodyIdB],
		states[rigidBodyIdB].m_position, // 接続位置
		0.1f);
}

int createGear(const EpxVector3 &offsetPosition,const EpxQuat &offsetOrientation)
{
	int gearId;

	EpxFloat cogsWidth = 0.25f;

	// ヒンジジョイント
	int rigidBodyIdA,rigidBodyIdB;

	{
		int id = numRigidBodies++;

		EpxVector3 scale(0.3f,0.5f,0.5f);

		states[id].reset();
		states[id].m_motionType = EpxMotionTypeStatic;
		states[id].m_position = offsetPosition + EpxVector3(0.0f,1.0f,0.0f);
		states[id].m_orientation = offsetOrientation;
		bodies[id].reset();
		collidables[id].reset();
	
		EpxShape shape;
		shape.reset();
		shape.m_offsetOrientation = EpxQuat::rotationY(0.5f*EPX_PI);
		epxCreateConvexMesh(&shape.m_geometry,cylinder_vertices,cylinder_numVertices,cylinder_indices,cylinder_numIndices,scale);
		shape.userData = (void*)createRenderMesh(&shape.m_geometry);
	
		collidables[id].addShape(shape);
		collidables[id].finish();
		
		rigidBodyIdA = id;
	}

	{
		int id = numRigidBodies++;

		EpxVector3 scale(0.25f,2.0f,2.0f);

		states[id].reset();
		states[id].m_position = offsetPosition + EpxVector3(0.0f,1.0f,0.0f);
		states[id].m_orientation = offsetOrientation;
		bodies[id].reset();
		bodies[id].m_mass = 10.0f;
		// 回転方向に安定するように慣性テンソル計算用の形状を調整
		bodies[id].m_inertia = epxCalcInertiaBox(EpxVector3(2.5f,2.5f,25.0f),10.0f);
		collidables[id].reset();
	
		{
			EpxShape shape;
			shape.reset();
			shape.m_offsetOrientation = EpxQuat::rotationY(0.5f*EPX_PI);
			epxCreateConvexMesh(&shape.m_geometry,cylinder_vertices,cylinder_numVertices,cylinder_indices,cylinder_numIndices,scale);
			shape.userData = (void*)createRenderMesh(&shape.m_geometry);
			collidables[id].addShape(shape);
		}

		for(int i=0;i<4;i++) {
			const EpxVector3 cogsScale(2.5f,cogsWidth,0.25f);
			EpxShape shape;
			shape.reset();
			shape.m_offsetOrientation = EpxQuat::rotationZ(i*0.25f*EPX_PI);
			epxCreateConvexMesh(&shape.m_geometry,box_vertices,box_numVertices,box_indices,box_numIndices,cogsScale);
			shape.userData = (void*)createRenderMesh(&shape.m_geometry);
			collidables[id].addShape(shape);
		}

		collidables[id].finish();

		rigidBodyIdB = id;

		gearId = id;
	}

	EpxJoint &joint = joints[numJoints++];
	epxInitializeJointAsHinge(joint,
		rigidBodyIdA,rigidBodyIdB,
		states[rigidBodyIdA],states[rigidBodyIdB],
		offsetPosition + EpxVector3(0.0f,1.0f,0.0f), // 接続位置
		EpxVector3(0.0f,0.0f,1.0f), // ヒンジの回転軸
		0.1f,0.0f,0.0f,0.0f);

	return gearId;
}

void createSceneGearJoint()
{
	createGear(EpxVector3(2.0f,1.0f,0.0f),EpxQuat::identity());

	int gearId = createGear(EpxVector3(-2.5f,1.0f,0.0f),EpxQuat::rotationZ(0.125f*EPX_PI));
	states[gearId].m_angularVelocity = EpxVector3(0.0f,0.0f,-10.0f);
}

void createSceneChain()
{
	const EpxVector3 chainScale(0.125f,0.5f,0.125f);
	const EpxVector3 ballScale(1.5f);

	// ダミー
	int dummyId;
	{
		dummyId = numRigidBodies++;
		
		EpxVector3 scale(0.01f);
		
		states[dummyId].reset();
		states[dummyId].m_motionType = EpxMotionTypeStatic;
		states[dummyId].m_position = EpxVector3(0.0f,-10.0f,0.0f);
		bodies[dummyId].reset();
		collidables[dummyId].reset();
		
		EpxShape shape;
		shape.reset();
		
		epxCreateConvexMesh(&shape.m_geometry,box_vertices,box_numVertices,box_indices,box_numIndices,scale);
		shape.userData = (void*)createRenderMesh(&shape.m_geometry);
		
		collidables[dummyId].addShape(shape);
		collidables[dummyId].finish();
	}

	// 1kgの鎖の先に50kgの錘を接続
	
	// 慣性テンソル調整なし
	for(int i=0;i<5;i++) {
		int id = numRigidBodies++;

		states[id].reset();
		states[id].m_position = EpxVector3(-2.5f,5.0f,0.0f) + EpxVector3(0.0f,-i*chainScale[1]*2.0f,0.0f);
		bodies[id].reset();
		bodies[id].m_mass = 1.0f;
		bodies[id].m_inertia = epxCalcInertiaBox(chainScale,1.0f);
		collidables[id].reset();
	
		EpxShape shape;
		shape.reset();
	
		epxCreateConvexMesh(&shape.m_geometry,box_vertices,box_numVertices,box_indices,box_numIndices,chainScale);
		shape.userData = (void*)createRenderMesh(&shape.m_geometry);
	
		collidables[id].addShape(shape);
		collidables[id].finish();
		
		EpxJoint &joint = joints[numJoints++];
		
		if(i>0) {
			epxInitializeJointAsBall(joint,
				id-1,id,
				states[id-1],states[id],
				states[id-1].m_position + EpxVector3(0.0f,-chainScale[1],0.0f), // 接続位置
				0.1f,0.0f);
		}
		else {
			epxInitializeJointAsBall(joint,
				dummyId,id,
				states[dummyId],states[id],
				states[id].m_position + EpxVector3(0.0f,chainScale[1],0.0f), // 接続位置
				0.1f,0.0f);
		}
	}

	{
		int id = numRigidBodies++;
		
		states[id].reset();
		states[id].m_position = states[id-1].m_position + EpxVector3(0.0f,-chainScale[1]-ballScale[1],0.0f);
		bodies[id].reset();
		bodies[id].m_mass = 50.0f;
		bodies[id].m_inertia = epxCalcInertiaSphere(ballScale[1],50.0f);
		collidables[id].reset();
		
		EpxShape shape;
		shape.reset();
		
		epxCreateConvexMesh(&shape.m_geometry,sphere_vertices,sphere_numVertices,sphere_indices,sphere_numIndices,ballScale);
		shape.userData = (void*)createRenderMesh(&shape.m_geometry);
		
		collidables[id].addShape(shape);
		collidables[id].finish();

		EpxJoint &joint = joints[numJoints++];
		epxInitializeJointAsBall(joint,
			id-1,id,
			states[id-1],states[id],
			states[id-1].m_position + EpxVector3(0.0f,-chainScale[1],0.0f), // 接続位置
			0.1f,0.0f);
	}

	// 慣性テンソル調整あり
	for(int i=0;i<5;i++) {
		int id = numRigidBodies++;

		states[id].reset();
		states[id].m_position = EpxVector3(2.5f,5.0f,0.0f) + EpxVector3(0.0f,-i*chainScale[1]*2.0f,0.0f);
		bodies[id].reset();
		bodies[id].m_mass = 1.0f;
		bodies[id].m_inertia = epxCalcInertiaBox(15.0f * chainScale,1.0f); // 慣性テンソル増加
		collidables[id].reset();
	
		EpxShape shape;
		shape.reset();
	
		epxCreateConvexMesh(&shape.m_geometry,box_vertices,box_numVertices,box_indices,box_numIndices,chainScale);
		shape.userData = (void*)createRenderMesh(&shape.m_geometry);
	
		collidables[id].addShape(shape);
		collidables[id].finish();
		
		EpxJoint &joint = joints[numJoints++];
		joint.reset();
		if(i>0) {
			epxInitializeJointAsBall(joint,
				id-1,id,
				states[id-1],states[id],
				states[id-1].m_position + EpxVector3(0.0f,-chainScale[1],0.0f), // 接続位置
				0.1f,0.0f);
		}
		else {
			epxInitializeJointAsBall(joint,
				dummyId,id,
				states[dummyId],states[id],
				states[id].m_position + EpxVector3(0.0f,chainScale[1],0.0f), // 接続位置
				0.1f,0.0f);
		}
	}

	{
		int id = numRigidBodies++;
		
		states[id].reset();
		states[id].m_position = states[id-1].m_position + EpxVector3(0.0f,-chainScale[1]-ballScale[1],0.0f);
		bodies[id].reset();
		bodies[id].m_mass = 50.0f;
		bodies[id].m_inertia = epxCalcInertiaSphere(ballScale[1],50.0f);
		collidables[id].reset();
		
		EpxShape shape;
		shape.reset();
		
		epxCreateConvexMesh(&shape.m_geometry,sphere_vertices,sphere_numVertices,sphere_indices,sphere_numIndices,ballScale);
		shape.userData = (void*)createRenderMesh(&shape.m_geometry);
		
		collidables[id].addShape(shape);
		collidables[id].finish();

		EpxJoint &joint = joints[numJoints++];
		epxInitializeJointAsBall(joint,
			id-1,id,
			states[id-1],states[id],
			states[id-1].m_position + EpxVector3(0.0f,-chainScale[1],0.0f), // 接続位置
			0.1f,0.0f);
	}
}

static const int maxScenes = 6;
static const char titles[][32] = {
	"variety of joints",
	"ball joint",
	"hinge joint",
	"fixed joint",
	"gear joint",
	"chain joint",
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
	
	sid = sceneId%maxScenes;

	createFireBody();

	switch(sid) {
		case 0:
		createSceneVarietyOfJoints();
		break;
		
		case 1:
		createSceneBallJoint();
		break;
		
		case 2:
		createSceneHingeJoint();
		break;

		case 3:
		createSceneFixedJoint();
		break;

		case 4:
		createSceneGearJoint();
		break;

		case 5:
		createSceneChain();
		break;
	}
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
	epxPrintf("fire pos %f %f %f vel %f %f %f\n",
		position[0],position[1],position[2],velocity[0],velocity[1],velocity[2]);

	states[fireRigidBodyId].m_motionType = EpxMotionTypeActive;
	states[fireRigidBodyId].m_position = position;
	states[fireRigidBodyId].m_linearVelocity = velocity;
	states[fireRigidBodyId].m_angularVelocity = EpxVector3(0.0f);
}
