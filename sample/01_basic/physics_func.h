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

#ifndef PHYSICS_FUNC_H
#define PHYSICS_FUNC_H

#include "../../easy_physics/EpxInclude.h"

// シミュレーション関数
bool physicsInit();
void physicsRelease();
void physicsCreateScene(int sceneId);
void physicsSimulate();

// シーンのタイトル名を取得する
const char *physicsGetSceneTitle(int i);

// 剛体に関連するデータを取得する関数
int physicsGetNumRigidbodies();
const EasyPhysics::EpxState &physicsGetState(int i);
const EasyPhysics::EpxRigidBody &physicsGetRigidBody(int i);
const EasyPhysics::EpxCollidable &physicsGetCollidable(int i);

// 衝突情報を取得する関数
int physicsGetNumContacts();
const EasyPhysics::EpxContact &physicsGetContact(int i);
EasyPhysics::EpxUInt32 physicsGetRigidBodyAInContact(int i);
EasyPhysics::EpxUInt32 physicsGetRigidBodyBInContact(int i);

// ワールドへの干渉
void physicsFire(const EasyPhysics::EpxVector3 &position,const EasyPhysics::EpxVector3 &velocity);

#endif // PHYSICS_FUNC_H
