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

#include "EpxConstraintSolver.h"
#include "../collision/EpxVectorFunction.h"
#include "../elements/EpxSolverBody.h"
#include "../elements/EpxSolverFunction.h"

namespace EasyPhysics {

void epxSolveConstraints(
	EpxState *states,
	const EpxRigidBody *bodies,
	EpxUInt32 numRigidBodies,
	const EpxPair *pairs,
	EpxUInt32 numPairs,
	EpxJoint *joints,
	EpxUInt32 numJoints,
	EpxUInt32 iteration,
	EpxFloat bias,
	EpxFloat slop,
	EpxFloat timeStep,
	EpxStackAllocator *stackAllocator)
{
	EPX_ALWAYS_ASSERT(states);
	EPX_ALWAYS_ASSERT(bodies);
	EPX_ALWAYS_ASSERT(pairs);
	
	// ソルバー用プロキシを作成
	EpxSolverBody *solverBodies = (EpxSolverBody*)stackAllocator->allocate(sizeof(EpxSolverBody)*numRigidBodies);
	EPX_ASSERT(solverBodies);

	for(EpxUInt32 i=0;i<numRigidBodies;i++) {
		EpxState &state = states[i];
		const EpxRigidBody &body = bodies[i];
		EpxSolverBody &solverBody = solverBodies[i];
		
		solverBody.orientation = state.m_orientation;
		solverBody.deltaLinearVelocity = EpxVector3(0.0f);
		solverBody.deltaAngularVelocity = EpxVector3(0.0f);
		
		if(state.m_motionType == EpxMotionTypeStatic) {
			solverBody.massInv = 0.0f;
			solverBody.inertiaInv = EpxMatrix3(0.0f);
		}
		else {
			solverBody.massInv = 1.0f/body.m_mass;
			EpxMatrix3 m(solverBody.orientation);
			solverBody.inertiaInv = m * inverse(body.m_inertia) * transpose(m);
		}
	}
	
	// 拘束のセットアップ
	for(EpxUInt32 i=0;i<numJoints;i++) {
		EpxJoint &joint = joints[i];
		
		EpxState &stateA = states[joint.rigidBodyIdA];
		const EpxRigidBody &bodyA = bodies[joint.rigidBodyIdA];
		EpxSolverBody &solverBodyA = solverBodies[joint.rigidBodyIdA];
		
		EpxState &stateB = states[joint.rigidBodyIdB];
		const EpxRigidBody &bodyB = bodies[joint.rigidBodyIdB];
		EpxSolverBody &solverBodyB = solverBodies[joint.rigidBodyIdB];
		
		epxSetupJointConstraint(joint,stateA,stateB,solverBodyA,solverBodyB,timeStep);
	}

	for(EpxUInt32 i=0;i<numPairs;i++) {
		const EpxPair &pair = pairs[i];
		
		EpxState &stateA = states[pair.rigidBodyA];
		const EpxRigidBody &bodyA = bodies[pair.rigidBodyA];
		EpxSolverBody &solverBodyA = solverBodies[pair.rigidBodyA];
		
		EpxState &stateB = states[pair.rigidBodyB];
		const EpxRigidBody &bodyB = bodies[pair.rigidBodyB];
		EpxSolverBody &solverBodyB = solverBodies[pair.rigidBodyB];
		
		EPX_ALWAYS_ASSERT(pair.contact);
		
		EpxFloat friction = sqrtf(bodyA.m_friction * bodyB.m_friction);
		EpxFloat restitution = pair.type==EpxPairTypeNew ? 0.5f*(bodyA.m_restitution + bodyB.m_restitution) : 0.0f;
		
		epxSetupContactConstraint(*pair.contact,friction,restitution,stateA,stateB,solverBodyA,solverBodyB,bias,slop,timeStep);
	}
	
	// Warm starting
	for(EpxUInt32 i=0;i<numPairs;i++) {
		const EpxPair &pair = pairs[i];
		
		EpxSolverBody &solverBodyA = solverBodies[pair.rigidBodyA];
		EpxSolverBody &solverBodyB = solverBodies[pair.rigidBodyB];
		
		for(EpxUInt32 j=0;j<pair.contact->m_numContactPoints;j++) {
			EpxContactPoint &cp = pair.contact->m_contactPoints[j];
			EpxVector3 rA = rotate(solverBodyA.orientation,cp.pointA);
			EpxVector3 rB = rotate(solverBodyB.orientation,cp.pointB);

			for(EpxUInt32 k=0;k<3;k++) {
				EpxFloat deltaImpulse = cp.constraints[k].accumImpulse;
				solverBodyA.deltaLinearVelocity += deltaImpulse * solverBodyA.massInv * cp.constraints[k].axis;
				solverBodyA.deltaAngularVelocity += deltaImpulse * solverBodyA.inertiaInv * cross(rA,cp.constraints[k].axis);
				solverBodyB.deltaLinearVelocity -= deltaImpulse * solverBodyB.massInv * cp.constraints[k].axis;
				solverBodyB.deltaAngularVelocity -= deltaImpulse * solverBodyB.inertiaInv * cross(rB,cp.constraints[k].axis);
			}
		}
	}

	// 拘束の演算
	for(EpxUInt32 itr=0;itr<iteration;itr++) {
		for(EpxUInt32 i=0;i<numJoints;i++) {
			EpxJoint &joint = joints[i];

			EpxSolverBody &solverBodyA = solverBodies[joint.rigidBodyIdA];
			EpxSolverBody &solverBodyB = solverBodies[joint.rigidBodyIdB];
			
			epxSolveJointConstraint(joint,solverBodyA,solverBodyB);
		}

		for(EpxUInt32 i=0;i<numPairs;i++) {
			const EpxPair &pair = pairs[i];

			EpxSolverBody &solverBodyA = solverBodies[pair.rigidBodyA];
			EpxSolverBody &solverBodyB = solverBodies[pair.rigidBodyB];
			
			epxSolveContactConstraint(*pair.contact,solverBodyA,solverBodyB);
		}
	}
	
	// 速度を更新
	for(EpxUInt32 i=0;i<numRigidBodies;i++) {
		states[i].m_linearVelocity += solverBodies[i].deltaLinearVelocity;
		states[i].m_angularVelocity += solverBodies[i].deltaAngularVelocity;
	}
	
	stackAllocator->deallocate(solverBodies);
}

} // namespace EasyPhysics
