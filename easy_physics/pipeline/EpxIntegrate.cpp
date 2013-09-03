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

#include "EpxIntegrate.h"

namespace EasyPhysics {

void epxIntegrate(EpxState *states,EpxUInt32 numRigidBodies,EpxFloat timeStep)
{
	for(EpxUInt32 i=0;i<numRigidBodies;i++) {
		EpxState &state = states[i];
		
		EpxQuat dAng = EpxQuat(state.m_angularVelocity,0) * state.m_orientation * 0.5f;

		state.m_position += state.m_linearVelocity * timeStep;
		state.m_orientation = normalize(state.m_orientation + dAng * timeStep);
	}
}

void epxApplyExternalForce(
	EpxState &state,
	const EpxRigidBody &body,
	const EpxVector3 &externalForce,
	const EpxVector3 &externalTorque,
	EpxFloat timeStep)
{
	if(state.m_motionType == EpxMotionTypeStatic) {
		return;
	}
	
	EpxMatrix3 orientation(state.m_orientation);
	EpxMatrix3 worldInertia = orientation * body.m_inertia * transpose(orientation);
	EpxMatrix3 worldInertiaInv = orientation * inverse(body.m_inertia) * transpose(orientation);
	EpxVector3 angularMomentum = worldInertia * state.m_angularVelocity;
	
	state.m_linearVelocity += externalForce / body.m_mass * timeStep;
	angularMomentum += externalTorque * timeStep;
	state.m_angularVelocity = worldInertiaInv * angularMomentum;

	EpxFloat linVelSqr = lengthSqr(state.m_linearVelocity);
	if(linVelSqr > (EPX_MAX_LINEAR_VELOCITY*EPX_MAX_LINEAR_VELOCITY)) {
		state.m_linearVelocity = (state.m_linearVelocity/sqrtf(linVelSqr)) * EPX_MAX_LINEAR_VELOCITY;
	}
	
	EpxFloat angVelSqr = lengthSqr(state.m_angularVelocity);
	if(angVelSqr > (EPX_MAX_ANGULAR_VELOCITY*EPX_MAX_ANGULAR_VELOCITY)) {
		state.m_angularVelocity = (state.m_angularVelocity/sqrtf(angVelSqr)) * EPX_MAX_ANGULAR_VELOCITY;
	}
}

} // namespace EasyPhysics
