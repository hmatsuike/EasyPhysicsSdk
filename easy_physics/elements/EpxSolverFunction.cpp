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

#include "EpxSolverFunction.h"
#include "../collision/EpxVectorFunction.h"

namespace EasyPhysics {

static inline
void epxSolveLinearConstraint(
	EpxConstraint &constraint,
	EpxSolverBody &solverBodyA,
	EpxSolverBody &solverBodyB,
	const EpxVector3 &rA,
	const EpxVector3 &rB)
{
	EpxFloat deltaImpulse = constraint.rhs;
	EpxVector3 deltaVelocityA = solverBodyA.deltaLinearVelocity + cross(solverBodyA.deltaAngularVelocity,rA);
	EpxVector3 deltaVelocityB = solverBodyB.deltaLinearVelocity + cross(solverBodyB.deltaAngularVelocity,rB);
	deltaImpulse -= constraint.jacDiagInv * dot(constraint.axis,deltaVelocityA - deltaVelocityB);
	EpxFloat oldImpulse = constraint.accumImpulse;
	constraint.accumImpulse = EPX_CLAMP(oldImpulse + deltaImpulse,constraint.lowerLimit,constraint.upperLimit);
	deltaImpulse = constraint.accumImpulse - oldImpulse;
	solverBodyA.deltaLinearVelocity  += deltaImpulse * solverBodyA.massInv * constraint.axis;
	solverBodyA.deltaAngularVelocity += deltaImpulse * solverBodyA.inertiaInv * cross(rA,constraint.axis);
	solverBodyB.deltaLinearVelocity  -= deltaImpulse * solverBodyB.massInv * constraint.axis;
	solverBodyB.deltaAngularVelocity -= deltaImpulse * solverBodyB.inertiaInv * cross(rB,constraint.axis);
}

static inline
void epxSolveAngularConstraint(
	EpxConstraint &constraint,
	EpxSolverBody &solverBodyA,
	EpxSolverBody &solverBodyB)
{
	EpxFloat deltaImpulse = constraint.rhs;
	deltaImpulse -= constraint.jacDiagInv * dot(constraint.axis,solverBodyA.deltaAngularVelocity-solverBodyB.deltaAngularVelocity);
	EpxFloat oldImpulse = constraint.accumImpulse;
	constraint.accumImpulse = EPX_CLAMP(oldImpulse + deltaImpulse,constraint.lowerLimit,constraint.upperLimit);
	deltaImpulse = constraint.accumImpulse - oldImpulse;
	solverBodyA.deltaAngularVelocity += deltaImpulse * solverBodyA.inertiaInv * constraint.axis;
	solverBodyB.deltaAngularVelocity -= deltaImpulse * solverBodyB.inertiaInv * constraint.axis;
}

static inline
EpxFloat epxModifyAngle(EpxFloat angle)
{
	if(angle > EPX_PI) angle -= 2.0f * EPX_PI;
	else if(angle < -EPX_PI) angle += 2.0f * EPX_PI;
	return angle;
}

static inline
void epxGetRotationAngleAndAxis(const EpxQuat &unitQuat,EpxFloat &angle,EpxVector3 &axis)
{
	if(fabs(unitQuat.getW()) < 1.0f-EPX_EPSILON && lengthSqr(unitQuat.getXYZ()) > EPX_EPSILON) {
		EpxFloat angleHalf = acosf(unitQuat.getW());
		EpxFloat sinAngleHalf = sinf(angleHalf);

		if(fabs(sinAngleHalf) > 1.0e-10f) {
			axis = unitQuat.getXYZ()/sinAngleHalf;
		} else {
			axis = unitQuat.getXYZ();
		}
		angle = 2.0f*angleHalf;
	} else {
		angle = 0.0f;
		axis = EpxVector3(1.0f, 0.0f, 0.0f);
	}
}

static inline
void epxCalcLinearLimit(EpxJointConstraint &jointConstraint,EpxFloat &posErr,EpxFloat &velocityAmp,EpxFloat &lowerLimit,EpxFloat &upperLimit)
{
	const EpxFloat slop = 0.01f;
	
	switch(jointConstraint.lockType) {
		case EpxLockTypeFree:
		posErr = 0.0f;
		velocityAmp *= jointConstraint.damping;
		break;
		
		case EpxLockTypeLimit:
		if(posErr >= jointConstraint.lowerLimit && posErr <= jointConstraint.upperLimit) {
			posErr = 0.0f;
			velocityAmp *= jointConstraint.damping;
		}
		else {
			if(posErr < jointConstraint.lowerLimit) {
				posErr = posErr - jointConstraint.lowerLimit;
				posErr = EPX_MIN(0.0f,posErr+slop);
				upperLimit = EPX_MIN(0.0f,upperLimit);
				velocityAmp = 1.0f;
			}
			else {
				posErr = posErr - jointConstraint.upperLimit;
				posErr = EPX_MAX(0.0f,posErr-slop);
				lowerLimit = EPX_MAX(0.0f,lowerLimit);
				velocityAmp = 1.0f;
			}
		}
		break;
		
		default:
		break;
	}
}

static inline
void epxCalcAngularLimit(EpxJointConstraint &jointConstraint,EpxFloat &posErr,EpxFloat &velocityAmp,EpxFloat &lowerLimit,EpxFloat &upperLimit)
{
	const EpxFloat slop = 0.01f;
	
	switch(jointConstraint.lockType) {
		case EpxLockTypeFree:
		posErr = 0.0f;
		velocityAmp *= jointConstraint.damping;
		break;
		
		case EpxLockTypeLimit:
		if(posErr >= jointConstraint.lowerLimit && posErr <= jointConstraint.upperLimit) {
			posErr = 0.0f;
			velocityAmp *= jointConstraint.damping;
		}
		else {
			EpxFloat diffLower = epxModifyAngle(jointConstraint.lowerLimit - posErr);
			EpxFloat diffUpper = epxModifyAngle(jointConstraint.upperLimit - posErr);
			if(0.0f >= diffLower && 0.0f <= diffUpper) {
				posErr = 0.0f;
				velocityAmp *= jointConstraint.damping;
			}
			else {
				if(0.0f < diffLower) {
					posErr = -diffLower;
					posErr = EPX_MIN(0.0f,posErr+slop);
					upperLimit = EPX_MIN(0.0f,upperLimit);
					velocityAmp = 1.0f;
				}
				else {
					posErr = -diffUpper;
					posErr = EPX_MAX(0.0f,posErr-slop);
					lowerLimit = EPX_MAX(0.0f,lowerLimit);
					velocityAmp = 1.0f;
				}
			}
		}
		break;
		
		default:
		break;
	}
}

void epxCalcJointAngleHinge(const EpxMatrix3 &worldFrameA,const EpxMatrix3 &worldFrameB,EpxVector3 &angle,EpxMatrix3 &axis)
{
	// フレームA座標系への変換マトリクス
	EpxMatrix3 frameBA = transpose(worldFrameA) * worldFrameB;

	// スイング回転を算出
	EpxQuat swing,qBA(frameBA);
	swing = EpxQuat::rotation(EpxVector3(1.0f,0.0f,0.0f),frameBA.getCol0());
	
	// 回転軸の角度を算出
	EpxVector3 by = frameBA.getCol1();
	angle[0] = atan2f(dot(by,EpxVector3(0.0f,0.0f,1.0f)),dot(by,EpxVector3(0.0f,1.0f,0.0f)));
	
	// スイング軸と回転角度を算出
	EpxFloat angle_;
	epxGetRotationAngleAndAxis(normalize(swing),angle_,axis[1]);
	angle[1] = angle_;

	if(angle[1] < EPX_EPSILON) {
		axis[1] = EpxVector3(0.0f,1.0f,0.0f);
	}

	axis[0] = worldFrameB.getCol0(); // axis
	axis[1] = worldFrameA * axis[1]; // swing
	axis[2] = cross(axis[0],axis[1]); // stabilize
	angle[2] = 0.0f;
}

void epxCalcJointAngleSwingTwist(const EpxMatrix3 &worldFrameA,const EpxMatrix3 &worldFrameB,EpxVector3 &angle,EpxMatrix3 &axis)
{
	// フレームA座標系への変換マトリクス
	EpxMatrix3 frameBA = transpose(worldFrameA) * worldFrameB;

	// クォータニオン回転をtwistとswingに分離
	EpxQuat swing,twist,qBA(frameBA);
	swing = EpxQuat::rotation(EpxVector3(1.0f,0.0f,0.0f),frameBA.getCol0());
	twist = qBA * conj(swing);
	
	if(dot(twist,EpxQuat::rotationX(0.0f)) < 0.0f) {
		twist = -twist;
	}
	
	// それぞれの回転軸と回転角度を算出
	EpxFloat angle_0,angle_1;
	epxGetRotationAngleAndAxis(normalize(twist),angle_0,axis[0]);
	epxGetRotationAngleAndAxis(normalize(swing),angle_1,axis[1]);
	angle[0] = angle_0;
	angle[1] = angle_1;

	if(angle[1] < EPX_EPSILON) {
		axis[1] = EpxVector3(0.0f,1.0f,0.0f);
	}
	
	// twistの軸方向のチェック
	if(dot(axis[0],frameBA.getCol0()) < 0.0f) {
		angle[0] = -angle[0];
	}
	
	axis[0] = worldFrameB.getCol0();
	axis[1] = worldFrameA * axis[1];
	axis[2] = cross(axis[0],axis[1]);
	angle[2] = 0.0f;
}

void epxSetupJointConstraint(
	EpxJoint &joint,
	EpxState &stateA,
	EpxState &stateB,
	EpxSolverBody &solverBodyA,
	EpxSolverBody &solverBodyB,
	EpxFloat timeStep)
{
	EpxVector3 rA = rotate(solverBodyA.orientation,joint.anchorA);
	EpxVector3 rB = rotate(solverBodyB.orientation,joint.anchorB);

	EpxVector3 vA = stateA.m_linearVelocity + cross(stateA.m_angularVelocity,rA);
	EpxVector3 vB = stateB.m_linearVelocity + cross(stateB.m_angularVelocity,rB);
	EpxVector3 vAB = vA-vB;

	EpxVector3 angAB = stateA.m_angularVelocity - stateB.m_angularVelocity;

	EpxVector3 distance = stateA.m_position - stateB.m_position + rA - rB;

	EpxMatrix3 worldFrameA,worldFrameB;
	worldFrameA = EpxMatrix3(solverBodyA.orientation) * joint.frameA;
	worldFrameB = EpxMatrix3(solverBodyB.orientation) * joint.frameB;
	
	// Linear Constraint
	EpxMatrix3 K = EpxMatrix3::scale(EpxVector3(solverBodyA.massInv + solverBodyB.massInv)) - 
			crossMatrix(rA) * solverBodyA.inertiaInv * crossMatrix(rA) - 
			crossMatrix(rB) * solverBodyB.inertiaInv * crossMatrix(rB);
	
	for(int c=0;c<3;c++) {
		EpxJointConstraint &jointConstraint = joint.jointConstraints[c];
		EpxConstraint &constraint = jointConstraint.constraint;
		
		EpxVector3 normal = worldFrameA[c];
		
		EpxFloat posErr = dot(distance,-normal);
		EpxFloat lowerLimit = -EPX_FLT_MAX;
		EpxFloat upperLimit =  EPX_FLT_MAX;
		EpxFloat velocityAmp = 1.0f;
		
		epxCalcLinearLimit(jointConstraint,posErr,velocityAmp,lowerLimit,upperLimit);
		
		EpxFloat denom = dot(K*normal,normal);
		
		constraint.jacDiagInv = velocityAmp/denom;
		constraint.rhs = -dot(vAB,normal);
		constraint.rhs -= (jointConstraint.bias * (-posErr)) / timeStep;
		constraint.rhs *= constraint.jacDiagInv;
		constraint.lowerLimit = lowerLimit;
		constraint.upperLimit = upperLimit;
		constraint.axis = normal;
		constraint.accumImpulse = 0.0f;
	}

	EpxMatrix3 axis;
	EpxVector3 angle;
	EPX_ALWAYS_ASSERT(joint.calcAngleFunc);
	joint.calcAngleFunc(worldFrameA,worldFrameB,angle,axis);
	
	// Angular Constraint
	for(int c=3;c<6;c++) {
		EpxJointConstraint &jointConstraint = joint.jointConstraints[c];
		EpxConstraint &constraint = jointConstraint.constraint;

		EpxVector3 normal = axis[c-3];

		EpxFloat posErr = angle[c-3];
		EpxFloat lowerLimit = -EPX_FLT_MAX;
		EpxFloat upperLimit =  EPX_FLT_MAX;
		EpxFloat velocityAmp = 1.0f;
		
		epxCalcAngularLimit(jointConstraint,posErr,velocityAmp,lowerLimit,upperLimit);
		
		EpxFloat denom = dot((solverBodyA.inertiaInv+solverBodyB.inertiaInv)*normal,normal);
		
		constraint.jacDiagInv = velocityAmp/denom;
		constraint.rhs = -dot(angAB,normal); // velocity error
		constraint.rhs -= (jointConstraint.bias * (-posErr)) / timeStep; // position error
		constraint.rhs *= constraint.jacDiagInv;
		constraint.lowerLimit = lowerLimit;
		constraint.upperLimit = upperLimit;
		constraint.axis = normal;
		constraint.accumImpulse = 0.0f;
	}
}

void epxSetupContactConstraint(
	EpxContact &contact,
	EpxFloat friction,
	EpxFloat restitution,
	EpxState &stateA,
	EpxState &stateB,
	EpxSolverBody &solverBodyA,
	EpxSolverBody &solverBodyB,
	EpxFloat bias,
	EpxFloat slop,
	EpxFloat timeStep)
{
	contact.m_friction = friction;

	for(EpxUInt32 j=0;j<contact.m_numContactPoints;j++) {
		EpxContactPoint &cp = contact.m_contactPoints[j];

		EpxVector3 rA = rotate(solverBodyA.orientation,cp.pointA);
		EpxVector3 rB = rotate(solverBodyB.orientation,cp.pointB);

		EpxMatrix3 K = EpxMatrix3::scale(EpxVector3(solverBodyA.massInv + solverBodyB.massInv)) - 
				crossMatrix(rA) * solverBodyA.inertiaInv * crossMatrix(rA) - 
				crossMatrix(rB) * solverBodyB.inertiaInv * crossMatrix(rB);
		
		EpxVector3 velocityA = stateA.m_linearVelocity + cross(stateA.m_angularVelocity,rA);
		EpxVector3 velocityB = stateB.m_linearVelocity + cross(stateB.m_angularVelocity,rB);
		EpxVector3 relativeVelocity = velocityA - velocityB;

		EpxVector3 tangent1,tangent2;

		epxCalcTangentVector(cp.normal,tangent1,tangent2);

		// Normal
		{
			EpxVector3 axis = cp.normal;
			EpxFloat denom = dot(K * axis,axis);
			cp.constraints[0].jacDiagInv = 1.0f / denom;
			cp.constraints[0].rhs = -(1.0f + restitution) * dot(relativeVelocity,axis); // velocity error
			cp.constraints[0].rhs -= (bias * EPX_MIN(0.0f,cp.distance + slop)) / timeStep; // position error
			cp.constraints[0].rhs *= cp.constraints[0].jacDiagInv;
			cp.constraints[0].lowerLimit = 0.0f;
			cp.constraints[0].upperLimit = EPX_FLT_MAX;
			cp.constraints[0].axis = axis;
		}

		// Tangent1
		{
			EpxVector3 axis = tangent1;
			EpxFloat denom = dot(K * axis,axis);
			cp.constraints[1].jacDiagInv = 1.0f / denom;
			cp.constraints[1].rhs = -dot(relativeVelocity,axis);
			cp.constraints[1].rhs *= cp.constraints[1].jacDiagInv;
			cp.constraints[1].lowerLimit = 0.0f;
			cp.constraints[1].upperLimit = 0.0f;
			cp.constraints[1].axis = axis;
		}

		// Tangent2
		{
			EpxVector3 axis = tangent2;
			EpxFloat denom = dot(K * axis,axis);
			cp.constraints[2].jacDiagInv = 1.0f / denom;
			cp.constraints[2].rhs = -dot(relativeVelocity,axis);
			cp.constraints[2].rhs *= cp.constraints[2].jacDiagInv;
			cp.constraints[2].lowerLimit = 0.0f;
			cp.constraints[2].upperLimit = 0.0f;
			cp.constraints[2].axis = axis;
		}
	}

	for(EpxUInt32 j=0;j<contact.m_numClosestPoints;j++) {
		EpxClosestPoint &cp = contact.m_closestPoints[j];
		
		EpxVector3 rA = rotate(solverBodyA.orientation,cp.pointA);
		EpxVector3 rB = rotate(solverBodyB.orientation,cp.pointB);
		
		EpxMatrix3 K = EpxMatrix3::scale(EpxVector3(solverBodyA.massInv + solverBodyB.massInv)) - 
				crossMatrix(rA) * solverBodyA.inertiaInv * crossMatrix(rA) - 
				crossMatrix(rB) * solverBodyB.inertiaInv * crossMatrix(rB);
		
		EpxVector3 velocityA = stateA.m_linearVelocity + cross(stateA.m_angularVelocity,rA);
		EpxVector3 velocityB = stateB.m_linearVelocity + cross(stateB.m_angularVelocity,rB);
		EpxVector3 relativeVelocity = velocityA - velocityB;
		
		EpxVector3 axis = cp.normal;
		EpxFloat denom = dot(K * axis,axis);
		cp.constraint.jacDiagInv = 1.0f / denom;
		cp.constraint.rhs = -(dot(relativeVelocity,axis) + cp.distance / timeStep); // velocity error
		cp.constraint.rhs *= cp.constraint.jacDiagInv;
		cp.constraint.lowerLimit = 0.0f;
		cp.constraint.upperLimit = EPX_FLT_MAX;
		cp.constraint.axis = axis;
	}
}

void epxSolveJointConstraint(
	EpxJoint &joint,
	EpxSolverBody &solverBodyA,
	EpxSolverBody &solverBodyB)
{
	EpxVector3 rA = rotate(solverBodyA.orientation,joint.anchorA);
	EpxVector3 rB = rotate(solverBodyB.orientation,joint.anchorB);

	// Linear Constraint
	epxSolveLinearConstraint(joint.jointConstraints[0].constraint,solverBodyA,solverBodyB,rA,rB);
	epxSolveLinearConstraint(joint.jointConstraints[1].constraint,solverBodyA,solverBodyB,rA,rB);
	epxSolveLinearConstraint(joint.jointConstraints[2].constraint,solverBodyA,solverBodyB,rA,rB);

	// Angular Constraint
	epxSolveAngularConstraint(joint.jointConstraints[3].constraint,solverBodyA,solverBodyB);
	epxSolveAngularConstraint(joint.jointConstraints[4].constraint,solverBodyA,solverBodyB);
	epxSolveAngularConstraint(joint.jointConstraints[5].constraint,solverBodyA,solverBodyB);
}

void epxSolveContactConstraint(
	EpxContact &contact,
	EpxSolverBody &solverBodyA,
	EpxSolverBody &solverBodyB)
{
	for(EpxUInt32 j=0;j<contact.m_numContactPoints;j++) {
		EpxContactPoint &cp = contact.m_contactPoints[j];
		EpxVector3 rA = rotate(solverBodyA.orientation,cp.pointA);
		EpxVector3 rB = rotate(solverBodyB.orientation,cp.pointB);
		
		epxSolveLinearConstraint(cp.constraints[0],solverBodyA,solverBodyB,rA,rB);
		
		EpxFloat maxFriction = contact.m_friction * fabs(cp.constraints[0].accumImpulse);
		cp.constraints[1].lowerLimit = -maxFriction;
		cp.constraints[1].upperLimit =  maxFriction;
		cp.constraints[2].lowerLimit = -maxFriction;
		cp.constraints[2].upperLimit =  maxFriction;
		
		epxSolveLinearConstraint(cp.constraints[1],solverBodyA,solverBodyB,rA,rB);
		epxSolveLinearConstraint(cp.constraints[2],solverBodyA,solverBodyB,rA,rB);
	}
	
	for(EpxUInt32 j=0;j<contact.m_numClosestPoints;j++) {
		EpxClosestPoint &cp = contact.m_closestPoints[j];
		
		EpxVector3 rA = rotate(solverBodyA.orientation,cp.pointA);
		EpxVector3 rB = rotate(solverBodyB.orientation,cp.pointB);
		
		epxSolveLinearConstraint(cp.constraint,solverBodyA,solverBodyB,rA,rB);
	}
}

} // namespace EasyPhysics
