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

#include "EpxJoint.h"
#include "../collision/EpxVectorFunction.h"

namespace EasyPhysics {

extern void epxCalcJointAngleHinge(const EpxMatrix3 &worldFrameA,const EpxMatrix3 &worldFrameB,EpxVector3 &angle,EpxMatrix3 &axis);
extern void epxCalcJointAngleSwingTwist(const EpxMatrix3 &worldFrameA,const EpxMatrix3 &worldFrameB,EpxVector3 &angle,EpxMatrix3 &axis);

void epxInitializeJointAsBallInLocal(
	EpxJoint &joint,
	EpxUInt32 rigidBodyIdA,
	EpxUInt32 rigidBodyIdB,
	const EpxVector3 &anchorA,
	const EpxVector3 &anchorB,
	const EpxMatrix3 &frameA,
	const EpxMatrix3 &frameB,
	EpxFloat bias,
	EpxFloat damping)
{
	joint.rigidBodyIdA = rigidBodyIdA;
	joint.rigidBodyIdB = rigidBodyIdB;
	joint.calcAngleFunc = epxCalcJointAngleSwingTwist;
	
	if(damping > 0.0f) {
		joint.numConstraints = 6;
	}
	else {
		joint.numConstraints = 3;
	}

	joint.jointConstraints[0].bias = bias;
	joint.jointConstraints[1].bias = bias;
	joint.jointConstraints[2].bias = bias;

	joint.jointConstraints[3].damping = damping;
	joint.jointConstraints[4].damping = damping;
	joint.jointConstraints[5].damping = damping;
	
	joint.jointConstraints[0].lockType = EpxLockTypeFix;
	joint.jointConstraints[1].lockType = EpxLockTypeFix;
	joint.jointConstraints[2].lockType = EpxLockTypeFix;
	joint.jointConstraints[3].lockType = EpxLockTypeFree;
	joint.jointConstraints[4].lockType = EpxLockTypeFree;
	joint.jointConstraints[5].lockType = EpxLockTypeFree;

	joint.anchorA = anchorA;
	joint.anchorB = anchorB;
	joint.frameA = frameA;
	joint.frameB = frameB;
}

void epxInitializeJointAsBall(
	EpxJoint &joint,
	EpxUInt32 rigidBodyIdA,
	EpxUInt32 rigidBodyIdB,
	const EpxState &stateA,
	const EpxState &stateB,
	const EpxVector3 &anchor,
	EpxFloat bias,
	EpxFloat damping)
{
	joint.reset();

	EpxMatrix3 rotA = transpose(EpxMatrix3(stateA.m_orientation));
	EpxMatrix3 rotB = transpose(EpxMatrix3(stateB.m_orientation));
	
	EpxVector3 anchorA = rotA * (anchor - stateA.m_position);
	EpxVector3 anchorB = rotB * (anchor - stateB.m_position);
	
	epxInitializeJointAsBallInLocal(joint,
		rigidBodyIdA,rigidBodyIdB,
		anchorA,anchorB,
		rotA,rotB,
		bias,damping);
}

void epxInitializeJointAsHingeInLocal(
	EpxJoint &joint,
	EpxUInt32 rigidBodyIdA,
	EpxUInt32 rigidBodyIdB,
	const EpxVector3 &anchorA,
	const EpxVector3 &anchorB,
	const EpxMatrix3 &frameA,
	const EpxMatrix3 &frameB,
	EpxFloat bias,
	EpxFloat damping,
	EpxFloat lowerAngle,
	EpxFloat upperAngle)
{
	joint.rigidBodyIdA = rigidBodyIdA;
	joint.rigidBodyIdB = rigidBodyIdB;
	joint.numConstraints = 6;
	joint.calcAngleFunc = epxCalcJointAngleHinge;

	joint.jointConstraints[3].damping = damping;
	joint.jointConstraints[3].bias = bias;
	
	joint.jointConstraints[0].lockType = EpxLockTypeFix;
	joint.jointConstraints[1].lockType = EpxLockTypeFix;
	joint.jointConstraints[2].lockType = EpxLockTypeFix;

	if(lowerAngle == 0.0f && upperAngle == 0.0f) {
		joint.jointConstraints[3].lockType = EpxLockTypeFree;
	}
	else {
		joint.jointConstraints[3].lockType = EpxLockTypeLimit;
	}
	joint.jointConstraints[4].lockType = EpxLockTypeFix;
	joint.jointConstraints[5].lockType = EpxLockTypeFix;
	
	joint.jointConstraints[3].lowerLimit = lowerAngle;
	joint.jointConstraints[3].upperLimit = upperAngle;
	
	joint.anchorA = anchorA;
	joint.anchorB = anchorB;
	joint.frameA = frameA;
	joint.frameB = frameB;
}

void epxInitializeJointAsHinge(
	EpxJoint &joint,
	EpxUInt32 rigidBodyIdA,
	EpxUInt32 rigidBodyIdB,
	const EpxState &stateA,
	const EpxState &stateB,
	const EpxVector3 &anchor,
	const EpxVector3 &direction,
	EpxFloat bias,
	EpxFloat damping,
	EpxFloat lowerAngle,
	EpxFloat upperAngle)
{
	joint.reset();
	
	EpxMatrix3 rotA = transpose(EpxMatrix3(stateA.m_orientation));
	EpxMatrix3 rotB = transpose(EpxMatrix3(stateB.m_orientation));
	
	EpxVector3 anchorA = rotA * (anchor - stateA.m_position);
	EpxVector3 anchorB = rotB * (anchor - stateB.m_position);
	
	EpxVector3 axis0, axis1, axis2;
	axis0 = normalize(direction);
	epxCalcTangentVector(axis0, axis1, axis2 );
	EpxMatrix3 frame(axis0, axis1, axis2);
	
	epxInitializeJointAsHingeInLocal(joint,
		rigidBodyIdA,rigidBodyIdB,
		anchorA,anchorB,
		rotA * frame,rotB * frame,
		bias,damping,lowerAngle,upperAngle);
}

void epxInitializeJointAsFixInLocal(
	EpxJoint &joint,
	EpxUInt32 rigidBodyIdA,
	EpxUInt32 rigidBodyIdB,
	const EpxVector3 &anchorA,
	const EpxVector3 &anchorB,
	const EpxMatrix3 &frameA,
	const EpxMatrix3 &frameB,
	EpxFloat bias)
{
	joint.rigidBodyIdA = rigidBodyIdA;
	joint.rigidBodyIdB = rigidBodyIdB;
	joint.numConstraints = 6;
	joint.calcAngleFunc = epxCalcJointAngleSwingTwist;
	
	joint.jointConstraints[0].bias = bias;
	joint.jointConstraints[1].bias = bias;
	joint.jointConstraints[2].bias = bias;
	joint.jointConstraints[3].bias = bias;
	joint.jointConstraints[4].bias = bias;
	joint.jointConstraints[5].bias = bias;
	
	joint.jointConstraints[0].lockType = EpxLockTypeFix;
	joint.jointConstraints[1].lockType = EpxLockTypeFix;
	joint.jointConstraints[2].lockType = EpxLockTypeFix;
	joint.jointConstraints[3].lockType = EpxLockTypeFix;
	joint.jointConstraints[4].lockType = EpxLockTypeFix;
	joint.jointConstraints[5].lockType = EpxLockTypeFix;

	joint.anchorA = anchorA;
	joint.anchorB = anchorB;
	joint.frameA = frameA;
	joint.frameB = frameB;
}

void epxInitializeJointAsFix(
	EpxJoint &joint,
	EpxUInt32 rigidBodyIdA,
	EpxUInt32 rigidBodyIdB,
	const EpxState &stateA,
	const EpxState &stateB,
	const EpxVector3 &anchor,
	EpxFloat bias)
{
	joint.reset();

	EpxMatrix3 rotA = transpose(EpxMatrix3(stateA.m_orientation));
	EpxMatrix3 rotB = transpose(EpxMatrix3(stateB.m_orientation));
	
	EpxVector3 anchorA = rotA * (anchor - stateA.m_position);
	EpxVector3 anchorB = rotB * (anchor - stateB.m_position);
	
	epxInitializeJointAsFixInLocal(joint,
		rigidBodyIdA,rigidBodyIdB,
		anchorA,anchorB,
		rotA,rotB,
		bias);
}

void epxInitializeJointAsSwingTwistInLocal(
	EpxJoint &joint,
	EpxUInt32 rigidBodyIdA,
	EpxUInt32 rigidBodyIdB,
	const EpxVector3 &anchorA,
	const EpxVector3 &anchorB,
	const EpxMatrix3 &frameA,
	const EpxMatrix3 &frameB,
	EpxFloat bias,
	EpxFloat damping,
	EpxFloat twistLowerAngle,
	EpxFloat twistUpperAngle,
	EpxFloat swingLowerAngle,
	EpxFloat swingUpperAngle)
{
	joint.rigidBodyIdA = rigidBodyIdA;
	joint.rigidBodyIdB = rigidBodyIdB;
	joint.numConstraints = 6;
	joint.calcAngleFunc = epxCalcJointAngleSwingTwist;
	
	joint.jointConstraints[3].damping = damping;
	joint.jointConstraints[3].bias = bias;
	joint.jointConstraints[4].damping = damping;
	joint.jointConstraints[4].bias = bias;
	joint.jointConstraints[5].damping = damping;
	joint.jointConstraints[5].bias = bias;
	
	joint.jointConstraints[0].lockType = EpxLockTypeFix;
	joint.jointConstraints[1].lockType = EpxLockTypeFix;
	joint.jointConstraints[2].lockType = EpxLockTypeFix;
	
	joint.jointConstraints[3].lockType = EpxLockTypeLimit;
	joint.jointConstraints[4].lockType = EpxLockTypeLimit;
	joint.jointConstraints[5].lockType = EpxLockTypeFree;
	
	joint.jointConstraints[3].lowerLimit = twistLowerAngle;
	joint.jointConstraints[3].upperLimit = twistUpperAngle;
	joint.jointConstraints[4].lowerLimit = swingLowerAngle;
	joint.jointConstraints[4].upperLimit = swingUpperAngle;
	
	joint.anchorA = anchorA;
	joint.anchorB = anchorB;
	joint.frameA = frameA;
	joint.frameB = frameB;
}

void epxInitializeJointAsSwingTwist(
	EpxJoint &joint,
	EpxUInt32 rigidBodyIdA,
	EpxUInt32 rigidBodyIdB,
	const EpxState &stateA,
	const EpxState &stateB,
	const EpxVector3 &anchor,
	const EpxVector3 &direction,
	EpxFloat bias,
	EpxFloat damping,
	EpxFloat twistLowerAngle,
	EpxFloat twistUpperAngle,
	EpxFloat swingLowerAngle,
	EpxFloat swingUpperAngle)
{
	joint.reset();
	
	EpxMatrix3 rotA = transpose(EpxMatrix3(stateA.m_orientation));
	EpxMatrix3 rotB = transpose(EpxMatrix3(stateB.m_orientation));
	
	EpxVector3 anchorA = rotA * (anchor - stateA.m_position);
	EpxVector3 anchorB = rotB * (anchor - stateB.m_position);
	
	EpxVector3 axis0, axis1, axis2;
	axis0 = normalize(direction);
	epxCalcTangentVector(axis0, axis1, axis2 );
	EpxMatrix3 frame(axis0, axis1, axis2);
	
	epxInitializeJointAsSwingTwistInLocal(joint,
		rigidBodyIdA,rigidBodyIdB,
		anchorA,anchorB,
		rotA * frame,rotB * frame,
		bias,damping,
		twistLowerAngle,twistUpperAngle,
		swingLowerAngle,swingUpperAngle);
}

void epxInitializeJointAsSliderInLocal(
	EpxJoint &joint,
	EpxUInt32 rigidBodyIdA,
	EpxUInt32 rigidBodyIdB,
	const EpxVector3 &anchorA,
	const EpxVector3 &anchorB,
	const EpxMatrix3 &frameA,
	const EpxMatrix3 &frameB,
	EpxFloat bias,
	EpxFloat damping,
	EpxFloat lowerDistance,
	EpxFloat upperDistance)
{
	joint.rigidBodyIdA = rigidBodyIdA;
	joint.rigidBodyIdB = rigidBodyIdB;
	joint.numConstraints = 6;
	joint.calcAngleFunc = epxCalcJointAngleSwingTwist;
	
	joint.jointConstraints[0].bias = bias;
	joint.jointConstraints[0].damping = damping;
	
	if(lowerDistance == 0.0f && upperDistance == 0.0f) {
		joint.jointConstraints[0].lockType = EpxLockTypeFree;
	}
	else {
		joint.jointConstraints[0].lockType = EpxLockTypeLimit;
	}
	
	joint.jointConstraints[1].lockType = EpxLockTypeFix;
	joint.jointConstraints[2].lockType = EpxLockTypeFix;
	joint.jointConstraints[3].lockType = EpxLockTypeFix;
	joint.jointConstraints[4].lockType = EpxLockTypeFix;
	joint.jointConstraints[5].lockType = EpxLockTypeFix;

	joint.jointConstraints[0].lowerLimit = lowerDistance;
	joint.jointConstraints[0].upperLimit = upperDistance;

	joint.anchorA = anchorA;
	joint.anchorB = anchorB;
	joint.frameA = frameA;
	joint.frameB = frameB;
}

void epxInitializeJointAsSlider(
	EpxJoint &joint,
	EpxUInt32 rigidBodyIdA,
	EpxUInt32 rigidBodyIdB,
	const EpxState &stateA,
	const EpxState &stateB,
	const EpxVector3 &anchor,
	const EpxVector3 &direction,
	EpxFloat bias,
	EpxFloat damping,
	EpxFloat lowerDistance,
	EpxFloat upperDistance)
{
	joint.reset();
	
	EpxMatrix3 rotA = transpose(EpxMatrix3(stateA.m_orientation));
	EpxMatrix3 rotB = transpose(EpxMatrix3(stateB.m_orientation));
	
	EpxVector3 anchorA = rotA * (anchor - stateA.m_position);
	EpxVector3 anchorB = rotB * (anchor - stateB.m_position);
	
	EpxVector3 axis0, axis1, axis2;
	axis0 = normalize(direction);
	epxCalcTangentVector(axis0, axis1, axis2 );
	EpxMatrix3 frame(axis0, axis1, axis2);

	epxInitializeJointAsSliderInLocal(joint,
		rigidBodyIdA,rigidBodyIdB,
		anchorA,anchorB,
		rotA * frame,rotB * frame,
		bias,damping,
		lowerDistance,upperDistance);

}

} // namespace EasyPhysics
