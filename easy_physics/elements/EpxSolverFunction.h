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

#ifndef EPX_SOLVER_FUNCTION_H
#define EPX_SOLVER_FUNCTION_H

#include "../EpxBase.h"
#include "EpxState.h"
#include "EpxSolverBody.h"
#include "EpxContact.h"
#include "EpxJoint.h"

namespace EasyPhysics {

void epxSetupJointConstraint(
	EpxJoint &joint,
	EpxState &stateA,
	EpxState &stateB,
	EpxSolverBody &solverBodyA,
	EpxSolverBody &solverBodyB,
	EpxFloat timeStep);

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
	EpxFloat timeStep);

void epxSolveJointConstraint(
	EpxJoint &joint,
	EpxSolverBody &solverBodyA,
	EpxSolverBody &solverBodyB);

void epxSolveContactConstraint(
	EpxContact &contact,
	EpxSolverBody &solverBodyA,
	EpxSolverBody &solverBodyB);

} // namespace EasyPhysics

#endif // EPX_SOLVER_FUNCTION_H
