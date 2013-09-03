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

#include "EpxContact.h"

#define EPX_CONTACT_SAME_POINT			0.01f
#define EPX_CONTACT_THRESHOLD_NORMAL	0.01f	// 衝突点の閾値（法線方向）
#define EPX_CONTACT_THRESHOLD_TANGENT	0.002f	// 衝突点の閾値（平面上）

namespace EasyPhysics {

static inline
EpxFloat calcArea4Points(const EpxVector3 &p0,const EpxVector3 &p1,const EpxVector3 &p2,const EpxVector3 &p3)
{
	EpxFloat areaSqrA = lengthSqr(cross(p0-p1,p2-p3));
	EpxFloat areaSqrB = lengthSqr(cross(p0-p2,p1-p3));
	EpxFloat areaSqrC = lengthSqr(cross(p0-p3,p1-p2));
	return EPX_MAX(EPX_MAX(areaSqrA,areaSqrB),areaSqrC);
}

int EpxContact::findNearestContactPoint(const EpxVector3 &newPointA,const EpxVector3 &newPointB,const EpxVector3 &newNormal)
{
	int nearestIdx = -1;
	
	EpxFloat minDiff = EPX_CONTACT_SAME_POINT;
	for(EpxUInt32 i=0;i<m_numContactPoints;i++) {
		EpxFloat diffA = lengthSqr(m_contactPoints[i].pointA - newPointA);
		EpxFloat diffB = lengthSqr(m_contactPoints[i].pointB - newPointB);
		if(diffA < minDiff && diffB < minDiff && dot(newNormal,m_contactPoints[i].normal) > 0.99f) {
			minDiff = EPX_MAX(diffA,diffB);
			nearestIdx = i;
		}
	}
	
	return nearestIdx;
}

int EpxContact::sort4ContactPoints(const EpxVector3 &newPoint,EpxFloat newDistance)
{
	int maxPenetrationIndex = -1;
	EpxFloat maxPenetration = newDistance;

	// 最も深い衝突点は排除対象からはずす
	for(EpxUInt32 i=0;i<m_numContactPoints;i++) {
		if(m_contactPoints[i].distance < maxPenetration) {
			maxPenetrationIndex = i;
			maxPenetration = m_contactPoints[i].distance;
		}
	}
	
	EpxFloat res[4] = {0.0f};
	
	// 各点を除いたときの衝突点が作る面積のうち、最も大きくなるものを選択
	EpxVector3 newp(newPoint);
	EpxVector3 p[4] = {
		m_contactPoints[0].pointA,
		m_contactPoints[1].pointA,
		m_contactPoints[2].pointA,
		m_contactPoints[3].pointA,
	};

	if(maxPenetrationIndex != 0) {
		res[0] = calcArea4Points(newp,p[1],p[2],p[3]);
	}

	if(maxPenetrationIndex != 1) {
		res[1] = calcArea4Points(newp,p[0],p[2],p[3]);
	}

	if(maxPenetrationIndex != 2) {
		res[2] = calcArea4Points(newp,p[0],p[1],p[3]);
	}

	if(maxPenetrationIndex != 3) {
		res[3] = calcArea4Points(newp,p[0],p[1],p[2]);
	}

	int maxIndex = 0;
	EpxFloat maxVal = res[0];

	if (res[1] > maxVal) {
		maxIndex = 1;
		maxVal = res[1];
	}

	if (res[2] > maxVal) {
		maxIndex = 2;
		maxVal = res[2];
	}

	if (res[3] > maxVal) {
		maxIndex = 3;
		maxVal = res[3];
	}

	return maxIndex;
}

void EpxContact::reset()
{
	m_numContactPoints = 0;
	for(int i=0;i<EPX_NUM_CONTACT_POINTS;i++) {
		m_contactPoints[i].reset();
	}
	
	m_numClosestPoints = 0;
	for(int i=0;i<EPX_NUM_CLOSEST_POINTS;i++) {
		m_closestPoints[i].reset();
	}
}

void EpxContact::removeContactPoint(int i)
{
	m_contactPoints[i] = m_contactPoints[m_numContactPoints-1];
	m_numContactPoints--;
}

void EpxContact::refresh(const EpxVector3 &pA,const EpxQuat &qA,const EpxVector3 &pB,const EpxQuat &qB)
{
	// 衝突点の更新
	// 両衝突点間の距離が閾値（CONTACT_THRESHOLD）を超えたら消去
	for(int i=0;i<(int)m_numContactPoints;i++) {
		EpxVector3 normal = m_contactPoints[i].normal;
		EpxVector3 cpA = pA + rotate(qA,m_contactPoints[i].pointA);
		EpxVector3 cpB = pB + rotate(qB,m_contactPoints[i].pointB);

		// 貫通深度がプラスに転じたかどうかをチェック
		EpxFloat distance = dot(normal,cpA - cpB);
		if(distance > EPX_CONTACT_THRESHOLD_NORMAL) {
			removeContactPoint(i);
			i--;
			continue;
		}
		m_contactPoints[i].distance = distance;

		// 深度方向を除去して両点の距離をチェック
		cpA = cpA - m_contactPoints[i].distance * normal;
		EpxFloat distanceAB = lengthSqr(cpA - cpB);
		if(distanceAB > EPX_CONTACT_THRESHOLD_TANGENT) {
			removeContactPoint(i);
			i--;
			continue;
		}
	}
}

void EpxContact::merge(const EpxContact &contact)
{
	for(EpxUInt32 i=0;i<contact.m_numContactPoints;i++) {
		EpxContactPoint &cp = m_contactPoints[i];
		
		int id = findNearestContactPoint(cp.pointA,cp.pointB,cp.normal);
		
		if(id >= 0) {
			if(fabs(dot(cp.normal,m_contactPoints[id].normal)) > 0.99f ) {
				// 同一点を発見、蓄積された情報を引き継ぐ
				m_contactPoints[id].distance = cp.distance;
				m_contactPoints[id].pointA = cp.pointA;
				m_contactPoints[id].pointB = cp.pointB;
				m_contactPoints[id].normal = cp.normal;
			}
			else {
				// 同一点ではあるが法線が違うため更新
				m_contactPoints[id] = cp;
			}
		}
		else if(m_numContactPoints < EPX_NUM_CONTACT_POINTS) {
			// 衝突点を新規追加
			m_contactPoints[m_numContactPoints++] = cp;
		}
		else {
			// ソート
			id = sort4ContactPoints(cp.pointA,cp.distance);
			
			// コンタクトポイント入れ替え
			m_contactPoints[id] = cp;
		}
	}
}

void EpxContact::addContactPoint(
	EpxFloat penetrationDepth,
	const EpxVector3 &normal,
	const EpxVector3 &contactPointA,
	const EpxVector3 &contactPointB)
{
	int id = findNearestContactPoint(contactPointA,contactPointB,normal);

	if(id < 0 && m_numContactPoints < EPX_NUM_CONTACT_POINTS) {
		// 衝突点を新規追加
		id = m_numContactPoints++;
		m_contactPoints[id].reset();
	}
	else if(id < 0){
		id = sort4ContactPoints(contactPointA,penetrationDepth);
		m_contactPoints[id].reset();
	}
	
	m_contactPoints[id].distance = penetrationDepth;
	m_contactPoints[id].pointA = contactPointA;
	m_contactPoints[id].pointB = contactPointB;
	m_contactPoints[id].normal = normal;
}

void EpxContact::addClosestPoint(
	EpxFloat distance,
	const EpxVector3 &normal,
	const EpxVector3 &closestPointA,
	const EpxVector3 &closestPointB)
{
	EpxInt32 id = -1;
	EpxFloat minDiff = EPX_CONTACT_SAME_POINT * EPX_CONTACT_SAME_POINT;
	for(EpxUInt32 i=0;i<m_numClosestPoints;i++) {
		EpxFloat diffA = lengthSqr(m_closestPoints[i].pointA - closestPointA);
		EpxFloat diffB = lengthSqr(m_closestPoints[i].pointB - closestPointB);
		
		if(diffA < minDiff && diffB < minDiff) {
			minDiff = EPX_MAX(diffA,diffB);
			id = (EpxInt32)i;
			break;
		}
	}
	
	if(id < 0 && m_numClosestPoints < EPX_NUM_CLOSEST_POINTS) {
		id = m_numClosestPoints++;
		m_closestPoints[id].distance = distance;
		m_closestPoints[id].constraint.accumImpulse = 0.0f;
		m_closestPoints[id].normal = normal;
		m_closestPoints[id].pointA = closestPointA;
		m_closestPoints[id].pointB = closestPointB;
	}
}

} // namespace EasyPhysics
