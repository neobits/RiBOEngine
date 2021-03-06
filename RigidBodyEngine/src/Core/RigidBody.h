//
//  RigidBody.h
//  RiBo Engine
//
//  Created by Angelo Moro on 10/12/2019
//

#pragma once

#ifndef __RIGIDBODY_H__
#define __RIGIDBODY_H__

#include "Entity.h"

class RigidBody : public tkEntity
{
public:
	RigidBody(void);
	~RigidBody(void);

	/* Sets the inertia tensor of a box (block) like object. */
	void SetIbodyBox(float m_a, float m_b, float m_c);

	void SetForce(const tkVec3& pforce);
	void AddForce(const tkVec3& pforce);
	void SetTorque(const tkVec3& ptorque);

	float a; /* Length	(X axis) */
	float b; /* Width	(Z Axis) */
	float c; /* Height	(Y Axis) */

	// Constant quantities //

	float mass;			/* M : mass of the body			*/
	tkMat3 Ibody;		/* I : Inertia tensor			*/
	tkMat3 Ibodyinv;	/* I−1 body (inverse of Ibody)	*/

	// State variables //

	tkMat3 R;			/* R(t) : orientation		*/
	tkVec3 P;			/* P(t) : linear momentum	*/
	tkVec3 L;			/* L(t) : angular momentum	*/

	// Derived quantities (auxiliary variables) //

	tkMat3 Iinv;		/* I−1(t)					*/
	tkVec3 velocity;	/* v(t) : linear velocity	*/
	tkVec3 omega;		/* ω(t) : angular velocity	*/

	// Computed quantities //

	tkVec3 force;		/* F(t) : force		*/
	tkVec3 torque;		/* τ(t) : torque	*/

	// Other interesting properties

	bool isStatic;		/* Should this unit be updated? */
};

#endif /*__RIGIDBODY_H__*/