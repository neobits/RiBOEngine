//
//  RigidBody.cpp
//  RiBo Engine
//
//  Created by Angelo Moro on 10/12/2019
//

#include "RigidBody.h"

//----------------------------------------------------------------------
RigidBody::RigidBody(void)
	:mass(1), 
	isStatic(false), 
	force(tkVec3::ZeroVector()),
	torque(tkVec3::ZeroVector())
{
	position = tkVec3::ZeroVector();
}
//----------------------------------------------------------------------
RigidBody::~RigidBody(void)
{
}
//----------------------------------------------------------------------
void RigidBody::SetIbodyBox(float m_a, float m_b, float m_c)
{
	a = m_a;
	b = m_b;
	c = m_c;

	Ibody(0, 0) = (mass / 12)*(b*b + c*c);
	Ibody(1, 1) = (mass / 12)*(a*a + c*c);
	Ibody(2, 2) = (mass / 12)*(a*a + b*b);

	Ibodyinv(0, 0) = (mass / 12)*(1 / (b*b + c*c));
	Ibodyinv(1, 1) = (mass / 12)*(1 / (a*a + c*c));
	Ibodyinv(2, 2) = (mass / 12)*(1 / (a*a + b*b));
}
//----------------------------------------------------------------------
void RigidBody::SetForce(const tkVec3& pforce)
{
	force = pforce;
}
//----------------------------------------------------------------------
void RigidBody::AddForce(const tkVec3& pforce)
{
	force += pforce;
}
//----------------------------------------------------------------------
void RigidBody::SetTorque(const tkVec3& ptorque)
{
	torque = ptorque;
}
//----------------------------------------------------------------------