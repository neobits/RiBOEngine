#include "Simulator.h"
#include "RigidBody.h"
#include "maths\MathsUtils.h"

using namespace Core;

tkSimulator::tkSimulator(void) :NBODIES(2)
{
	y_0 = new float[STATE_SIZE * NBODIES];
	yfinal = new float[STATE_SIZE * NBODIES];

	rBodies = new RigidBody[NBODIES];

	// By default, t=0, h=0.01, ct=10
	time = 0.0f;
	step = 0.01f;
	ct = (int)(step * 1000);
	dt = 1 / (float)ct;
	it = 0;
}

tkSimulator::~tkSimulator(void)
{
}

void tkSimulator::Init()
{
	// Box 1
	rBodies[0].SetIbodyBox(5.0f, 1.0f, 2.0f);
	//Bodies[0].SetPosition(tkVec3(0, 10.0f, 0.0f));
	rBodies[0].position = tkVec3(0, 10.0f, 0.0f);
	//Bodies[0].R(0, 0) = 0;
	//Bodies[0].R(1, 1) = 1;
	//Bodies[0].R(2, 2) = 0;
	//colors.push_back(Color::Red());

	// Box 2
	rBodies[1].SetIbodyBox(8.0f, 4.0f, 2.0f);
	rBodies[1].isStatic = true;

	BodiesToArray(yfinal);
}

/* Copy the state information into an array */
void tkSimulator::StateToArray(RigidBody *rb, float *y)
{
	*y++ = rb->position.x;
	*y++ = rb->position.y;
	*y++ = rb->position.z;

	*y++ = rb->q.w;
	*y++ = rb->q.x;
	*y++ = rb->q.y;
	*y++ = rb->q.z;

	*y++ = rb->P.x;
	*y++ = rb->P.y;
	*y++ = rb->P.z;

	*y++ = rb->L.x;
	*y++ = rb->L.y;
	*y++ = rb->L.z;
}

/* Copy information from an array into the state variables */
void tkSimulator::ArrayToState(RigidBody *rb, float *y)
{
	rb->position.x = *y++;
	rb->position.y = *y++;
	rb->position.z = *y++;

	rb->q.w = *y++;
	rb->q.x = *y++;
	rb->q.y = *y++;
	rb->q.z = *y++;

	rb->P.x = *y++;
	rb->P.y = *y++;
	rb->P.z = *y++;

	rb->L.x = *y++;
	rb->L.y = *y++;
	rb->L.z = *y++;

	// Compute auxiliary variables... //
	// v(t) = P(t) / M 
	rb->velocity = rb->P / rb->mass;

	tkQuat q = rb->q;
	q.Normalize();
	rb->R = tkQuat::QuatToMatrix(q);

	// I^(-1)(t) = R(t).I^(-1)_body.R^T(t)
	tkMat3 RxIinv = rb->R*rb->Ibodyinv;
	rb->Iinv = RxIinv*tkMat3::Transpose(rb->R);
	// ?(t) = I^(-1)(t).L(t)
	rb->omega = rb->Iinv*rb->L;
}

void tkSimulator::ArrayToBodies(float y[])
{
	for (int i = 0; i < NBODIES; i++)
	{
		if (!rBodies[i].isStatic) {
			ArrayToState(&rBodies[i], &y[i * STATE_SIZE]);
		}
	}
}

void tkSimulator::BodiesToArray(float y[])
{
	for (int i = 0; i < NBODIES; i++) {
		StateToArray(&rBodies[i], &y[i * STATE_SIZE]);
	}
}

void tkSimulator::ComputeForces(RigidBody *rb)
{
	// Total force and torque applied to the rigid body
	tkVec3 Ftot, Ttot;

	// Gravity
	tkVec3 Fg = GRAVITY*rb->mass;
	//Ftot += Fg;

	// Add a torque to contact point pr with force Fr
	tkVec3 r(0, 0, 10.f);
	tkVec3 pr = r - rb->position;

	tkVec3 Fr(0.0f, 0.0f, 50.0f);
	Ttot += tkVec3::Cross(Fr, pr);

	rb->SetForce(Ftot);
	rb->SetTorque(Ttot);
}

void tkSimulator::dydt(float y[], float ydot[])
{
	// Put data in y[] into Bodies[]
	ArrayToBodies(y);
	for (int i = 0; i < NBODIES; i++)
	{
		ComputeForces(&rBodies[i]);
		ddt_StateToArray(&rBodies[i], &ydot[i * STATE_SIZE]);
	}
}

void tkSimulator::ddt_StateToArray(RigidBody *rb, float *ydot)
{
	// copy	d/dt x(t) = v(t) into ydot
	*ydot++ = rb->velocity.x;
	*ydot++ = rb->velocity.y;
	*ydot++ = rb->velocity.z;

	tkQuat qdot = MathUtils::VecMultQuat(rb->omega, rb->q)*0.5f;
	*ydot++ = 1; //*ydot++ = qdot.w;
	*ydot++ = qdot.x;
	*ydot++ = qdot.y;
	*ydot++ = qdot.z;

	// d/dt P(t) = F(t)
	*ydot++ = rb->force.x;
	*ydot++ = rb->force.y;
	*ydot++ = rb->force.z;
	// d/dt L(t) = ?(t)
	*ydot++ = rb->torque.x;
	*ydot++ = rb->torque.y;
	*ydot++ = rb->torque.z;
}

void tkSimulator::DerivativeCalculation()
{
	int i_state;
	float *y = new float[STATE_SIZE*NBODIES];
	float *ydot = new float[STATE_SIZE*NBODIES];

	for (i_state = 0; i_state < STATE_SIZE*NBODIES; i_state++) 
	{
		y[i_state] = y_0[i_state];
		ydot[i_state] = 0.0f;
	}

	dydt(y, ydot);

	// Solvers
	for (i_state = 0; i_state < STATE_SIZE*NBODIES; i_state++) 
	{
		y[i_state] += (ydot[i_state] * step); // ODE Euler method
	}

	for (i_state = 0; i_state < STATE_SIZE*NBODIES; i_state++) 
	{
		yfinal[i_state] = y[i_state];
	}
}

void tkSimulator::SystemEvolution()
{
	ArrayToBodies(yfinal);

	time += step;
	it++;
}

void tkSimulator::SimulationLoop()
{
	for (int i = 0; i < STATE_SIZE * NBODIES; i++) 
	{
		y_0[i] = yfinal[i];
	}
	DerivativeCalculation();
	SystemEvolution();
}

