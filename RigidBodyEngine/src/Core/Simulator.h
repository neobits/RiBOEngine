//
//  Simulator.h
//  RiBo Engine
//
//  Created by Angelo Moro on 10/12/2019
//

#pragma once

#ifndef __SIMULATOR_H__
#define __SIMULATOR_H__

#include <vector>
#include "IListener.h"
#include "freeglut.h"
#include "Color.h"

#define STATE_SIZE 13
#define GRAVITY tkVec3(0,-9.8f,0)

class RigidBody;

//typedef void (*dydt_func)(float t, float y[], float ydot[]);
//void ode(float y0[], float yend[], int len, float t0,float t1, dydt_func dydt);

namespace Core
{
	class tkSimulator : public IListener
	{
	public:
		tkSimulator(void);
		~tkSimulator(void);

		void Init();
		void Render();
		void Update();
		void KeyPressed(unsigned char key, int x, int y);

		/* Copy the state information into an array */
		void StateToArray(RigidBody *rb, float *y);
		/* Copy information from an array into the state variables */
		void ArrayToState(RigidBody *rb, float *y);
		void ArrayToBodies(float y[]);
		void BodiesToArray(float y[]);
		/* Compute forces applied and torque */
		void ComputeForces(RigidBody *rb);
		void dydt(float y[], float ydot[]);
		void ddt_StateToArray(RigidBody *rb, float *ydot);
		void DerivativeCalculation();
		//void ForceCalculation();
		void SystemEvolution();
		void SimulationLoop();

		inline unsigned int GetFixedTime() { return fixedTime; }

	private:
		float time;					/* Time since the simulation started */
		float it;					/* Number of iterations */
		float step;					/* Simulation step */
		unsigned int fixedTime;		/* Timer callback in milliseconds (ms) */
		float dt;					/* Delta time (between each frame) */
		int NBODIES;				/* Amount of rigid bodies to process */
		RigidBody *rBodies;
		std::vector<RigidBody> bodies;

		float *y_0;
		float *yfinal;
	};
}

#endif /*!__SIMULATOR_H__*/