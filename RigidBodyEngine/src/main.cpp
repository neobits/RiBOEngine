//
//  main.cpp
//  RiBo Engine
//
//  Created by Angelo Moro on 10/12/2019
//


#include "Core/Simulator.h"
#include "Core/GlutMachine.h"

using namespace std;
using namespace Core;

//-------- Application entry point --------//

int main(int argc, char **argv)
{
	tkGlutMachine *machine = new tkGlutMachine(argc, argv);
	tkSimulator *sim = new tkSimulator();
	sim->Init();
	IListener *pSim = dynamic_cast<IListener*>(sim);
	tkGlutMachine::SetListener(pSim);
	machine->InitGLUT(sim->GetFixedTime());

	tkGlutMachine::RunGLUT();

	delete pSim;
	delete sim;
	delete machine;

	return 0;
}