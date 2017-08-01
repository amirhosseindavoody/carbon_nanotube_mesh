#include <stdio.h>
#include <iostream>
#include <ctime>
#include <array>

#include "../misc_files/CommonInterfaces/CommonExampleInterface.h"
#include "../misc_files/CommonInterfaces/CommonGUIHelperInterface.h"
#include "../misc_files/Utils/b3Clock.h"

#include "../misc_files/OpenGLWindow/SimpleOpenGL3App.h"
#include "../misc_files/ExampleBrowser/OpenGLGuiHelper.h"

#include "cnt_mesh.h"


// this block of code and the global variable and function is used for handling mouse input and
// moving objects via mouse. you can comment it if this capability is not needed any more.
//*************************************************************************************************
// CommonExampleInterface*    example;
cnt_mesh*    example;
int gSharedMemoryKey=-1;

b3MouseMoveCallback prevMouseMoveCallback = 0;
static void OnMouseMove( float x, float y)
{
	bool handled = false; 
	handled = example->mouseMoveCallback(x,y); 	 
	if (!handled)
	{
		if (prevMouseMoveCallback)
			prevMouseMoveCallback (x,y);
	}
}

b3MouseButtonCallback prevMouseButtonCallback  = 0;
static void OnMouseDown(int button, int state, float x, float y) {
	bool handled = false;

	handled = example->mouseButtonCallback(button, state, x,y); 
	if (!handled)
	{
		if (prevMouseButtonCallback )
			prevMouseButtonCallback (button,state,x,y);
	}
}
//*************************************************************************************************

int main(int argc, char* argv[])
{
	// print the start time and start recording the run time
	std::clock_t start = std::clock();
	std::time_t start_time = std::time(nullptr);
	std::cout << std::endl << "start time:" << std::endl << std::asctime(std::localtime(&start_time)) << std::endl;

	SimpleOpenGL3App* app;
	GUIHelperInterface* gui;

	// flag to let the graphic visualization happen
	const bool visualize = false;
	
	// SimpleOpenGL3App is a child of CommonGraphicsApp virtual class.
	app = new SimpleOpenGL3App("carbon nanotube mesh",1024,768,true);

	prevMouseButtonCallback = app->m_window->getMouseButtonCallback();
	prevMouseMoveCallback = app->m_window->getMouseMoveCallback();

	app->m_window->setMouseButtonCallback((b3MouseButtonCallback)OnMouseDown);
	app->m_window->setMouseMoveCallback((b3MouseMoveCallback)OnMouseMove);
	
	gui = new OpenGLGuiHelper(app,false); // the second argument is a dummy one
	// 	gui = new DummyGUIHelper();

	CommonExampleOptions options(gui);

	// CommonExampleInterface* example;
	example = new cnt_mesh(options.m_guiHelper);
	
	example->processCommandLineArgs(argc, argv);

	example->initPhysics();
	example->create_container(20.,20.);

	if (visualize)
	{
		example->resetCamera();
	}
	
	int step_number = 0;

	while(example->num_tubes() < 200)
	{
		step_number ++;
	
		btScalar dtSec = 0.05;
		example->stepSimulation(dtSec);

		if (step_number % 50 == 0)
		{
			example->add_tube(10, 1., 0.5);
			example->freeze_tube(10);
			std::cout << "number of tubes: " << example->num_tubes() << "\n";
			// std::cin.ignore();
		}

		if (visualize)
		{
			app->m_instancingRenderer->init();
			app->m_instancingRenderer->updateCamera(app->getUpAxis());
			example->renderScene();
			
			// draw some grids in the space
			DrawGridData dg;
			dg.upAxis = app->getUpAxis();
			app->drawGrid(dg);
			
			app->swapBuffer();
		}
	}


	if (not visualize)
	{
		gui = new OpenGLGuiHelper(app,false); // the second argument is a dummy one

		example->resetCamera();
		app->m_instancingRenderer->init();
		app->m_instancingRenderer->updateCamera(app->getUpAxis());
		example->renderScene();
		
		// draw some grids in the space
		DrawGridData dg;
		dg.upAxis = app->getUpAxis();
		app->drawGrid(dg);
		
		app->swapBuffer();

	}
	
	// print the end time and the runtime
	std::clock_t end = std::clock();
	std::time_t end_time = std::time(nullptr);
	std::cout << std::endl << "end time:" << std::endl << std::asctime(std::localtime(&end_time));
	std::cout << "runtime: " << std::difftime(end_time,start_time) << " seconds" << std::endl << std::endl;
	
	std::cin.ignore();

	example->exitPhysics();
	delete example;
	delete app;


	return 0;
}

