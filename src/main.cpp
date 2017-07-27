#include <stdio.h>
#include <iostream>
#include <ctime>

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

// this is a child of DummyGUIHelper that can only return the CommonGraphicsApp pointer that is fed to it.
class LessDummyGuiHelper : public DummyGUIHelper
{
	CommonGraphicsApp* m_app;
public:
	virtual CommonGraphicsApp* getAppInterface()
	{
		return m_app;
	}

	LessDummyGuiHelper(CommonGraphicsApp* app)
		:m_app(app)
	{
	}
};




int main(int argc, char* argv[])
{

	std::clock_t begin = std::clock();

	
	// SimpleOpenGL3App is a child of CommonGraphicsApp virtual class.
	SimpleOpenGL3App* app = new SimpleOpenGL3App("Bullet Standalone Example",1024,768,true);

	prevMouseButtonCallback = app->m_window->getMouseButtonCallback();
	prevMouseMoveCallback = app->m_window->getMouseMoveCallback();

	app->m_window->setMouseButtonCallback((b3MouseButtonCallback)OnMouseDown);
	app->m_window->setMouseMoveCallback((b3MouseMoveCallback)OnMouseMove);
	
	// OpenGLGuiHelper gui(app,false); // the second argument is a dummy one
	// LessDummyGuiHelper gui(app);
	DummyGUIHelper gui;

	CommonExampleOptions options(&gui);

	// CommonExampleInterface* example;
	example = new cnt_mesh(options.m_guiHelper);
	
	example->processCommandLineArgs(argc, argv);

	example->initPhysics();
	example->create_container();
	example->add_tube();
	example->resetCamera();
	
	// std::cin.ignore();
	
	b3Clock clock;

	int step_number = 0;
	while(step_number < 1000)
	{
		step_number ++;

		app->m_instancingRenderer->init();
		app->m_instancingRenderer->updateCamera(app->getUpAxis());

		btScalar dtSec = btScalar(clock.getTimeInSeconds());
		if (dtSec<0.1)
			dtSec = 0.1;
	
		// example->stepSimulation(dtSec);

		example->stepSimulation(5);
	 	clock.reset();

		example->renderScene();
	

		// draw some grids in the space
		DrawGridData dg;
		dg.upAxis = app->getUpAxis();
		app->drawGrid(dg);
		
		app->swapBuffer();
	
		if (step_number % 50 == 0)
		{
			example->add_tube();
			std::cout << "step number: " << step_number << "\n";
			// std::cin.ignore();
		}
	}

	example->exitPhysics();
	delete example;
	delete app;

	std::clock_t end = std::clock();
	float elapsed_secs = float(end - begin) / CLOCKS_PER_SEC;
	std::cout << "run time: " << elapsed_secs << " seconds\n";

	return 0;
}

