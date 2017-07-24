#include <stdio.h>
#include <iostream>

#include "../misc_files/CommonInterfaces/CommonExampleInterface.h"
#include "../misc_files/CommonInterfaces/CommonGUIHelperInterface.h"
#include "../misc_files/Utils/b3Clock.h"

#include "../misc_files/OpenGLWindow/SimpleOpenGL3App.h"
#include "../misc_files/ExampleBrowser/OpenGLGuiHelper.h"

#include "BasicExample.h"

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
	// std::cin.ignore();

	
	// SimpleOpenGL3App is a child of CommonGraphicsApp virtual class.
	SimpleOpenGL3App* app = new SimpleOpenGL3App("Bullet Standalone Example",1024,768,true);
	
	OpenGLGuiHelper gui(app,false); // the second argument is a dummy one
	// LessDummyGuiHelper gui(app);
	// DummyGUIHelper gui;

	CommonExampleOptions options(&gui);

	CommonExampleInterface* example = new BasicExample(options.m_guiHelper);
	example->processCommandLineArgs(argc, argv);

	example->initPhysics();
	example->resetCamera();
	
	b3Clock clock;

	do
	{
		app->m_instancingRenderer->init();
		app->m_instancingRenderer->updateCamera(app->getUpAxis());

		btScalar dtSec = btScalar(clock.getTimeInSeconds());
		if (dtSec<0.1)
			dtSec = 0.1;
	
		example->stepSimulation(dtSec);
	 	clock.reset();

		example->renderScene();
	

		// draw some grids in the space
		DrawGridData dg;
		dg.upAxis = app->getUpAxis();
		app->drawGrid(dg);
		
		app->swapBuffer();
	

	} while (!app->m_window->requestedExit());

	example->exitPhysics();
	delete example;
	delete app;
	return 0;
}

