/*****************************************************************************
* main.cpp
* Written By:
*			Alex Gabourie
*
* Creates demo application for CNT mesh modelling
******************************************************************************/

#include "stdafx.h"
#include "MeshEnv.h"
#include "GlutStuff.h"


string inputXMLPath;

int main(int argc, char *argv[])
{
	if (argc != 2)
	{
		string errMess = "Incorrect parameters. Only enter file path of config xml";
		throw std::exception(errMess.c_str());
	}
	inputXMLPath = argv[1];
	MeshEnv ccdDemo;
	ccdDemo.initPhysics(btScalar(300));
	//ccdDemo.setDebugMode(btIDebugDraw::DBG_DrawConstraints + btIDebugDraw::DBG_DrawConstraintLimits); 


#ifdef CHECK_MEMORY_LEAKS
	ccdDemo.exitPhysics();
#else
	return glutmain(argc, argv, 1910, 1010, "CNT Mesh World", &ccdDemo);
#endif

	//default glut doesn't return from mainloop
	return 0;
}

