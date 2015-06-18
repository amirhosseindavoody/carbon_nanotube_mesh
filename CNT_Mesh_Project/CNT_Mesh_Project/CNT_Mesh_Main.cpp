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
#include <iostream>


string inputXMLPath = "./CNT_Mesh_Config.xml";
int xmlArrayLength = 300;
string temp = " ";

int main(int argc, char *argv[])
{
	if (argc == 1)
	{
		char *inputXMLPathArray = new char[xmlArrayLength];
		cout << "Enter config xml path (Example in program files directory):\n";
		int tempo = sizeof(*inputXMLPathArray);
		cin.getline(inputXMLPathArray, xmlArrayLength);
		inputXMLPath = inputXMLPathArray;
		delete inputXMLPathArray;
	}
	else if (argc > 2)
	{
		cout << "Incorrect parameters. Only enter file path of config xml\n";
		system("pause");
		exit(EXIT_FAILURE);
	}
	else
	{
		inputXMLPath = argv[1];
	}
	
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

