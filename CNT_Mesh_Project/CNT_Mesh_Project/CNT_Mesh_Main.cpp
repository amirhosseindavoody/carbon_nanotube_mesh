/**
CNT_Mesh_Main.cpp
Purpose: Initializes and starts CNT physics simulation.

@author Alex Gabourie
@version 1.01 4/15/15

This is a heavily edited version of main.cpp in the BasicDemo example provided by Bullet Physics
*/

#include "stdafx.h"
#include "MeshEnv.h"
#include "GlutStuff.h"
#include <iostream>


string inputXMLPath = "./CNT_Mesh_Config.xml";
int xmlArrayLength = 300;
string temp = " ";
extern string timeStamp;

int main(int argc, char *argv[])
{
	if (argc == 1)
	{
		char *inputXMLPathArray = new char[xmlArrayLength];
		cout << "Enter config xml path (Example in program files directory):\n";
		cin.getline(inputXMLPathArray, xmlArrayLength);
		inputXMLPath = inputXMLPathArray;
		delete[] inputXMLPathArray;
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


#ifdef CHECK_MEMORY_LEAKS
	ccdDemo.exitPhysics();
#else
	return glutmain(argc, argv, 1910, 1010, timeStamp.c_str(), &ccdDemo);
#endif

	//default glut doesn't return from mainloop
	return 0;
}

