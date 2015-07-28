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

//method declaration
void GetDesktopResolution(int& horizontal, int& vertical);

//global variables
string inputXMLPath = "./CNT_Mesh_Config.xml";
int xmlArrayLength = 300;
string temp = " ";
extern string runID;

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
	int vert = 0;
	int horiz = 0;
	GetDesktopResolution(vert, horiz);
	return glutmain(argc, argv, vert, horiz, runID.c_str(), &ccdDemo);
#endif

	//default glut doesn't return from mainloop
	return 0;
}

// Get the horizontal and vertical screen sizes in pixel
void GetDesktopResolution(int& horizontal, int& vertical)
{
	RECT desktop;
	// Get a handle to the desktop window
	const HWND hDesktop = GetDesktopWindow();
	// Get the size of screen to the variable desktop
	GetWindowRect(hDesktop, &desktop);
	// The top left corner will have coordinates (0,0)
	// and the bottom right corner will have coordinates
	// (horizontal, vertical)
	horizontal = desktop.right;
	vertical = desktop.bottom;
}