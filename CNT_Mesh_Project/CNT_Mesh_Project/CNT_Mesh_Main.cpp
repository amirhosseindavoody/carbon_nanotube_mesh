/*****************************************************************************
* main.cpp
* Written By:
*			Alex Gabourie
*
* Creates demo application for CNT mesh modelling
******************************************************************************/

#include "stdafx.h"
#include "BasicTubeDemo.h"
#include "GlutStuff.h"
#include "btBulletDynamicsCommon.h"
//#include "CNTFunc.h"

int _tmain(int argc, char** argv)
{
	//CNT x1 = CNT(7, 5);
	//CNT x2 = CNT(7, 6);
	//CNT x3 = CNT(8, 6);
	//CNT x4 = CNT(8, 7);
	//CNT x5 = CNT(9, 7);

	BasicTubeDemo ccdDemo;
	ccdDemo.initPhysics(btScalar(300));
	ccdDemo.setDebugMode(btIDebugDraw::DBG_DrawConstraints + btIDebugDraw::DBG_DrawConstraintLimits);


#ifdef CHECK_MEMORY_LEAKS
	ccdDemo.exitPhysics();
#else
	return glutmain(argc, argv, 1910, 1010, "Bullet Physics Demo. http://bulletphysics.org", &ccdDemo);
#endif

	//default glut doesn't return from mainloop
	return 0;
}

