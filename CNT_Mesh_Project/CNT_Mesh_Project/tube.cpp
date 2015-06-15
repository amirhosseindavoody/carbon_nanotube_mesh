/*****************************************************************************
* tube.cpp
* Written By:
*			Alex Gabourie
*
* Stores all tube info
******************************************************************************/

#include "stdafx.h"
#include "tube.h"

using namespace std;

tube::tube(int num, shared_ptr<CNT> curr_cnt, double newLength, double newCylHeight,
	double newTubeSpacing, double newMinSpacing)
{
	tubeNum = num;
	length = newLength;
	cnt = curr_cnt;
	cylHeight = newCylHeight;
	tubeSpacing = newTubeSpacing;
	minSpacing = newMinSpacing;
	cylList = static_cast<shared_ptr<list<btRigidBody*>>>(new list<btRigidBody*>());
}

tube::~tube()
{
	// Need to go assign all of the btRigidBody objects to null as they are already deleted 
	//  elsewhere
	for (list<btRigidBody*>::iterator itr = cylList->begin(); itr != cylList->end(); ++itr)
	{
		*itr = NULL;
	}

	/* cnt and tubeNum should delete fine on their own. The list can now clean itself up.*/
}

int tube::getTubeNum()
{
	return tubeNum;
}

// Adds pointer to end of list
void tube::addCyl(btRigidBody* body)
{
	cylList->push_back(body);
}

shared_ptr<CNT> tube::getCNT()
{
	return cnt;
}

shared_ptr<std::list<btRigidBody*>> tube::getCylList()
{
	return cylList;
}

double tube::getLength()
{
	return length;
}

double tube::getCylHeight()
{
	return cylHeight;
}

double tube::getMinSpacing()
{
	return minSpacing;
}

double tube::getTubeSpacing()
{
	return tubeSpacing;
}



