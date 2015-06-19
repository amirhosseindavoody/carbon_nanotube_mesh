/**
tube.cpp
Purpose: Stores all nanotube information. Implements tube functions.

@author Alex Gabourie
@version 1.01 4/15/15
*/

#include "stdafx.h"
#include "tube.h"

using namespace std;

/**
Creates the structure responsible for remembering all bullet physics related information
for each nanotube

@param num Represents the numth tube created in the simulation. Bookkeeping number
@param curr_cnt The CNT object with all of the physical properties
@param newLength The length of the nanotube
@param newCylHeight The height of the individual cylinders
@param newTubeSpacing The spacing between the cylinders in the tube
@param newMinSpacing The distance the spacing cylinder goes out from the tube diameter
*/
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

/**
Deletes pointers for the cylinder list such that they are not double deleted later. (Sketchy, I know)
*/
tube::~tube()
{
	// Need to go assign all of the btRigidBody objects to null as they are already deleted 
	//  elsewhere. This may look like a memory leak because by setting the pointers to null,
	// the actual pointer cannot be deleted. However, there are two pointers pointing to the
	// memory locations in question and the second set of pointers handles the memory
	for (list<btRigidBody*>::iterator itr = cylList->begin(); itr != cylList->end(); ++itr)
	{
		*itr = NULL;
	}

	/* cnt and tubeNum should delete fine on their own. The list can now clean itself up.*/
}

/**
Gets the number of the tube

@return The tube's creation number
*/
int tube::getTubeNum()
{
	return tubeNum;
}

/**
Adds a pointer of the cylinder to the end of the cylList

@param body The pointer to the cylinder to be added
*/
void tube::addCyl(btRigidBody* body)
{
	cylList->push_back(body);
}

/**
Gets the CNT object

@return The CNT objec that this tube holds
*/
shared_ptr<CNT> tube::getCNT()
{
	return cnt;
}

/**
Gets the entire list of cylinders

@return A pointer to a list of rigid body object pointers
*/
shared_ptr<std::list<btRigidBody*>> tube::getCylList()
{
	return cylList;
}

/**
Gets the length of the tube

@return the length of the nanotube
*/
double tube::getLength()
{
	return length;
}

/**
Gets the height of the cylinders in the tube (all the same)

@return the cylinder height
*/
double tube::getCylHeight()
{
	return cylHeight;
}

/**
Gets the spacing between other nanotubes

@return the minimum spacing between this tube and others
*/
double tube::getMinSpacing()
{
	return minSpacing;
}

/**
Gets the spacing between adjacent cylinders within the same nanotube

@return the spacing between adjacent cylinders within the same nanotube
*/
double tube::getTubeSpacing()
{
	return tubeSpacing;
}



