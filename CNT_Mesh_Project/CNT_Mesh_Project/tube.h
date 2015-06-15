/*****************************************************************************
* tube.cpp
* Written By:
*			Alex Gabourie
*
* Header file for tube.cpp
******************************************************************************/

#ifndef __TUBE_H__
#define __TUBE_H__

#include "CNTFunc.h"
#include <list>
#include "btBulletDynamicsCommon.h"
#include <memory>

using namespace std;

class tube
{
	int tubeNum;
	double length;
	double cylHeight;
	double tubeSpacing;
	double minSpacing;
	shared_ptr<CNT> cnt;
	shared_ptr<std::list<btRigidBody*>> cylList;


	/*There are special ways that I want to handle this object as the btRigidBody 
	pointers in the cylList will be copies of the pointers in the dynamics world.
	I decided that it would be easiest to use all pointers and not try to use
	smart pointers as I would have to change the underlying structure of the dynamics
	engine and I do not want to do that. CNT object can use the shared pointer though
	as that is my object. 
	
	For the cylList, I need to make sure that the only remaining shared pointer is the
	one left in the tube object, so when the tube object is destroyed, I can ensure that
	all of the btRigidBody pointers are null so we don't have double deleting occuring.
	This should be the only special treatment of this object*/
public:
	tube(int num, shared_ptr<CNT> curr_cnt, double newLength, double newCylHeight, 
		double newTubeSpacing, double newMinSpacing);
	~tube();
	void addCyl(btRigidBody* body);
	shared_ptr<list<btRigidBody*>> getCylList();
	int getTubeNum();
	shared_ptr<CNT> getCNT();
	double getLength();
	double getCylHeight();
	double getTubeSpacing();
	double getMinSpacing();

};

#endif
