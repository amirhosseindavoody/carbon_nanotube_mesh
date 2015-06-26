/**
MeshEnv.cpp
Purpose: Creates the world, objects, and camera manipulations for the CNT mesh demo

@author Alex Gabourie
@version 1.01 4/15/15

This is a heavily edited version of BasicDemo.cpp provided by Bullet Physics
*/

// Definitions:
// Spacing Cylinder - The large cylinder used to ensure a minimum distance between nanotubes.
//		Non-rendered.
// Spine Cylinder - The smaller cylinder that is used for curvature limitations and represents
//		the actual radius of the nanotubes. Rendered

#include "stdafx.h" //must be the first line of code

#define SIMD_QUARTER_PI (SIMD_PI * btScalar(0.25)) 

#include "MeshEnv.h"
#include "GlutStuff.h"
///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include "btBulletDynamicsCommon.h"

#include <stdio.h> //printf debugging
#include "GLDebugDrawer.h"
#include "LinearMath/btAabbUtil2.h"
#include "time.h"
#include "CNTFunc.h"
#include "rapidxml.hpp"
#include "rapidxml_utils.hpp"
#include <iostream>
#include <sstream>
#include "windows.h"

using namespace rapidxml;
using namespace std;
static GLDebugDrawer gDebugDraw;
//Externs are from CNT_Mesh_Main.cpp
extern string inputXMLPath;
extern string temp;
extern int xmlArrayLength;
string outputPath;
bool toDebugDraw = true;

/**
Multiplies two btMatrix3x3 matricies together. m*v

@param m First matrix
@param v Second Matrix
@return The resulting matrix
*/
SIMD_FORCE_INLINE btVector3 
MeshEnv::multOperator(const btMatrix3x3& m, const btVector3& v)
{
	return btVector3(m[0].dot(v), m[1].dot(v), m[2].dot(v));
}

/**
Custom collision filter callback to ensure spacing cylinders don't collide with each other
*/
struct AGCustomFilterCallback : public btOverlapFilterCallback
{
	//const after function name means that the function will not change any of the member variables of the
	// class/struct. In this case, there are no member variables to change, so this is true.
	//return true when pairs need collision

	/**
	Adding additional checks to see if collision detections will be added to rest of collision handling for
	spacing cylinders.

	@param proxy0 First object
	@param proxy1 Second object
	@return True if need collision, false otherwise
	*/
	virtual bool needBroadphaseCollision(btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1) const 
	{
		//lines to still implement the collision mask filtering completed before
		// obj0 collides with obj1 and vice versa
		bool collides = (proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask) != 0;
		collides = collides && (proxy0->m_collisionFilterMask & proxy1->m_collisionFilterGroup);

		//additional logic to handle space-space collisions

		//if they are both spacing cylinders
		if ((proxy0->m_collisionFilterGroup == proxy1->m_collisionFilterGroup) 
									&& proxy0->m_collisionFilterGroup == COL_SPACE)
		{
			//check their spine index and their virtibrae index
			// HACK: CNT program only uses collisionObjects in proxies. Will not work if others used.
			btCollisionObject* obj0 = static_cast<btCollisionObject*>(proxy0->m_clientObject);
			btCollisionObject* obj1 = static_cast<btCollisionObject*>(proxy1->m_clientObject);

			//Check if same tube
			if ((obj0->getSpine() == obj1->getSpine()) && obj0->getSpine()!=0)
			{
				int vertebraeDiff = abs(obj0->getVertebrae() - obj1->getVertebrae());
				//check if adjacent vertebrae
				if (vertebraeDiff == 1)
				{
					collides = false;
				}
				
			}
		}

		return collides;
	}
};

/**
Rotates the given rigid body object about the origin as the rotation matrix specifies

@param rotMatrix Matrix that specifies rotation transformation
@param obj Rigid body to be rotated
@return void
*/
void MeshEnv::rotateAboutOrigin(btMatrix3x3 &rotMatrix, btRigidBody* obj)
{
	btMatrix3x3 basis = obj->getWorldTransform().getBasis();
	btVector3 origin = obj->getWorldTransform().getOrigin();
	basis *= rotMatrix;
	origin = multOperator(rotMatrix, origin);
	obj->getWorldTransform().setBasis(basis);
	obj->getWorldTransform().setOrigin(origin);

}

/**
Rotates the given rigid body object about the origin as the rotation matrix specifies

@param rotMatrix Matrix that specifies rotation transformation
@param obj Rigid body to be rotated
@param shift The amount the object should be translated
@return void
*/
void MeshEnv::rotateAndShift(btMatrix3x3 &rotMatrix, btRigidBody* obj, btVector3 &shift)
{
	rotateAboutOrigin(rotMatrix, obj);
	btTransform* objTransform = &obj->getWorldTransform();
	objTransform->setOrigin(objTransform->getOrigin() + shift);
}

/**
Shifts a rigid body

@param obj The object to be shifted
@param shift The amount to be shifted
@return void
*/
void MeshEnv::shift(btRigidBody* obj, btVector3& shift)
{
	btTransform* objTransform = &obj->getWorldTransform();
	objTransform->setOrigin(objTransform->getOrigin() + shift);
}

/**
Calculates all of the necessary parameters for the simulation to build correct CNTs

@param cnt The cnt object that holds all of the CNT's physical properties
@param length The desired length of the tube
@return tubeParams A struct that holds relevant tube building parameters. (cylHeight, cylSpacing, etc)
*/
MeshEnv::tubeParams* MeshEnv::extractTubeParams(const shared_ptr<CNT> cnt, const double length)
{
	double a_min = 20.0; //Angstroms
	double a = a_min;
	double range = a_min*1.5;
	if (length < a_min)
	{
		throw new runtime_error(string("Desired length must be greater than 2 nm.\n"));
	}

	double steps = 1000;
	double stepSize = range / steps;

	double prevDec = 1;
	double prevVal = 0;
	int numSections = 0;
	//t_a increases monotonically with a, so we will not have any unexpected results

	//finds number of tubes at an approximate height
	while (a < a_min + range)
	{
		tubeSepResult* tubeSepRes = getTubeSeparation(cnt, a, btScalar(0.));
		//checks to make sure calculation went alright
		if (!tubeSepRes->converge)
		{
			MeshEnv::exitPhysics();
			delete tubeSepRes;
			throw new runtime_error(string("Tube separation calculation did not converge.\n"));
		}
		double t_a = tubeSepRes->result;
		delete tubeSepRes;

		double val = (length + t_a) / (a + t_a);
		double currDec = val - floor(val);
		if (currDec > prevDec)
		{
			numSections = floor(prevVal);
			break;
		}
		prevDec = currDec;
		prevVal = val;
		a += stepSize;
	}

	heightResult* heightInfo = nullptr; 
	try
	{
		heightInfo = getCylHeight(cnt, a, numSections, length);
	}
	catch (runtime_error err)
	{
		cout << err.what();
		cout << "\n";
		system("pause");
		exit(EXIT_FAILURE);
	}
	//check if height calc went alright -> This is currently a double check
	if (!heightInfo->converge)
	{
		MeshEnv::exitPhysics();
		delete heightInfo;
		throw new runtime_error(string("Section height calculation did not converge.\n"));
	}
	tubeParams* returnParams = new tubeParams();
	returnParams->height = heightInfo->result;
	delete heightInfo;
	
	tubeSepResult* tubeSepFinal = getTubeSeparation(cnt, returnParams->height, btScalar(0.));
	//checks to make sure calculation went alright
	if (!tubeSepFinal->converge)
	{
		MeshEnv::exitPhysics();
		delete tubeSepFinal;
		throw new runtime_error(string("Tube separation calculation did not converge.\n"));
	}
	returnParams->tubeSeparation = tubeSepFinal->result;
	delete tubeSepFinal;
	
	returnParams->numSections = numSections;

	return returnParams;
}

/**
Calculates the required cylinder height based on multiple parameters

@param cnt The cnt object that holds all of the CNT's physical properties
@param heightGuess The guess for newton's fixed point method
@param numSections The number of sections to be used to make the length of the tube correct
@param length Desired length of the tube
@return heightResult The object that contains the height of each cylinder and some other method information
*/
MeshEnv::heightResult* 
MeshEnv::getCylHeight(const shared_ptr<CNT> cnt, const double heightGuess, int const numSections, const double length)
{
	double tol = pow(10, -10);
	int iterlim = 500;
	MeshEnv::heightResult* result = new MeshEnv::heightResult();
	double a = heightGuess;
	double h = .001;

	for (int i = 0; i < iterlim && !result->converge; i++)
	{
		tubeSepResult* t = getTubeSeparation(cnt, a, btScalar(0.));
		tubeSepResult* tplus = getTubeSeparation(cnt, a+h, btScalar(0.));
		tubeSepResult* tminus = getTubeSeparation(cnt, a-h, btScalar(0.));
		//checks to make sure calculation went alright
		if (!(t->converge && tplus->converge && tminus->converge))
		{
			MeshEnv::exitPhysics();
			delete t;
			throw new runtime_error(string("Tube separation calculation did not converge"));
		}
		double t_a = t->result;
		double t_ap = tplus->result;
		double t_am = tminus->result;
		delete t;
		delete tplus;
		delete tminus;

		double faplus = (length + t_ap) / ((a + h) + t_ap) - float(numSections);
		double faminus = (length + t_am) / ((a - h) + t_am) - float(numSections);
		double fprime = (faplus - faminus) / (2 * h);
		double anew = a - ((length + t_a) / (a + t_a) - float(numSections)) / fprime;

		if (abs(anew - a) < tol)
		{
			result->converge = true;
			result->result = anew;
			result->iter = i;
		}
		a = anew;
	}
	return result;
}

/**
Gets the necessary separation between two cylinders to get the correct length

@param cnt The cnt object that holds all of the CNT's physical properties
@param height Height of the cylinder
@param guess The guess for newton's fixed point method
@return tubeSepResult The tube separation number and some other call specific stats
*/
MeshEnv::tubeSepResult* MeshEnv::getTubeSeparation(const shared_ptr<CNT> cnt, const double height, const double guess)
{
	double tol = pow(10, -10);
	int iterlim = 500;

	MeshEnv::tubeSepResult* result = new MeshEnv::tubeSepResult();
	double x = guess;

	double radius = cnt->getDiameter() / 2.0;

	for (int i = 0; i < iterlim && !result->converge; i++)
	{
		double xnew = x - (atan(x / (2.0*radius)) / cnt->getCurvature() - x / 2 - height) /
			((1 / cnt->getCurvature())*(radius*2.0) / (pow(x, 2) + 4*pow(radius, 2)) - .5);
		if (abs(xnew - x) < tol)
		{
			result->converge = true;
			result->iter = i;
			result->result = xnew;
		}
		x = xnew;
	}
	return result;
}

/**
Converts numbers with some units to angstroms

@param unit The current unit
@param val The current value
@return the value in angstroms
*/
double MeshEnv::convertUnits(string unit, double val)
{
	if (unit.compare("mm") == 0 || unit.compare("millimeter") == 0)
	{
		return val*10000000;
	}
	else if (unit.compare("um") == 0 || unit.compare("micrometer") == 0)
	{
		return val*10000;
	}
	else if (unit.compare("nm") == 0 || unit.compare("nanometer") == 0)
	{
		return val * 10;
	}
	else if (unit.compare("pm") == 0 || unit.compare("picometer") == 0)
	{
		return val * .01;
	}
	else if (unit.compare("A") == 0 || unit.compare("angstrom") == 0)
	{
		return val;
	}
	else
	{
		return 0; // no proper units provided
	}
}

/**
Overloaded initPhysics method that simply passes a predetermined camera distance to
initPhysics(float camDistance)
*/
void	MeshEnv::initPhysics()
{
	MeshEnv::initPhysics(btScalar(100.));
}

/**
Initializes the physics world with all of the carbon nanotubes as specified by the input XML.

@param camDistance The initial camera distance
*/
void	MeshEnv::initPhysics(float camDistance)
{

	//////////////////// ENVIRONMENT PARAMETERS /////////////////////////////////////////////
	setTexturing(true);
	setShadows(true);
	setCameraDistance(camDistance);

	//// XML Parameter Instantiation ////
	string outputFolderPath = "";
	int numTubes = 5;
	btScalar friction = 1.0;
	btScalar gravity = -9.81;
	btScalar minSpacing = 1.5;
	btScalar lmin = 100;
	btScalar lmax = 200;
	btScalar xdim = 150.0;
	btScalar ydim = 100.0;
	btScalar zdim = 150.0;
	vector<int> chirality(0);
	int chirCount = 0;

	bool done = false;
	
	while (!done){
		try{
			xml_document<> doc; //create xml object
			file<> xmlFile(inputXMLPath.c_str()); //open file
			doc.parse<0>(xmlFile.data()); //parse contents of file
			xml_node<>* currNode = doc.first_node(); //gets the node "Document" or the root node
			currNode = currNode->first_node(); //OUTPUT FOLDER
			outputFolderPath = currNode->value();
			currNode = currNode->next_sibling(); //NUMTUBES NODE
			numTubes = atoi(currNode->value());
			currNode = currNode->next_sibling(); //FRICTION NODE
			friction = atof(currNode->value());
			currNode = currNode->next_sibling(); //GRAVITY NODE
			gravity = atof(currNode->value());

			// SPACING NODE //
			currNode = currNode->next_sibling();
			minSpacing = convertUnits(string(currNode->first_node()->value()),
				atof(currNode->first_node()->next_sibling()->value())) / 2.0;
			// END SPACING NODE //

			// LENGTHS NODE //
			currNode = currNode->next_sibling();
			lmin = convertUnits(string(currNode->first_node()->value()),
				atof(currNode->first_node()->next_sibling()->value()));
			lmax = convertUnits(string(currNode->first_node()->value()),
				atof(currNode->first_node()->next_sibling()->next_sibling()->value()));
			// END LENGTHS NODE //

			// DEVICE DIMENSIONS NODE //
			currNode = currNode->next_sibling();
			xdim = convertUnits(string(currNode->first_node()->value()),
				atof(currNode->first_node()->next_sibling()->value())) / 2;
			ydim = convertUnits(string(currNode->first_node()->value()),
				atof(currNode->first_node()->next_sibling()->next_sibling()->value())) / 2;
			zdim = convertUnits(string(currNode->first_node()->value()),
				atof(currNode->first_node()->next_sibling()->next_sibling()->next_sibling()->value())) / 2;
			// END DEVICE DIMENSIONS NODE //

			// CHIRALITY NODE //
			currNode = currNode->next_sibling();
			for (xml_node<>* child = currNode->first_node(); child; child = child->next_sibling())
			{
				chirCount++;
			}
			currNode = currNode->first_node(); //Go to first cnt
			for (int i = 0; i < chirCount; i++)
			{
				chirality.insert(chirality.begin() + 2 * i, atoi(currNode->first_node()->value()));
				chirality.insert(chirality.begin() + 2 * i + 1, atoi(currNode->first_node()->next_sibling()->value()));
				currNode = currNode->next_sibling();
			}
			delete currNode;
			done = true;
		}
		catch (runtime_error err)
		{
			cout << err.what();
			cout << "\n";
			cout << "Continue? [y/n]: ";
			cin >> temp;
			if (temp.compare("y") != 0)
			{
				system("pause");
				exit(EXIT_FAILURE);
			}
			char *inputXMLPathArray = new char[xmlArrayLength];
			system("cls");
			cout << "Enter config xml path (Example in program files directory):\n";
			cin.ignore();
			cin.getline(inputXMLPathArray, xmlArrayLength);
			inputXMLPath = inputXMLPathArray;
			delete inputXMLPathArray;
		}
	}

	string timeStamp = "Date_";
	string response;
	{
		time_t timer;
		struct tm currTime;
		if (time(&timer) != -1)
		{
			errno_t err = localtime_s(&currTime, &timer);
			if (err)
			{
				printf("Invalid argument to localtime.\n");
				system("pause");
				exit(EXIT_FAILURE);
			}
		}

		timeStamp = operator+(timeStamp, to_string(currTime.tm_mday));
		timeStamp = operator+(timeStamp, ".");
		timeStamp = operator+(timeStamp, to_string(currTime.tm_mon + 1));
		timeStamp = operator+(timeStamp, ".");
		timeStamp = operator+(timeStamp, to_string(currTime.tm_year % 100));
		timeStamp = operator+(timeStamp, "_Time_");
		timeStamp = operator+(timeStamp, to_string(currTime.tm_hour));
		timeStamp = operator+(timeStamp, ".");
		timeStamp = operator+(timeStamp, to_string(currTime.tm_min));
		timeStamp = operator+(timeStamp, ".");
		timeStamp = operator+(timeStamp, to_string(currTime.tm_sec));
		timeStamp = operator+(timeStamp, "/");
	}

	outputPath = operator+(outputFolderPath, timeStamp);
	wstring wide_string(outputPath.begin(), outputPath.end());
	if (CreateDirectory(wide_string.c_str(), NULL) == 0)
	{
		auto error = GetLastError();
		if (error == ERROR_ALREADY_EXISTS)
		{
			printf("Output folder already exists. Continue anyways? [y/n]\n");
			cin >> response;
			if (response.compare(string("y")) != 0)
			{
				exit(EXIT_FAILURE);
			}
		}
		else if (error == ERROR_PATH_NOT_FOUND)
		{
			printf("Invalid output folder path.\n");
			system("pause");
			exit(EXIT_FAILURE);
		}
	}

	/////////////////////////////// END OF ENVIRONMENT PARAMETERS //////////////////////////////////

	///collision configuration contains default setup for memory, collision setup
	m_collisionConfiguration = new btDefaultCollisionConfiguration();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);

	m_broadphase = new btDbvtBroadphase();

	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	btSequentialImpulseConstraintSolver* sol = new btSequentialImpulseConstraintSolver;
	m_solver = sol;

	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);
	m_dynamicsWorld->setDebugDrawer(&gDebugDraw);
	gDebugDraw.setDebugMode(1024);

	//setting up the new filter callback for non-adjacent spacing cylinder collisions
	btOverlapFilterCallback* filterCallback = new AGCustomFilterCallback();
	m_dynamicsWorld->getPairCache()->setOverlapFilterCallback(filterCallback);

	m_dynamicsWorld->setGravity(btVector3(0, gravity, 0));

	///create a few basic rigid bodies
	//This is the ground plane box, made the same way as in Blender: sizes are from center of object to outside
	//btScalar gndDim = btScalar(25.);
	btScalar planeHlfExtThick = btScalar(.5); //half extent thickness of plane
	btBoxShape* groundShape = new btBoxShape(btVector3(xdim, planeHlfExtThick , zdim));

	groundShape->setMargin(.0);
	//adding the ground shape to the aligned object array for easy cleanup later
	m_collisionShapes.push_back(groundShape);
	btTransform planeTransform;
	planeTransform.setIdentity();
	planeTransform.setOrigin(btVector3(0, -planeHlfExtThick, 0));
	btScalar planeMass = 0.;

	//creates the necessary parts of an object and adds the object to the dynamics world
	//  #not sure what to do with handling the memory of the rigid body, the dynamics world
	//  # might take care of it becuase it has all of the pointers in the world

	int planeCollidesWith = COL_EVERYTHING & ~COL_PLANE;

	//I created this overloaded method to allow for the collision masking
	btRigidBody* body = localCreateRigidBody(planeMass, planeTransform, groundShape, COL_PLANE, planeCollidesWith);
	body->setFriction(friction);
	
	//
	//Create Walls of Simulation
	//
	planeTransform.setOrigin(btVector3(0, 0, 0));

	//x wall plane
	btBoxShape* xWallShape = new btBoxShape(btVector3(planeHlfExtThick, ydim, zdim));
	xWallShape->setMargin(0.);
	m_collisionShapes.push_back(xWallShape);

	//+x plane
	body = localCreateRigidBody(planeMass, planeTransform, xWallShape, COL_PLANE, planeCollidesWith);
	//btMatrix3x3 planeRot = btMatrix3x3(btQuaternion(btVector3(0, 0, 1), SIMD_HALF_PI));
	btVector3 planeShift(xdim + planeHlfExtThick, ydim, 0);
	shift(body, planeShift);
	body->setDrawable(FALSE);

	//-x plane
	body = localCreateRigidBody(planeMass, planeTransform, xWallShape, COL_PLANE, planeCollidesWith);
	planeShift = btVector3(-(xdim + planeHlfExtThick), ydim, 0);
	shift(body, planeShift);
	body->setDrawable(false);

	//z wall plane
	btBoxShape* zWallShape = new btBoxShape(btVector3(xdim, ydim, planeHlfExtThick));
	zWallShape->setMargin(0.);
	m_collisionShapes.push_back(zWallShape);

	//+z plane
	body = localCreateRigidBody(planeMass, planeTransform, zWallShape, COL_PLANE, planeCollidesWith);
	//planeRot = btMatrix3x3(btQuaternion(btVector3(1, 0, 0), SIMD_HALF_PI));
	planeShift = btVector3(0, ydim, zdim + planeHlfExtThick);
	shift(body, planeShift);
	body->setDrawable(FALSE);

	//-z plane
	body = localCreateRigidBody(planeMass, planeTransform, zWallShape, COL_PLANE, planeCollidesWith);
	planeShift = btVector3(0, ydim, -(zdim + planeHlfExtThick));
	shift(body, planeShift);
	body->setDrawable(false);
	
	//
	//Create Funnel Flaps
	//
	btScalar flapLen = lmax/(2*sqrt(2));
	planeTransform.setOrigin(btVector3(0, 0, 0));
	
	//x flaps
	btBoxShape* xFlapShape = new btBoxShape(btVector3(flapLen, planeHlfExtThick, zdim + 2*flapLen));
	xFlapShape->setMargin(0.);
	m_collisionShapes.push_back(xFlapShape);

	//+x flap
	body = localCreateRigidBody(planeMass, planeTransform, xFlapShape, COL_PLANE, planeCollidesWith);
	btMatrix3x3 planeRot = btMatrix3x3(btQuaternion(btVector3(0, 0, 1), SIMD_QUARTER_PI));
	planeShift = btVector3(xdim + (sqrt(2) / 2)*(flapLen + planeHlfExtThick), 2*ydim + (sqrt(2) / 2)*(flapLen - planeHlfExtThick), 0);
	rotateAndShift(planeRot, body, planeShift);
	body->setDrawable(false);

	//-x flap
	body = localCreateRigidBody(planeMass, planeTransform, xFlapShape, COL_PLANE, planeCollidesWith);
	planeRot = btMatrix3x3(btQuaternion(btVector3(0, 0, 1), -SIMD_QUARTER_PI));
	planeShift = btVector3(-(xdim + (sqrt(2) / 2)*(flapLen + planeHlfExtThick)), 2 * ydim + (sqrt(2) / 2)*(flapLen - planeHlfExtThick), 0);
	rotateAndShift(planeRot, body, planeShift);
	body->setDrawable(false);


	//z flaps
	btBoxShape* zFlapShape = new btBoxShape(btVector3(xdim + 2*flapLen, planeHlfExtThick, flapLen));
	zFlapShape->setMargin(0.);
	m_collisionShapes.push_back(zFlapShape);

	//+z flap
	body = localCreateRigidBody(planeMass, planeTransform, zFlapShape, COL_PLANE, planeCollidesWith);
	planeRot = btMatrix3x3(btQuaternion(btVector3(1, 0, 0), -SIMD_QUARTER_PI));
	//planeRot = planeRot.operator*=(btMatrix3x3(btQuaternion(btVector3(0, 1, 0), SIMD_HALF_PI)));
	planeShift = btVector3(0, 2 * ydim + (sqrt(2) / 2)*(flapLen - planeHlfExtThick), zdim + (sqrt(2) / 2)*(flapLen + planeHlfExtThick));
	rotateAndShift(planeRot, body, planeShift);
	body->setDrawable(false);

	//-z flap
	body = localCreateRigidBody(planeMass, planeTransform, zFlapShape, COL_PLANE, planeCollidesWith);
	planeRot = btMatrix3x3(btQuaternion(btVector3(1, 0, 0), SIMD_QUARTER_PI));
	//planeRot = planeRot.operator*=(btMatrix3x3(btQuaternion(btVector3(0, 1, 0), SIMD_HALF_PI)));
	planeShift = btVector3(0, 2 * ydim + (sqrt(2) / 2)*(flapLen - planeHlfExtThick), -(zdim + (sqrt(2) / 2)*(flapLen + planeHlfExtThick)));
	rotateAndShift(planeRot, body, planeShift);
	body->setDrawable(false);
	
	//////////////////////////////////////////////////////////////////////////////////////////////

	//////////////////////// Nanotube Creation //////////////////////////////////////////////////
	


	//Collision mask
	int spineCollidesWith = COL_SPINE | COL_PLANE | COL_BOX;
	int spacingCollidesWith = COL_SPACE |COL_PLANE | COL_BOX;

	//Initialize random number generation
	time_t seconds; //variable to hold seconds on clock
	time(&seconds); //assign time from clock
	//Get the random numbers up and running
	std::srand(static_cast<unsigned int>(seconds));
	for (int i = 0; i < 6; i++)
	{
		#pragma warning(suppress: 6031)
		rand();
	}

	//
	// NANOTUBE CREATION LOOP
	//
	for (int i = 1; i <= numTubes; i++)
	{
		//
		// Choose Nanotubes
		//
		int chiralityIdx = (rand() % chirCount) * 2; //random int 0 to chirCount-1
		//want to delete these pointers in the output object
		shared_ptr<CNT>curr_cnt(new CNT(chirality.at(chiralityIdx), chirality.at(chiralityIdx+1)));
		//shared_ptr<CNT>curr_cnt(new CNT(5, 5));


		//
		// Physical Nanotube Parameters
		//
		double lrange = lmax - lmin;
		double radius = curr_cnt->getDiameter() / 2; //radius of the tube
		double length = (static_cast<double>(rand()) / static_cast<double>(RAND_MAX))*lrange + lmin; //A
		tubeParams* currTubeParam = nullptr;
		try{
			currTubeParam = extractTubeParams(curr_cnt, length);
		} catch (runtime_error err)
		{
			cout << err.what();
			cout << "\n";
			system("pause");
			exit(EXIT_FAILURE);

		}
		double height = currTubeParam->height;
		double tubeSeparation = currTubeParam->tubeSeparation;
		int numSection = currTubeParam->numSections;
		delete currTubeParam;

	
		//
		// create tube
		//
		shared_ptr<tube>curr_tube(new tube(i, curr_cnt,length,height, tubeSeparation, minSpacing));
		m_tubeList.push_back(curr_tube);

		//NOTE: USE PLANE COORDINATES (AS VIEWED IN WIREFRAME MODE) FOR ALL REFERENCES
		
		//
		// Pre-Build Parameter Calculations
		//
		double cMass = 12.0 / 1000.0; //[g/mol]
		double avagadro = 6.0221413*pow(10, 23);
		double cylMass = (height + tubeSeparation) / curr_cnt->getTvecLen() * curr_cnt->getNumAtomsUnitCell()*cMass / avagadro;
		//calculate the extra height needed to get above the top of the funnel
		double funHeight = 2 * ydim + sqrt(2)*flapLen;
		//location of the first cylinder, and the prev cylinder in the for loop below
		double prevYPos = height / 2;
		/*Each tube will refered to as a spine made up of individual vertebrae. Each tube will have a spine
		number. Each cylinder in that spine will have a member field specifying the spine number as well.
		Each cylinder also represents a vertebrae in the spine. Each vertebrae will have an index as well.
		These numbers are necessary for broadphase collision filtering.
		*/
		double currYPos = prevYPos; // center of the next cylinder in the chain. To be calculated.
		double constraintShift = (height + tubeSeparation) / 2;
		//Determines actual tube length after convergence methods
		double tubeLength = (numSection - 1)*(tubeSeparation + height) + height;
		
		//Random Placement limits for tubes at the top of the funnel
		double funPad = 0; //extra padding for tube placement in funnel
		double xFunLimit = xdim + sqrt(2)*flapLen - tubeLength / 2 - funPad;
		double zFunLimit = zdim + sqrt(2)*flapLen - tubeLength / 2 - funPad;
		

		//Non-random positions
		/*btScalar xpos = 0;
		btScalar zpos = 0;
		btScalar theta = 0;*/
		//
		//Random number generation
		//
		btScalar xpos = (static_cast<double>(rand()) / static_cast<double>(RAND_MAX))*xFunLimit*2. - xFunLimit;
		btScalar zpos = (static_cast<double>(rand()) / static_cast<double>(RAND_MAX))*zFunLimit*2. - zFunLimit;
		btScalar theta = (static_cast<double>(rand()) / static_cast<double>(RAND_MAX))*SIMD_PI - SIMD_PI/2.;
		
		
		btVector3 shiftToOrigin(0,tubeLength / 2.,0);
		btMatrix3x3 rotMatrix = btMatrix3x3(btQuaternion(btVector3(0, 1, 0), theta));
		rotMatrix.operator*=(btMatrix3x3(btQuaternion(btVector3(0, 0, 1), btScalar(-SIMD_PI / 2.))));
		shiftToOrigin = -multOperator(rotMatrix, shiftToOrigin);
		//btVector3 shift = btVector3(xpos, radius + minSeparation + 2 * (radius + minSeparation) * (i - 1), zpos) + shiftToOrigin; //ground test tubes
		btVector3 shift = btVector3(xpos, funHeight + radius + minSpacing + 2*(radius + minSpacing) * (i - 1), zpos) + shiftToOrigin;


		/// Create Dynamic Objects
		btTransform startTransform;
		//Set the basis to the identity matrix which is a basic Euler matrix. See ms to need to be called before
		// trying to do any sort of transforms.
		startTransform.setIdentity();
		//after basis is set, setting origin makes more sense


		//Create the first cylinder manually and then the rest programmatically
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance
		// NOTE: To get the correct aabb for the cylinders, you must define radius for both x 
		// and z components (as this is a cylinder aligned around the y axis)
		btCollisionShape* spineShape = new btCylinderShape(btVector3(radius, prevYPos, radius));
		//Only need to push back each collision shape not each object with a collision shape
		m_collisionShapes.push_back(spineShape);
		//startTransform will be used for all subsequent created objects
		startTransform.setOrigin(btVector3(0, prevYPos, 0));
		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
		btScalar	spineMass(cylMass*.9); //mass of the object
		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (spineMass != 0.f);
		btVector3 localInertiaSpine(0, 0, 0);
		if (isDynamic)
			spineShape->calculateLocalInertia(spineMass, localInertiaSpine);
		//Must not reuse this as the Motion state must change for each cylinder
		btRigidBody::btRigidBodyConstructionInfo rbInfoSpine(spineMass, myMotionState, spineShape, localInertiaSpine);
		btRigidBody* prevSpineCyl = new btRigidBody(rbInfoSpine);
		//rotate the object accordingly
		rotateAndShift(rotMatrix, prevSpineCyl,shift);
		//Add cylinder to world
		m_dynamicsWorld->addRigidBody(prevSpineCyl, COL_SPINE, spineCollidesWith); //Spine cylinder1
		//set spine
		prevSpineCyl->setSpine(i);
		curr_tube->addCyl(prevSpineCyl);



		//SPACING CYLINDER code - reuse as many pointers from spine cylinder
		//startTransform.setOrigin(btVector3(0, prevYPos, 0)); //start transform should be same as spine cylinder
		btScalar spaceMass = cylMass*.1; //mass of the spacing cylinder
		myMotionState = new btDefaultMotionState(startTransform);
		btCylinderShape* spaceShape = 
			new btCylinderShape(btVector3(radius + minSpacing, prevYPos+.01, radius + minSpacing));
		m_collisionShapes.push_back(spaceShape);
		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		isDynamic = (spaceMass != 0.f);
		btVector3 localInertiaSpace(0, 0, 0);
		if (isDynamic)
			spaceShape->calculateLocalInertia(spaceMass, localInertiaSpace);
		//Must not reuse this as the Motion state must change for each cylinder
		btRigidBody::btRigidBodyConstructionInfo rbInfoSpace(spaceMass, myMotionState, spaceShape, localInertiaSpace);
		btRigidBody* currSpaceCyl = new btRigidBody(rbInfoSpace);
		currSpaceCyl->setFriction(friction);
		//body is active forever. Might want to look at WANT_TO_SLEEP later to symbolize end of simulation
		//currSpaceCyl->setActivationState(DISABLE_DEACTIVATION);
		currSpaceCyl->setDrawable(false);
		rotateAndShift(rotMatrix, currSpaceCyl, shift);
		m_dynamicsWorld->addRigidBody(currSpaceCyl, COL_SPACE, spacingCollidesWith);
		//spine & vertebrae for first spacing cylinder
		currSpaceCyl->setSpine(i);
		currSpaceCyl->setVertebrae(1);
		
		//some data to build constraint frames for all subsequent spine-space cylinders
		btVector3 axisA(0.f, 1.f, 0.f);
		btVector3 axisB(0.f, 1.f, 0.f);
		btVector3 pivotA(0.f, 0.f, 0.f);
		btVector3 pivotB(0.f, 0.f, 0.f);

		//Constraint for first cylinder
		//had issues calling this with typed constraint
		btHingeConstraint* hinge = 
			new btHingeConstraint(*prevSpineCyl, *currSpaceCyl, pivotA, pivotB, axisA, axisB);
		hinge->setBreakingImpulseThreshold(btScalar(BT_LARGE_FLOAT));
		hinge->setLimit(0, 0);
		m_dynamicsWorld->addConstraint(hinge); 


		//loop to create the rest of the chain that is the nanotube
		for (int j = 2; j <= numSection; j++)
		{
			//
			//Common parameters
			//

			currYPos += (height + tubeSeparation); //get its height
			startTransform.setOrigin(btVector3(0, currYPos, 0)); //set its height


			//
			//ith spine cylinder
			//
			myMotionState = new btDefaultMotionState(startTransform);
			btRigidBody::btRigidBodyConstructionInfo rbInfoSpine(spineMass, myMotionState, spineShape, localInertiaSpine);
			btRigidBody* currSpineCyl = new btRigidBody(rbInfoSpine);
			//Rotations on rigid body
			rotateAndShift(rotMatrix, currSpineCyl,shift);
			//Add spine cylinder to world
			m_dynamicsWorld->addRigidBody(currSpineCyl, COL_SPINE, spineCollidesWith);
			currSpineCyl->setSpine(i);
			curr_tube->addCyl(currSpineCyl);


			//
			//Point-2-point constraint
			//

			//add (i-1)th p2p constraint
			btVector3 pivotInA(0, constraintShift, 0);
			btVector3 pivotInB(0, -constraintShift, 0);
			//set p2p constraint to be on tube axis and halfway between the two
			btTypedConstraint* p2p = new btPoint2PointConstraint(*prevSpineCyl, *currSpineCyl, pivotInA, pivotInB);
			//set constraint to max value so there is no breaking
			p2p->setBreakingImpulseThreshold(btScalar(BT_LARGE_FLOAT));
			p2p->setDbgDrawSize(btScalar(2.f));
			m_dynamicsWorld->addConstraint(p2p);

			//
			//ith space cylinder
			//

			myMotionState = new btDefaultMotionState(startTransform);//shares same startTrans with spine cylinder
			btRigidBody::btRigidBodyConstructionInfo rbInfoSpace(spaceMass, myMotionState, spaceShape, localInertiaSpace);
			btRigidBody* currSpaceCyl = new btRigidBody(rbInfoSpace);
			currSpaceCyl->setFriction(friction);
			m_dynamicsWorld->addRigidBody(currSpaceCyl, COL_SPACE, spacingCollidesWith);
			currSpaceCyl->setDrawable(false);
			//Rotations on rigid body
			rotateAndShift(rotMatrix, currSpaceCyl, shift);
			currSpaceCyl->setSpine(i);
			currSpaceCyl->setVertebrae(j);

			//
			//Hinge constraint
			//
			hinge = new btHingeConstraint(*currSpineCyl, *currSpaceCyl, pivotA, pivotB, axisA, axisB);
			hinge->setBreakingImpulseThreshold(btScalar(BT_LARGE_FLOAT));
			hinge->setLimit(0, 0);
			m_dynamicsWorld->addConstraint(hinge);

			//
			//Update spine cylinder pointer
			//

			//set current to previous for next loop
			prevSpineCyl = currSpineCyl;

		}

	
	}
	//////////////////////////////////////////////////////////////////////////////////////////
	MeshEnv::keyboardCallback('D', 0, 0); //starts without rendering anything
}

///The MyOverlapCallback is used to show how to collect object that overlap with a given bounding box defined by aabbMin and aabbMax. 
///See m_dynamicsWorld->getBroadphase()->aabbTest.
struct	MyOverlapCallback : public btBroadphaseAabbCallback
{
	btVector3 m_queryAabbMin;
	btVector3 m_queryAabbMax;

	int m_numOverlap;
	MyOverlapCallback(const btVector3& aabbMin, const btVector3& aabbMax) : m_queryAabbMin(aabbMin), m_queryAabbMax(aabbMax), m_numOverlap(0)	{}
	virtual bool	process(const btBroadphaseProxy* proxy)
	{
		btVector3 proxyAabbMin, proxyAabbMax;
		btCollisionObject* colObj0 = (btCollisionObject*)proxy->m_clientObject;
		colObj0->getCollisionShape()->getAabb(colObj0->getWorldTransform(), proxyAabbMin, proxyAabbMax);
		if (TestAabbAgainstAabb2(proxyAabbMin, proxyAabbMax, m_queryAabbMin, m_queryAabbMax))
		{
			m_numOverlap++;
		}
		return true;
	}
};


uint8_t checkCntr = 0x00;
uint8_t bitMask = 0x07;
/**
steps the simulation, then it renders the dynamics world. Also responsible for checking if the 
simulation is done and saving the data.
*/
void MeshEnv::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//simple dynamics world doesn't handle fixed-time-stepping
	float ms = getDeltaTimeMicroseconds();

	///step the simulation
	if (m_dynamicsWorld)
	{
		m_dynamicsWorld->stepSimulation(ms / 1000000.f); ///////////////STEP SIMULATION////////////
		if ((checkCntr&bitMask) == 0){
			/*each 8th step, I would like to check if simulation is done*/
			bool simComplete = 1;
			//iterate over the list of tubes while we think that the simulation might be done
			for (list<shared_ptr<tube>>::iterator itrTube = m_tubeList.begin();
				itrTube != m_tubeList.end() && simComplete; ++itrTube)
			{
				shared_ptr<list<btRigidBody*>> tempCylList = (*itrTube)->getCylList();
				int sleepCntr = 0;
				for (list<btRigidBody*>::iterator itrCyl = tempCylList->begin(); itrCyl != tempCylList->end(); ++itrCyl)
				{
					if ((*itrCyl)->getActivationState() == ISLAND_SLEEPING ||
						(*itrCyl)->getActivationState() == WANTS_DEACTIVATION)
					{
						sleepCntr++;
					}
				}
				//if 10% of the cylinders are ready to or are sleeping, then the tube is done.
				if (float(sleepCntr) / float(tempCylList->size()) < .1)
				{
					simComplete = 0;
				}
			}

			if (simComplete)
			{
				//iterate for file output
				//string folder = "C:/Users/Gabory/Dropbox/Research/OutputFiles/";
				for (list<shared_ptr<tube>>::iterator itrTube = m_tubeList.begin();
					itrTube != m_tubeList.end() && simComplete; ++itrTube)
				{
					//create file with some filename
					int tubeNum = (*itrTube)->getTubeNum();
					stringstream filePathMaker;
					filePathMaker << outputPath << "CNT_Num_" << tubeNum << ".csv";
					string fileName = filePathMaker.str();
					ofstream file;
					file.open(fileName);
					// add initial info to file
					file << "Chirality:,";
					file << (*itrTube)->getCNT()->getn();
					file << "  ";
					file << (*itrTube)->getCNT()->getm();
					file << "\n";
					file << "Length:,";
					file << (*itrTube)->getLength();
					file << "\n";
					file << "Cylinder Height:,";
					file << (*itrTube)->getCylHeight();
					file << "\n";
					file << "Intertube Spacing:,";
					file << (*itrTube)->getMinSpacing();
					file << "\n";
					file << "Intercylinder Spacing:,";
					file << (*itrTube)->getTubeSpacing();
					file << "\n";
					file << "x";
					file << tubeNum;
					file << ",y";
					file << tubeNum;
					file << ",z";
					file << tubeNum;
					file << "\n";

					shared_ptr<list<btRigidBody*>> tempCylList = (*itrTube)->getCylList();
					for (list<btRigidBody*>::iterator itrCyl = tempCylList->begin(); itrCyl != tempCylList->end(); ++itrCyl)
					{
						btVector3 pos = (*itrCyl)->getCenterOfMassPosition();
						file << pos[0];
						file << ",";
						file << pos[1];
						file << ",";
						file << pos[2];
						file << "\n";
					}
					file.close();
				}
				exitPhysics();
				printf("Mesh created successfully!\n");
				exit(EXIT_SUCCESS);
			}

		}
		checkCntr++;
		//optional but useful: debug drawing
		if (toDebugDraw){
			m_dynamicsWorld->debugDrawWorld();
		}
	}

	//Rendering function. Uses
	renderme();

	//APIENTRY functions -> cannot edit
	glFlush();
	swapBuffers();
}

/**
renders the dynamics world as it stands currently
*/
void MeshEnv::displayCallback(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	renderme();

	//optional but useful: debug drawing to detect problems
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	glFlush();
	swapBuffers();
}

/**
Exits the current simulation and starts over
*/
void	MeshEnv::clientResetScene()
{
	initPhysics(exitPhysics());
}

/**
Exits the current simulation, cleaning up all data
*/
float	MeshEnv::exitPhysics()
{

	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	int i;
	for (i = m_dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject(obj);
		delete obj;
	}

	//delete collision shapes
	for (int j = 0; j<m_collisionShapes.size(); j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}
	m_collisionShapes.clear();

	delete m_dynamicsWorld;

	delete m_solver;

	delete m_broadphase;

	delete m_dispatcher;

	delete m_collisionConfiguration;

	return getCameraDistance();
}