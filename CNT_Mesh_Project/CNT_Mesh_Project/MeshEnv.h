/**
MeshEnv.h
Purpose: Header for MeshEnv.h

@author Alex Gabourie
@version 1.01 4/15/15

This is a heavily edited version of BasicDemo.h provided by Bullet Physics
*/

#ifndef MESH_ENV_H
#define MESH_ENV_H

#include "GlutDemoApplication.h"
#define PlatformDemoApplication GlutDemoApplication

#include "LinearMath/btAlignedObjectArray.h"
#include "CNTFunc.h"
#include <string>

//preprocessor function define
#define BIT(x)	(1<<(x))

//use this to create collision maps such that the spacing cylinders do not collide with
// the spine cylinders at all. In the end, we only want spacing cylinders to collide with
// their non-adjacent couterparts.
enum collisionTypes
{
	COL_NOTHING = 0, //collide with nothing
	COL_SPINE = BIT(0), //collide with spine cylinders
	COL_SPACE = BIT(1), //collide with spacing cylinders
	COL_PLANE = BIT(2), //collides with plane
	COL_BOX = BIT(3), //collides with box
	COL_EVERYTHING = 0xFFFFFFFF
};



class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;

///MeshEnv takes the basic demo and works off of it.

/**
ALL FUNCTION COMMENTS ARE IN THE CPP FILE
*/

using namespace std;

class MeshEnv : public PlatformDemoApplication
{

	//keep the collision shapes, for deletion/cleanup
	btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;

	btBroadphaseInterface*	m_broadphase;

	btCollisionDispatcher*	m_dispatcher;

	btConstraintSolver*	m_solver;

	btDefaultCollisionConfiguration* m_collisionConfiguration;

public:

	MeshEnv()
	{
	}
	virtual ~MeshEnv()
	{
		exitPhysics();
	}
	void	initPhysics();
	void	initPhysics(float camDistance);

	float	exitPhysics();

	virtual void clientMoveAndDisplay();

	virtual void displayCallback();
	virtual void	clientResetScene();

	static DemoApplication* Create()
	{
		MeshEnv* demo = new MeshEnv;
		demo->myinit();
		demo->initPhysics();
		return demo;
	}

	// Allows for multiple return values for the fixed point for tube separation calculation
	struct tubeSepResult
	{
		int iter;
		bool converge;
		long double result;

		tubeSepResult()
		{
			iter = 0;
			converge = false;
			result = 0; //converged value, true/real answer
		}
	};

	//Allows for multiple return values for fixed point for height calculation
	struct heightResult
	{
		int iter;
		bool converge;
		long double result;

		heightResult()
		{
			iter = 0;
			converge = false;
			result = 0;
		}
	};

	//relevant parameters for tube construction
	struct tubeParams
	{
		long double height;
		long double tubeSeparation;
		int numSections;

		tubeParams()
		{
			height = 0;
			tubeSeparation = 0;
			numSections = 0;
		}
	};

	///*Standard 3x3 matrix multiplication with a 3x1 vector. Needed as we have
	//types that are specific to bullet physics*/
	////@param a - 3x3 matrix
	////@param x - 3x1 vector
	////@return - a 3x1 vector that is a*x
	SIMD_FORCE_INLINE btVector3 multOperator(const btMatrix3x3& m, const btVector3& v);

	//rotates the object about the origin
	void rotateAboutOrigin(btMatrix3x3 &rotMatrix, btRigidBody* obj);

	//FIRST rotates the object about the origin  THEN shifts the object
	void rotateAndShift(btMatrix3x3 &rotMatrix, btRigidBody* obj, btVector3 &shift);

	//Shifts the object
	void shift(btRigidBody* obj, btVector3 &shift);

	//Newtons method to find tube separation
	//MeshEnv::tubeSepResult* 
	//	getTubeSeparation(btScalar height, btScalar diameter, btScalar curvature, btScalar guess);

	/*
	A couple notes: I know the curvature of the CNT will give me angle/distance. The distance is 2*height+tubeSeparation.
	I also know that the tubeSeparation is radius*tan(angle/2). Solving these two equations for height, you get some
	expression like height = atan(tubeSeparation/radius)/curvature - tubeSeparation/2. If I'm inputting a tubeSeparation,
	everything is determined easily. However, I feel like it would be more useful to fix the length of the tube sections.
	From my calculations, I see that, for the relevant chiralities, the tube separations will be about 13-15 % of the tube
	height and not a big deal. A majority of what we see will be the tube sections. The diameters of the tubes also range from
	8.1 to 10.9 angstroms. From my MATLAB plots, I will be able to get a good initial guess for the tubeSeparation needed for
	the height I want. This is somewhat manual, but it was always going to be manual. So I choose height, get tube separation

	It also appears that guessing between -2 and 2 should allow for convergence for all of the relevant nanotubes
	*/
	MeshEnv::tubeSepResult* getTubeSeparation(const shared_ptr<CNT> cnt, const double height, const double guess);

	//Gets relevant parameters for creating a nanotube
	MeshEnv::tubeParams* extractTubeParams(const shared_ptr<CNT> cnt, const double length);

	MeshEnv::heightResult* 
		getCylHeight(const shared_ptr<CNT> cnt, const double heightGuess, int const numSections, const double length);

	//converts the units of the xml doc to the units of the simulation
	double convertUnits(std::string unit, double val);

};

#endif //BASIC_DEMO_H