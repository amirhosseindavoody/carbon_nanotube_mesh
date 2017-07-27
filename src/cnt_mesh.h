#ifndef cnt_mesh_h
#define cnt_mesh_h


#include "btBulletDynamicsCommon.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h" 

#include "../misc_files/CommonInterfaces/CommonRigidBodyBase.h"

struct cnt_mesh : public CommonRigidBodyBase
{
	cnt_mesh(struct GUIHelperInterface* helper)
		:CommonRigidBodyBase(helper)
	{
	}
	virtual ~cnt_mesh(){}
	virtual void initPhysics();
	virtual void renderScene();
	
	void resetCamera()
	{
		float dist = 41;
		float pitch = -35;
		float yaw = 52;
		float targetPos[3]={0,0.46,0};
		m_guiHelper->resetCamera(dist,yaw,pitch,targetPos[0],targetPos[1],targetPos[2]);
	}


	virtual void add_tube(); // this method adds a tube to the system.
	virtual void create_container(); // this method creates an open top container for the cnts

	virtual void stepSimulation(float deltaTime)
	{
		if (m_dynamicsWorld)
		{
			m_dynamicsWorld->stepSimulation(deltaTime);
		}
	}
};


#endif //cnt_mesh_h
