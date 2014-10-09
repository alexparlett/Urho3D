//
// Copyright (c) 2008-2014 the Urho3D project.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//

#include "Precompiled.h"
#include "Context.h"
#include "DetourCrowdManager.h"
#include "NavigationMesh.h"

#include "DebugNew.h"
#include <DetourCrowd.h>
#include "Log.h"


namespace Urho3D
{

extern const char* NAVIGATION_CATEGORY;

static const int MAX_AGENTS = 128;

Vector3 FloatToVec3(const float* f)
{
	Vector3 v;
	v.x_ = f[0];
	v.y_ = f[1];
	v.z_ = f[2];
	return v;
}

DetourCrowdManager::DetourCrowdManager(Context* context) :
    Component(context),
	maxAgents_(MAX_AGENTS),
	crowd_(0),
	navMesh_(0)
{
}

DetourCrowdManager::~DetourCrowdManager()
{
	dtFreeCrowd(crowd_);
}

void DetourCrowdManager::RegisterObject(Context* context)
{
	context->RegisterFactory<DetourCrowdManager>(NAVIGATION_CATEGORY);
}

void DetourCrowdManager::SetNavigationMesh(NavigationMesh *navMesh)
{
	navMesh_ = WeakPtr<NavigationMesh>(navMesh);
	if (navMesh_ && navMesh_->navMeshQuery_ == 0)
		navMesh_->InitializeQuery();

}

NavigationMesh* DetourCrowdManager::GetNavigationMesh()
{
	return navMesh_.Get();
}

bool DetourCrowdManager::CreateCrowd()
{
	/// \todo check if already created ?

	if (navMesh_.Expired())
		return false;
	if (navMesh_->navMesh_ == 0 )
		return false;
	if (navMesh_->navMeshQuery_ == 0)
		if (!navMesh_->InitializeQuery())
			return false;

	if (crowd_==0)
		crowd_ = dtAllocCrowd();
	
	
	// Initialize the crowd
	bool b = crowd_->init(maxAgents_, navMesh_->GetAgentRadius(), navMesh_->navMesh_);
	if (b == false)
	{
		LOGERROR("Could not initialize  DetourCrowd");
		return false;
	}

	// Make polygons with 'disabled' flag invalid.
	crowd_->getEditableFilter(0)->setExcludeFlags(NavigationPolyFlags::PolyFlags_Disabled | NavigationPolyFlags::PolyFlags_Swim);

	// Setup local avoidance params to different qualities.
	dtObstacleAvoidanceParams params;
	memcpy(&params, crowd_->getObstacleAvoidanceParams(0), sizeof(dtObstacleAvoidanceParams));

	// Low (11)
	params.velBias = 0.5f;
	params.adaptiveDivs = 5;
	params.adaptiveRings = 2;
	params.adaptiveDepth = 1;
	crowd_->setObstacleAvoidanceParams(0, &params);

	// Medium (22)
	params.velBias = 0.5f;
	params.adaptiveDivs = 5;
	params.adaptiveRings = 2;
	params.adaptiveDepth = 2;
	crowd_->setObstacleAvoidanceParams(1, &params);



	// Good (45)
	params.velBias = 0.5f;
	params.adaptiveDivs = 7;
	params.adaptiveRings = 2;
	params.adaptiveDepth = 3;
	crowd_->setObstacleAvoidanceParams(2, &params);

	// High (66)
	params.velBias = 0.5f;
	params.adaptiveDivs = 7;
	params.adaptiveRings = 3;
	params.adaptiveDepth = 3;
	crowd_->setObstacleAvoidanceParams(3, &params);

	return true;
}

int DetourCrowdManager::AddAgent(Vector3 pos, float radius, float height, float accel, float maxSpeed)
{
	if (crowd_ == 0)
		return -1;
	dtCrowdAgentParams params;
	params.radius = radius;
	params.height = height;
	params.maxAcceleration = accel;
	params.maxSpeed = maxSpeed;
	params.collisionQueryRange = params.radius * 8.0f;
	params.pathOptimizationRange = params.radius * 30.0f;
	params.updateFlags = 0
		| DT_CROWD_ANTICIPATE_TURNS
		| DT_CROWD_OPTIMIZE_VIS
		| DT_CROWD_OPTIMIZE_TOPO
		| DT_CROWD_OBSTACLE_AVOIDANCE
		//| DT_CROWD_COLLISION_RESOLUTION
		//		| DT_CROWD_AGENT_PRIORITIES
		//		| DT_CROWD_SEPARATION
		;
	params.obstacleAvoidanceType = 3;
	params.separationWeight = 2.0f;
	params.queryFilterType = 0;
	dtPolyRef polyRef;
	float nearestPos[3];
	dtStatus status = navMesh_->navMeshQuery_->findNearestPoly(
		pos.Data(),
		crowd_->getQueryExtents(),
		crowd_->getFilter(0),
		&polyRef,
		nearestPos);

	return crowd_->addAgent(nearestPos, &params);
}

void DetourCrowdManager::RemoveAgent(int agent)
{
	if (crowd_ == 0)
		return;
	crowd_->removeAgent(agent);
}

void DetourCrowdManager::UpdateAgentNavigationQuality(int agent, NavigationAvoidanceQuality nq)
{
	if (crowd_ == 0)
		return;

	dtCrowdAgentParams params = crowd_->getAgent(agent)->params;
	switch (nq)
	{
	case NAVIGATIONQUALITY_LOW:
	{							
								  params.updateFlags &= ~0
									  & ~DT_CROWD_ANTICIPATE_TURNS
									  & ~DT_CROWD_OPTIMIZE_VIS
									  & ~DT_CROWD_OPTIMIZE_TOPO
									  & ~DT_CROWD_OBSTACLE_AVOIDANCE
									  ;
	}
		break;

	case NAVIGATIONQUALITY_MED:
	{
								  params.updateFlags |= 0 ;
								  params.updateFlags &= ~0
									  & ~DT_CROWD_OBSTACLE_AVOIDANCE
									  & ~DT_CROWD_ANTICIPATE_TURNS
									  & ~DT_CROWD_OPTIMIZE_VIS
									  & ~DT_CROWD_OPTIMIZE_TOPO
									  ;
	}
		break;

	case NAVIGATIONQUALITY_HIGH:
	{
								   params.obstacleAvoidanceType = 3;
								   params.updateFlags |= 0
									   | DT_CROWD_ANTICIPATE_TURNS
									   | DT_CROWD_OPTIMIZE_VIS
									   | DT_CROWD_OPTIMIZE_TOPO
									   | DT_CROWD_OBSTACLE_AVOIDANCE
									   ;
	}
		break;
	}

	crowd_->updateAgentParameters(agent, &params);
}

void DetourCrowdManager::UpdateAgentPushiness(int agent, NavigationPushiness pushiness)
{
	if (crowd_ == 0)
		return;

	dtCrowdAgentParams params = crowd_->getAgent(agent)->params;
	switch (pushiness)
	{
	case PUSHINESS_LOW:
		params.separationWeight = 4.0f;
		params.collisionQueryRange = params.radius * 16.0f;
		break;

	case PUSHINESS_MEDIUM:
		params.separationWeight = 2.0f;
		params.collisionQueryRange = params.radius * 8.0f;
		break;

	case PUSHINESS_HIGH:
		params.separationWeight = 0.5f;
		params.collisionQueryRange = params.radius * 1.0f;
		break;
	}
	crowd_->updateAgentParameters(agent, &params);
}

void DetourCrowdManager::UpdateAgentMaxSpeed(int agent, float maxSpeed)
{
	if (crowd_ == 0)
		return;
	dtCrowdAgentParams params = crowd_->getAgent(agent)->params;
	params.maxSpeed = maxSpeed;
	crowd_->updateAgentParameters(agent, &params);
}

void DetourCrowdManager::UpdateAgentMaxAcceleration(int agent, float accel)
{
	if (crowd_ == 0)
		return;
	dtCrowdAgentParams params = crowd_->getAgent(agent)->params;
	params.maxAcceleration = accel;
	crowd_->updateAgentParameters(agent, &params);
}

bool DetourCrowdManager::SetAgentTarget(int agent, Vector3 target)
{
	if (crowd_ == 0)
		return false;
	dtPolyRef polyRef;
	float nearestPos[3];
	dtStatus status = navMesh_->navMeshQuery_->findNearestPoly(
		target.Data(),
		crowd_->getQueryExtents(),
		crowd_->getFilter(0),
		&polyRef,
		nearestPos);

	if ((status & DT_FAILURE) == 0)
	{
		if (!crowd_->requestMoveTarget(agent, polyRef, nearestPos))
		{
			// TODO: Handle Failure (Couldn't request new target.)
			return false;
		}
	}
	else
	{
		// TODO: Handle failure (Couldn't find nearest polygon.)
		return false;
	}
	return true;
}



Vector3 DetourCrowdManager::GetAgentPosition(int agent)
{
	if (crowd_ == 0)
		return Vector3();

	if (!crowd_->getAgent(agent) || !crowd_->getAgent(agent)->active)
		return Vector3();
	return FloatToVec3(crowd_->getAgent(agent)->npos);
}

Vector3 DetourCrowdManager::GetAgentCurrentVelocity(int agent)
{
	if (crowd_ == 0)
		return Vector3();
	if (!crowd_->getAgent(agent) || !crowd_->getAgent(agent)->active)
		return Vector3();
	return FloatToVec3(crowd_->getAgent(agent)->vel);
}

Vector3 DetourCrowdManager::GetAgentDesiredVelocity(int agent)
{
	if (crowd_ == 0)
		return Vector3();
	if (!crowd_->getAgent(agent) || !crowd_->getAgent(agent)->active)
		return Vector3();
	return FloatToVec3(crowd_->getAgent(agent)->dvel);
}

Vector3 DetourCrowdManager::GetClosestWalkablePosition(Vector3 pos)
{
	if (crowd_ == 0)
		return Vector3();
	float closest[3];
	const static float extents[] = { 1.0f, 20.0f, 1.0f };
	dtPolyRef closestPoly;
	dtQueryFilter filter;
	dtStatus status = navMesh_->navMeshQuery_->findNearestPoly(
		pos.Data(),
		extents,
		&filter,
		&closestPoly,
		closest);
	return FloatToVec3(closest);
}


bool DetourCrowdManager::SetMoveVelocity(int agent, const Vector3 & velocity)
{
	if (crowd_ == 0)
		return false;
	// Request that we navigate to this position.
	 if (crowd_->getAgent(agent) && crowd_->getAgent(agent)->active)
		 return crowd_->requestMoveVelocity(agent, velocity.Data());

	// Not a valid operation.
	return false;
}

void DetourCrowdManager::Update(float delta)
{
	if (crowd_ == 0)
		return;
	crowd_->update(delta, NULL);
}



}
