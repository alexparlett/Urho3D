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
#include "NavigationCrowdManager.h"
#include "NavigationMesh.h"
#include "Log.h"
#include "NavigationAgent.h"
#include "Vector.h"
#include "NavigationDebugRenderer.h"
#include "Profiler.h"
#include "DetourDebugDraw.h"
#include "DebugRenderer.h"

#include <DetourCrowd.h>

#include "DebugNew.h"
#include "SceneEvents.h"
#include "Component.h"
#include "Node.h"

namespace Urho3D
{
    extern const char* NAVIGATION_CATEGORY;

    static const int MAX_AGENTS = 1024;

    Vector3 FloatToVec3(const float* f)
    {
        Vector3 v;
        v.x_ = f[0];
        v.y_ = f[1];
        v.z_ = f[2];
        return v;
    }

    NavigationCrowdManager::NavigationCrowdManager(Context* context) :
        Component(context),
        maxAgents_(MAX_AGENTS),
        crowd_(0),
        navigationMesh_(0),
        agentDebug_(0)
    {
    }

    NavigationCrowdManager::~NavigationCrowdManager()
    {
        dtFreeCrowd(crowd_);
        if (agentDebug_)
            delete agentDebug_;
    }

    void NavigationCrowdManager::RegisterObject(Context* context)
    {
        context->RegisterFactory<NavigationCrowdManager>(NAVIGATION_CATEGORY);
    }

    void NavigationCrowdManager::SetNavigationMesh(NavigationMesh *navMesh)
    {
        navigationMesh_ = WeakPtr<NavigationMesh>(navMesh);
        if (navigationMesh_ && !navigationMesh_->navMeshQuery_)
            navigationMesh_->InitializeQuery();
    }

    NavigationMesh* NavigationCrowdManager::GetNavigationMesh()
    {
        return navigationMesh_.Get();
    }

    bool NavigationCrowdManager::CreateCrowd()
    {
        if (!navigationMesh_)
            return false;

        if (navigationMesh_.Expired())
            return false;

        if (!navigationMesh_->navMesh_)
            return false;

        if (!navigationMesh_->navMeshQuery_)
        {
            if (!navigationMesh_->InitializeQuery())
                return false;
        }

        if (!crowd_)
            crowd_ = dtAllocCrowd();

        if (!agentDebug_)
            agentDebug_ = new dtCrowdAgentDebugInfo();

        // Initialize the crowd
        if (!crowd_->init(maxAgents_, navigationMesh_->GetAgentRadius(), navigationMesh_->navMesh_))
        {
            LOGERROR("Could not initialize DetourCrowd");
            return false;
        }

        // Make polygons with 'disabled' flag invalid.
        crowd_->getEditableFilter(0)->setExcludeFlags(PolyFlags_Disabled);

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

    int NavigationCrowdManager::AddAgent(const Vector3 &pos, float maxaccel, float maxSpeed, float radius, float height, unsigned flags, unsigned avoidenceType)
    {
        if (crowd_ == 0 && navigationMesh_.Expired())
            return -1;
        dtCrowdAgentParams params;
        params.radius = radius;
        params.height = height;
        params.maxAcceleration = maxaccel;
        params.maxSpeed = maxSpeed;
        params.collisionQueryRange = params.radius * 8.0f;
        params.pathOptimizationRange = params.radius * 30.0f;
        params.updateFlags = flags;
        params.obstacleAvoidanceType = avoidenceType;
        params.separationWeight = 2.0f;
        params.queryFilterType = 0;

        dtPolyRef polyRef;
        float nearestPos[3];
        dtStatus status = navigationMesh_->navMeshQuery_->findNearestPoly(
            pos.Data(),
            crowd_->getQueryExtents(),
            crowd_->getFilter(0),
            &polyRef,
            nearestPos);

        MarkNetworkUpdate();

        return crowd_->addAgent(nearestPos, &params);
    }

    void NavigationCrowdManager::RemoveAgent(int agent)
    {
        if (crowd_ == 0)
            return;
        crowd_->removeAgent(agent);
        MarkNetworkUpdate();
    }

    void NavigationCrowdManager::UpdateAgentNavigationQuality(int agent, NavigationAvoidanceQuality nq)
    {
        if (crowd_ == 0)
            return;

        dtCrowdAgentParams params = crowd_->getAgent(agent)->params;
        switch (nq)
        {
        case NAVIGATIONQUALITY_LOW:
        {
            params.obstacleAvoidanceType = 0;
        }
            break;

        case NAVIGATIONQUALITY_MEDIUM:
        {
            params.obstacleAvoidanceType = 1;
        }
            break;

        case NAVIGATIONQUALITY_HIGH:
        {
            params.obstacleAvoidanceType = 2;
        }
            break;

        case NAVIGATIONQUALITY_EXTRA_HIGH:
        {
            params.obstacleAvoidanceType = 3;
        }
            break;
        }

        crowd_->updateAgentParameters(agent, &params);

        MarkNetworkUpdate();
    }

    void NavigationCrowdManager::UpdateAgentPushiness(int agent, NavigationPushiness pushiness)
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

        MarkNetworkUpdate();
    }

    void NavigationCrowdManager::UpdateAgentMaxSpeed(int agent, float maxSpeed)
    {
        if (crowd_ == 0)
            return;
        dtCrowdAgentParams params = crowd_->getAgent(agent)->params;
        params.maxSpeed = maxSpeed;
        crowd_->updateAgentParameters(agent, &params);

        MarkNetworkUpdate();
    }

    void NavigationCrowdManager::UpdateAgentMaxAcceleration(int agent, float accel)
    {
        if (crowd_ == 0)
            return;
        dtCrowdAgentParams params = crowd_->getAgent(agent)->params;
        params.maxAcceleration = accel;
        crowd_->updateAgentParameters(agent, &params);

        MarkNetworkUpdate();
    }

    bool NavigationCrowdManager::SetAgentTarget(int agent, Vector3 target)
    {
        if (crowd_ == 0)
            return false;
        dtPolyRef polyRef;
        float nearestPos[3];
        dtStatus status = navigationMesh_->navMeshQuery_->findNearestPoly(
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

    bool NavigationCrowdManager::SetAgentTarget(int agent, Vector3 target, unsigned int & targetRef)
    {
        if (crowd_ == 0)
            return false;
        float nearestPos[3];
        dtStatus status = navigationMesh_->navMeshQuery_->findNearestPoly(
            target.Data(),
            crowd_->getQueryExtents(),
            crowd_->getFilter(0),
            &targetRef,
            nearestPos);

        if ((status & DT_FAILURE) == 0)
        {
            if (!crowd_->requestMoveTarget(agent, targetRef, nearestPos))
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

    Vector3 NavigationCrowdManager::GetAgentPosition(int agent)
    {
        if (crowd_ == 0)
            return Vector3();

        if (!crowd_->getAgent(agent) || !crowd_->getAgent(agent)->active)
            return Vector3();
        return FloatToVec3(crowd_->getAgent(agent)->npos);
    }

    Vector3 NavigationCrowdManager::GetAgentCurrentVelocity(int agent)
    {
        if (crowd_ == 0)
            return Vector3();
        if (!crowd_->getAgent(agent) || !crowd_->getAgent(agent)->active)
            return Vector3();
        return FloatToVec3(crowd_->getAgent(agent)->vel);
    }

    Vector3 NavigationCrowdManager::GetAgentDesiredVelocity(int agent)
    {
        if (crowd_ == 0)
            return Vector3();
        if (!crowd_->getAgent(agent) || !crowd_->getAgent(agent)->active)
            return Vector3();
        return FloatToVec3(crowd_->getAgent(agent)->dvel);
    }

    Vector3 NavigationCrowdManager::GetClosestWalkablePosition(Vector3 pos)
    {
        if (crowd_ == 0)
            return Vector3();
        float closest[3];
        const static float extents[] = { 1.0f, 20.0f, 1.0f };
        dtPolyRef closestPoly;
        dtQueryFilter filter;
        dtStatus status = navigationMesh_->navMeshQuery_->findNearestPoly(
            pos.Data(),
            extents,
            &filter,
            &closestPoly,
            closest);
        return FloatToVec3(closest);
    }

    bool NavigationCrowdManager::SetMoveVelocity(int agent, const Vector3 & velocity)
    {
        if (crowd_ == 0)
            return false;
        // Request that we navigate to this position.
        if (crowd_->getAgent(agent) && crowd_->getAgent(agent)->active)
            return crowd_->requestMoveVelocity(agent, velocity.Data());

        // Not a valid operation.
        return false;
    }

    void NavigationCrowdManager::Update(float delta)
    {
        if (crowd_ == 0)
            return;

        crowd_->update(delta, agentDebug_);

        /// \todo  use this ?
        PODVector<NavigationAgent*>::Iterator it;
        for (it = agentComponents_.Begin(); it != agentComponents_.End(); it++)
        {
            (*it)->OnNavigationAgentReposition(FloatToVec3(crowd_->getAgent((*it)->GetAgentCrowdId())->npos));
        }
        /// or use events ?
        /// or use this
        /// but there is a problem if you dont use NavigationAgent components ...
        // 		dtCrowdAgent* _agents[MAX_AGENTS] = { NULL };
        // 		//memset(_agents, 0, MAX_AGENTS * sizeof(*_agents));
        // 		int count = crowd_->getActiveAgents(_agents, MAX_AGENTS);
        // 		for (int i = 0; i < count; i++)
        // 		{
        // 			if (_agents[i])
        // 			{
        // 				NavigationAgent* a = static_cast<NavigationAgent*>(_agents[i]->params.userData);
        // 				if (a)
        // 				{
        // 					a->OnNavigationAgentReposition(FloatToVec3(_agents[i]->npos));
        // 				}
        // 			}
        // 		}
    }

    const dtCrowdAgent * NavigationCrowdManager::GetCrowdAgent(int agent)
    {
        if (!crowd_)
            return NULL;
        return crowd_->getAgent(agent);
    }

    void NavigationCrowdManager::AddAgentComponent(NavigationAgent* agent)
    {
        agentComponents_.Push(agent);


        dtCrowdAgentParams params = crowd_->getEditableAgent(agentCrowdId_)->params;
        params.userData = this;

        crowd_->updateAgentParameters(agentCrowdId_, &params);
    }

    void NavigationCrowdManager::RemoveAgentComponent(NavigationAgent* agent)
    {
        agentComponents_.Remove(agent);
    }

    void NavigationCrowdManager::DrawDebug(DebugRenderer* debug, bool depthTest)
    {
        if (debug && navigationMesh_.NotNull())
        {
            PROFILE(DrawDebugNavigationCrowdManager);
            NavigationDebugRenderer dd(context_);
            dd.SetDebugRenderer(debug);
            dd.depthMask(depthTest);

            // Paths corridor polys
            // 			for (int i = 0; i < crowd_->getAgentCount(); i++)
            // 			{
            // 				const dtCrowdAgent* ag = crowd_->getAgent(i);
            // 				if (!ag->active)
            // 					continue;
            // 				const dtPolyRef* path = ag->corridor.getPath();
            // 				const int npath = ag->corridor.getPathCount();
            // 				// Draw path corridor polys
            // 				for (int j = 0; j < npath; ++j)
            // 					duDebugDrawNavMeshPoly(&dd, *navMesh_->navMesh_, path[j], duRGBA(255, 255, 255, 5));
            // 			}

            // Current position-to-target line
            for (int i = 0; i < crowd_->getAgentCount(); i++)
            {
                const dtCrowdAgent* ag = crowd_->getAgent(i);
                if (!ag->active)
                    continue;

                Color color(0.6f, 0.2f, 0.2f, 1.0f);

                // Render line to target:
                Vector3 pos1(ag->npos[0], ag->npos[1], ag->npos[2]);
                Vector3 pos2;
                for (int i = 0; i < ag->ncorners; ++i)
                {
                    pos2.x_ = ag->cornerVerts[i * 3];
                    pos2.y_ = ag->cornerVerts[i * 3 + 1];
                    pos2.z_ = ag->cornerVerts[i * 3 + 2];
                    debug->AddLine(pos1, pos2, color, depthTest);
                    pos1 = pos2;
                }
                pos2.x_ = ag->targetPos[0];
                pos2.y_ = ag->targetPos[1];
                pos2.z_ = ag->targetPos[2];
                debug->AddLine(pos1, pos2, color, depthTest);

                // Target circle
                debug->AddSphere(Sphere(pos2, 0.5f), color, depthTest);
            }

            // Velocity stuff.
            for (int i = 0; i < crowd_->getAgentCount(); ++i)
            {
                const dtCrowdAgent* ag = crowd_->getAgent(i);
                if (!ag->active) continue;

                const float radius = ag->params.radius;
                const float height = ag->params.height;
                const float* pos = ag->npos;
                const float* vel = ag->vel;
                const float* dvel = ag->dvel;

                unsigned int col = duRGBA(220, 220, 220, 192);
                if (ag->targetState == DT_CROWDAGENT_TARGET_REQUESTING || ag->targetState == DT_CROWDAGENT_TARGET_WAITING_FOR_QUEUE)
                    col = duLerpCol(col, duRGBA(128, 0, 255, 192), 32);
                else if (ag->targetState == DT_CROWDAGENT_TARGET_WAITING_FOR_PATH)
                    col = duLerpCol(col, duRGBA(128, 0, 255, 192), 128);
                else if (ag->targetState == DT_CROWDAGENT_TARGET_FAILED)
                    col = duRGBA(255, 32, 16, 192);
                else if (ag->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
                    col = duLerpCol(col, duRGBA(64, 255, 0, 192), 128);

                duDebugDrawCircle(&dd, pos[0], pos[1]/*+height*/, pos[2], radius, col, 2.0f);

                duDebugDrawArrow(&dd, pos[0], pos[1] + height, pos[2],
                    pos[0] + dvel[0], pos[1] + height + dvel[1], pos[2] + dvel[2],
                    0.0f, 0.4f, duRGBA(0, 192, 255, 192), (agentDebug_->idx == i) ? 2.0f : 1.0f);

                duDebugDrawArrow(&dd, pos[0], pos[1] + height, pos[2],
                    pos[0] + vel[0], pos[1] + height + vel[1], pos[2] + vel[2],
                    0.0f, 0.4f, duRGBA(0, 0, 0, 160), 2.0f);
            }
        }
    }

    const PODVector<NavigationAgent*>& NavigationCrowdManager::GetNavigationAgents() const
    {
        return agentComponents_;
    }

    dtCrowd * NavigationCrowdManager::GetCrowd()
    {
        return crowd_;
    }

    void NavigationCrowdManager::HandleSceneSubsystemUpdate(StringHash eventType, VariantMap& eventData)
    {
        using namespace SceneSubsystemUpdate;

        Update(eventData[P_TIMESTEP].GetFloat());
    }

    void NavigationCrowdManager::OnNodeSet(Node* node)
    {
        // Subscribe to the scene subsystem update, which will trigger the crowd update step, and grab a reference
        // to the scene's NavigationMesh
        if (node)
        {
            SubscribeToEvent(node, E_SCENESUBSYSTEMUPDATE, HANDLER(NavigationCrowdManager, HandleSceneSubsystemUpdate));
            NavigationMesh *mesh = node->GetComponent<NavigationMesh>();
            if (mesh) SetNavigationMesh(mesh);
        }
    }

}