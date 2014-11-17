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
#include "AnimatedModel.h"
#include "Context.h"
#include "DebugRenderer.h"
#include "NavigationAgent.h"
#include "Scene.h"
#include "Component.h"
#include "Node.h"
#include "Log.h"
#include "NavigationCrowdManager.h"
#include "NavigationEvents.h"
#include "NavigationMesh.h"
#include "Profiler.h"
#include "Serializable.h"
#include "Variant.h"
#ifdef URHO3D_PHYSICS
#include "CollisionShape.h"
#endif
#include "StaticModel.h"

#include <DetourCommon.h>
#include <DetourCrowd.h>

#include "DebugNew.h"

namespace Urho3D
{

extern const char* NAVIGATION_CATEGORY;

static const float DEFAULT_AGENT_MAX_SPEED = 5.0f;
static const float DEFAULT_AGENT_MAX_ACCEL = 3.6f;
static const float DEFAULT_AGENT_HEIGHT = 2.0f;
static const float DEFAULT_AGENT_RADIUS = 0.6f;
static const NavigationAvoidanceQuality DEFAULT_AGENT_AVOIDANCE_QUALITY = NAQ_HIGH;
static const NavigationPushiness DEFAULT_AGENT_PUSHINESS = NP_MEDIUM;
static const unsigned DEFAULT_AGENT_FLAGS = NUF_ANTICIPATE_TURNS | NUF_OBSTACLE_AVOIDANCE | NUF_SEPARATION;

static const char* navigationQualityNames[] =
{
    "Low",
    "Medium",
    "High",
    "Extra",
    0
};

static const char* navigationPushinessNames[] =
{
    "Low",
    "Medium",
    "High",
    0
};

template<> NavigationAvoidanceQuality Variant::Get<NavigationAvoidanceQuality>() const
{
    return (NavigationAvoidanceQuality) GetInt();
}

template<> NavigationPushiness Variant::Get<NavigationPushiness>() const
{
    return (NavigationPushiness) GetInt();
}

template<> NavigationUpdateFlags Variant::Get<NavigationUpdateFlags>() const
{
    return (NavigationUpdateFlags)GetInt();
}

NavigationAgent::NavigationAgent(Context* context) :
    Component(context),
    inCrowd_(false),
    agentCrowdId_(-1),
    targetRef_(-1),
    targetPosition_(Vector3::ZERO),
    velocity_(Vector3::ZERO),
    updateNodePosition_(true),
    flags_(DEFAULT_AGENT_FLAGS),
    filterQuery_(0),
    maxAccel_(DEFAULT_AGENT_MAX_ACCEL),
    maxSpeed_(DEFAULT_AGENT_MAX_SPEED),
    height_(DEFAULT_AGENT_HEIGHT),
    radius_(DEFAULT_AGENT_RADIUS),
    avoidanceQuality_(DEFAULT_AGENT_AVOIDANCE_QUALITY),
    pushiness_(DEFAULT_AGENT_PUSHINESS)
{

}

NavigationAgent::~NavigationAgent()
{
}

void NavigationAgent::RegisterObject(Context* context)
{
    context->RegisterFactory<NavigationAgent>(NAVIGATION_CATEGORY);

    ATTRIBUTE(NavigationAgent, VAR_VECTOR3, "Target", targetPosition_, Vector3::ZERO, AM_DEFAULT);
    ATTRIBUTE(NavigationAgent, VAR_VECTOR3, "Velocity", velocity_, Vector3::ZERO, AM_DEFAULT | AM_NOEDIT);
    ACCESSOR_ATTRIBUTE(NavigationAgent, VAR_FLOAT, "Max Accel", GetMaxAccel, SetMaxAccel, float, DEFAULT_AGENT_MAX_ACCEL, AM_DEFAULT);
    ACCESSOR_ATTRIBUTE(NavigationAgent, VAR_FLOAT, "Max Speed", GetMaxSpeed, SetMaxSpeed, float, DEFAULT_AGENT_MAX_SPEED, AM_DEFAULT);
    ACCESSOR_ATTRIBUTE(NavigationAgent, VAR_FLOAT, "Height", GetHeight, SetHeight, float, DEFAULT_AGENT_HEIGHT, AM_DEFAULT);
    ACCESSOR_ATTRIBUTE(NavigationAgent, VAR_FLOAT, "Radius", GetRadius, SetRadius, float, DEFAULT_AGENT_RADIUS, AM_DEFAULT);
    ACCESSOR_ATTRIBUTE(NavigationAgent, VAR_INT, "Filter Query", GetFilterQuery, SetFilterQuery, int, 0, AM_DEFAULT);
    ENUM_ACCESSOR_ATTRIBUTE(NavigationAgent, "Avoidance Quality", GetNavigationAvoidanceQuality, SetNavigationAvoidanceQuality, NavigationAvoidanceQuality, navigationQualityNames, DEFAULT_AGENT_AVOIDANCE_QUALITY, AM_DEFAULT);
    ENUM_ACCESSOR_ATTRIBUTE(NavigationAgent, "Pushiness", GetNavigationPushiness, SetNavigationPushiness, NavigationPushiness, navigationPushinessNames, DEFAULT_AGENT_PUSHINESS, AM_DEFAULT);
    ACCESSOR_ATTRIBUTE(NavigationAgent, VAR_VARIANTVECTOR, "Flags", GetFlagsAttr, SetFlagsAttr, VariantVector, Variant::emptyVariantVector, AM_DEFAULT);
}

void NavigationAgent::OnNodeSet(Node* node)
{
    if (node)
    {
        Scene* scene = GetScene();
        if (scene)
        {
            if (scene == node)
            {
                LOGERROR(GetTypeName() + " should not be created to the root scene node");
                return;
            }

            crowdManager_ = scene->GetComponent<NavigationCrowdManager>();

#ifdef URHO3D_PHYSICS
            CollisionShape* cs = GetComponent<CollisionShape>();
            AnimatedModel* am = GetComponent<AnimatedModel>();
            StaticModel* sm = GetComponent<StaticModel>();
            if (cs)
            {
                height_ = cs->GetWorldBoundingBox().Size().y_;
                radius_ = Max(cs->GetWorldBoundingBox().HalfSize().x_, cs->GetWorldBoundingBox().HalfSize().z_);
            }
            else if (sm)
            {
                height_ = sm->GetWorldBoundingBox().Size().y_;
                radius_ = Max(sm->GetWorldBoundingBox().HalfSize().x_, sm->GetWorldBoundingBox().HalfSize().z_);
            }
            else if (am)
            {
                height_ = am->GetWorldBoundingBox().Size().y_;
                radius_ = Max(am->GetWorldBoundingBox().HalfSize().x_, am->GetWorldBoundingBox().HalfSize().z_);
            }
#else
            AnimatedModel* am = GetComponent<AnimatedModel>();
            StaticModel* sm = GetComponent<StaticModel>();
            if (sm)
            {
                height_ = sm->GetWorldBoundingBox().Size().y_;
                radius_ = Max(sm->GetWorldBoundingBox().HalfSize().x_, sm->GetWorldBoundingBox().HalfSize().z_);
            }
            else if (am)
            {
                height_ = am->GetWorldBoundingBox().Size().y_;
                radius_ = Max(am->GetWorldBoundingBox().HalfSize().x_, am->GetWorldBoundingBox().HalfSize().z_);
            }
#endif
            else
            {
                height_ = crowdManager_->GetNavigationMesh()->GetAgentHeight();
                radius_ = crowdManager_->GetNavigationMesh()->GetAgentRadius();
            }

            AddAgentToCrowd();
        }
        else
            LOGERROR("Node is detached from scene, can not create navigation agent.");

        node->AddListener(this);
    }
}

void NavigationAgent::ApplyAttributes()
{
    if (targetPosition_ != Vector3::ZERO)
        SetMoveTarget(targetPosition_);

    if (velocity_ != Vector3::ZERO)
        SetMoveVelocity(velocity_);
}

void NavigationAgent::OnSetEnabled()
{
    bool enabled = IsEnabledEffective();

    if (enabled && !inCrowd_)
        AddAgentToCrowd();
    else if (!enabled && inCrowd_)
        RemoveAgentFromCrowd();
}

void NavigationAgent::AddAgentToCrowd()
{
    if (!crowdManager_ || !crowdManager_->GetNavigationMesh())
        return;

    PROFILE(AddAgentToCrowd);

    if (agentCrowdId_ != -1)
        RemoveAgentFromCrowd();


    agentCrowdId_ = crowdManager_->AddAgent(node_->GetWorldPosition(), maxAccel_, maxSpeed_, radius_, height_, flags_, filterQuery_);
    if (agentCrowdId_ == -1)
    {
        inCrowd_ = false;
        LOGERROR("AddAgentToCrowd: Could not add agent to crowd!");
        return;
    }

    inCrowd_ = true;

    crowdManager_->AddAgentComponent(this);
    crowdManager_->UpdateAgentAvoidanceQuality(agentCrowdId_, avoidanceQuality_);
    crowdManager_->UpdateAgentPushiness(agentCrowdId_, pushiness_);
}

void NavigationAgent::RemoveAgentFromCrowd()
{
    if (crowdManager_ && agentCrowdId_ != -1 && inCrowd_)
    {
        crowdManager_->RemoveAgentComponent(this);
        crowdManager_->RemoveAgent(agentCrowdId_);
        inCrowd_ = false;
    }

}

bool NavigationAgent::SetMoveTarget(const Vector3& position)
{
    if (crowdManager_ && inCrowd_)
    {
        targetPosition_ = position;
        return crowdManager_->SetAgentTarget(agentCrowdId_, position, targetRef_);
    }
    return false;
}

bool NavigationAgent::SetMoveVelocity(const Vector3& velocity)
{
    if (crowdManager_ && inCrowd_)
    {		
        return crowdManager_->SetMoveVelocity(agentCrowdId_, velocity);
    }
    return false;
}

void NavigationAgent::SetMaxSpeed(float speed)
{
    maxSpeed_=speed;
    if(crowdManager_ && inCrowd_)
    {
        crowdManager_->UpdateAgentMaxSpeed(agentCrowdId_, maxSpeed_);
    }
}

void NavigationAgent::SetRadius(float radius)
{
    radius_ = radius;
    if(crowdManager_ && inCrowd_)
    {
        crowdManager_->UpdateAgentRadius(agentCrowdId_, radius_);
    }
}

void NavigationAgent::SetHeight(float height)
{
    height_ = height;
    if (crowdManager_ && inCrowd_)
    {
        crowdManager_->UpdateAgentHeight(agentCrowdId_, height_);
    }
}

void NavigationAgent::SetMaxAccel(float accel)
{
    maxAccel_ = accel;
    if (crowdManager_ && inCrowd_)
    {
        crowdManager_->UpdateAgentMaxAcceleration(agentCrowdId_, maxAccel_);
    }
}

void NavigationAgent::SetNavigationAvoidanceQuality(NavigationAvoidanceQuality val)
{
    avoidanceQuality_ = val;
    if(crowdManager_ && inCrowd_)
    {
        crowdManager_->UpdateAgentAvoidanceQuality(agentCrowdId_, avoidanceQuality_);
    }
}

void NavigationAgent::SetNavigationPushiness(NavigationPushiness val)
{
    pushiness_ = val;
    if(crowdManager_ && inCrowd_)
    {
        crowdManager_->UpdateAgentPushiness(agentCrowdId_, pushiness_);
    }
}

void NavigationAgent::SetFlags(unsigned flags)
{
    flags_ = flags;
    if (crowdManager_ && inCrowd_)
    {
        crowdManager_->UpdateAgentFlags(agentCrowdId_, flags_);
    }
}

void NavigationAgent::SetFilterQuery(int query)
{
    filterQuery_ = query;
    if (crowdManager_ && inCrowd_)
    {
        crowdManager_->UpdateAgentFilterQuery(agentCrowdId_, filterQuery_);
    }
}

Vector3 NavigationAgent::GetPosition() const
{
    if (crowdManager_ && inCrowd_)
    {
        return crowdManager_->GetAgentPosition(agentCrowdId_);
    }

    return Vector3::ZERO;
}

Vector3 NavigationAgent::GetDesiredVelocity() const
{
    if (crowdManager_ && inCrowd_)
    {
        return crowdManager_->GetAgentDesiredVelocity(agentCrowdId_);
    }
    return Vector3::ZERO;
}

Vector3 NavigationAgent::GetActualVelocity() const
{
    if (crowdManager_ && inCrowd_)
    {
        return crowdManager_->GetAgentCurrentVelocity(agentCrowdId_);
    }
    return Vector3::ZERO;
}

const Vector3& NavigationAgent::GetTargetPosition() const
{
    return targetPosition_;
}

NavigationAgentState NavigationAgent::GetAgentState() const
{
    if (crowdManager_ && inCrowd_)
    {
        const dtCrowdAgent * agent = crowdManager_->GetCrowdAgent(agentCrowdId_);
        if (!agent || !agent->active)
            return NAV_AGENT_INVALID;
        return (NavigationAgentState)agent->state;
    }
    return NAV_AGENT_INVALID;


}

NavigationTargetState NavigationAgent::GetTargetState() const
{
    if (crowdManager_ && inCrowd_)
    {
        const dtCrowdAgent* agent = crowdManager_->GetCrowdAgent(agentCrowdId_);
        if (!agent || !agent->active)
            return NAV_AGENT_TARGET_NONE;

        // Determine if we've arrived at the target
        if (agent->targetState == DT_CROWDAGENT_TARGET_VALID)
        {
            if (agent->ncorners)
            {
                // Is the agent at the end of its path?
                const bool endOfPath = (agent->cornerFlags[agent->ncorners - 1] & DT_STRAIGHTPATH_END) ? true : false;
                if (endOfPath)
                {
                    // Within its own radius of the goal?
                    if (dtVdist2D(agent->npos, &agent->cornerVerts[(agent->ncorners - 1) * 3]) <= agent->params.radius)
                        return NAV_AGENT_TARGET_ARRIVED;

                } // End if reaching end

            } // End if has path

        } // End if valid
        // Return the underlying state.
        return (NavigationTargetState)agent->targetState;
    }
    return NAV_AGENT_TARGET_NONE;
}

void NavigationAgent::SetUpdateNodePosition(bool unodepos)
{
    updateNodePosition_ = unodepos;
}

bool NavigationAgent::GetUpdateNodePosition()
{
    return updateNodePosition_;
}

void NavigationAgent::OnNavigationAgentReposition(const Vector3& newPos)
{
    if (node_)
    {
        if (updateNodePosition_)
            node_->SetWorldPosition(newPos);

        velocity_ = GetActualVelocity();

        VariantMap map = GetEventDataMap();
        map[NavigationAgentMovement::P_POSITION] = newPos;
        map[NavigationAgentMovement::P_VELOCITY] = velocity_;
        node_->SendEvent(E_NAVIGATION_AGENT_MOVEMENT, map);

        if (GetTargetState() == NAV_AGENT_TARGET_ARRIVED)
        {
            VariantMap map = GetEventDataMap();
            map[NavigationAgentReachedTarget::P_AGENT] = this;
            node_->SendEvent(E_NAVIGATION_AGENT_REACHED_TARGET, map);
        }
    }
}

void NavigationAgent::OnMarkedDirty(Node* node)
{
    Vector3 crowdPos = crowdManager_->GetAgentPosition(agentCrowdId_);
    Vector3 nodePos = node->GetPosition();

    if (crowdPos != Vector3::ZERO && crowdPos != nodePos)
        AddAgentToCrowd();
}

void NavigationAgent::SetFlagsAttr(VariantVector value)
{
    if (!value.Empty())
    {
        unsigned flags = 0;

        VariantVector::Iterator iter = value.Begin();
        for (iter; iter != value.End(); iter++)
            flags |= iter->Get<NavigationUpdateFlags>();

        SetFlags(flags);
    }
}

VariantVector NavigationAgent::GetFlagsAttr() const
{
    VariantVector value;

    if ((flags_ & NUF_ANTICIPATE_TURNS) != 0)
        value.Push(NUF_ANTICIPATE_TURNS);

    if ((flags_ & NUF_OBSTACLE_AVOIDANCE) != 0)
        value.Push(NUF_OBSTACLE_AVOIDANCE);

    if ((flags_ & NUF_SEPARATION) != 0)
        value.Push(NUF_SEPARATION);

    if ((flags_ & NUF_OPTIMIZE_VIS) != 0)
        value.Push(NUF_OPTIMIZE_VIS);

    if ((flags_ & NUF_OPTIMIZE_TOPO) != 0)
        value.Push(NUF_OPTIMIZE_TOPO);

    return value;
}

}
