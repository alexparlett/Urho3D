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

#pragma once

#include "Component.h"
#include "NavigationCrowdManager.h"

namespace Urho3D
{


class NavigationCrowdManager;

enum NavigationTargetState
{
    NAV_AGENT_TARGET_NONE = 0,
    NAV_AGENT_TARGET_FAILED,
    NAV_AGENT_TARGET_VALID,
    NAV_AGENT_TARGET_REQUESTING,
    NAV_AGENT_TARGET_WAITINGFORPATH,
    NAV_AGENT_TARGET_WAITINGFORQUEUE,
    NAV_AGENT_TARGET_VELOCITY,
    NAV_AGENT_TARGET_ARRIVED
};

enum NavigationAgentState
{
    NAV_AGENT_INVALID = 0,		///< The agent is not in a valid state.
    NAV_AGENT_READY,			///< The agent is traversing a normal navigation mesh polygon
    NAV_AGENT_TRAVERSINGLINK	///< The agent is traversing an off-mesh connection.
};


/// Root scene node must have an NavigationCrowdManager Component!
/// Do not create to the root scene node!
/// Agents radius and height is set through the navigation mesh.
/// 
class URHO3D_API NavigationAgent : public Component
{
    OBJECT(NavigationAgent);
    
public:
    /// Construct.
    NavigationAgent(Context* context);
    /// Destruct.
    virtual ~NavigationAgent();
    /// Register object factory.
    static void RegisterObject(Context* context);
    /// Handle enabled/disabled state change.
    virtual void OnSetEnabled();
    /// Handle attributes being applied.
    virtual void ApplyAttributes();

    /// Submits a new move request for the this agent.
    bool SetMoveTarget(const Vector3& position);
    /// Submits a new move velocity request for the this agent.
    bool SetMoveVelocity(const Vector3& velocity);
    /// Update the nodes position.
    void SetUpdateNodePosition(bool unodepos);
    /// Sets the agents max acceleration.
    void SetMaxAccel(float val);
    /// Sets the agents max velocity.
    void SetMaxSpeed(float val);
    /// Sets the agents radius.
    void SetRadius(float radius);
    /// Sets the agents height.
    void SetHeight(float height);
    /// Sets the agent's navigation quality.
    void SetNavigationAvoidanceQuality(NavigationAvoidanceQuality val);
    /// Sets the agent's navigation pushiness.
    void SetNavigationPushiness(NavigationPushiness val);
    /// Sets the agent's navigation flags.
    void SetFlags(unsigned flags);

    /// Returns the agents position.
    Vector3 GetPosition() const;
    /// Returns the agents desired velocity.
    Vector3 GetDesiredVelocity() const;
    /// Returns the agents actual velocity.
    Vector3 GetActualVelocity() const;
    /// Returns the agents target position.
    const Vector3& GetTargetPosition() const;
    /// Returns the agents  state.
    NavigationAgentState GetAgentState() const;
    /// Returns the agents target state.
    NavigationTargetState GetTargetState() const;
    /// Returns if the nodes position is updating because of the crowd.
    bool GetUpdateNodePosition();
    /// Returns the agent id.
    int GetAgentCrowdId() const	{ return agentCrowdId_; }
    /// Gets the agents max velocity.
    float GetMaxSpeed()	const { return maxSpeed_; }
    /// Gets the agents max acceleration.
    float GetMaxAccel() const { return maxAccel_; }
    /// Gets the agents radius.
    float GetRadius() const { return radius_; }
    /// Gets the agents height.
    float GetHeight() const { return height_; }
    /// Gets the agents flags.
    unsigned GetFlags() const { return flags_; }
    /// Gets the agent's navigation quality
    NavigationAvoidanceQuality GetNavigationAvoidanceQuality() const { return avoidanceQuality_; }
    /// Gets the agent's navigation pushiness
    NavigationPushiness GetNavigationPushiness() const {return pushiness_; }
    
    /// Updates the nodes position if updateNodePosition is set. Is called in NavigationCrowdManager::Update().
    virtual void OnNavigationAgentReposition(const Vector3& newPos);

    void SetFlagsAttr(VariantVector value);
    VariantVector GetFlagsAttr() const;

protected:
    /// Handle node being assigned.
    virtual void OnNodeSet(Node* node);
    /// Handle node transform being dirtied.
    virtual void OnMarkedDirty(Node* node);

private:
    /// Create or re-add 
    void AddAgentToCrowd();
    /// Remove 
    void RemoveAgentFromCrowd();

    WeakPtr<NavigationCrowdManager> crowdManager_;
    /// in DetourCrowd ? 
    bool inCrowd_;
    /// DetourCrowd reference to this agent.
    int agentCrowdId_;
    /// Reference to poly closest to requested target position.
    unsigned int targetRef_;         
    /// Actual target position, closest to that requested.
    Vector3 targetPosition_;   
    /// update nodes position ?
    bool updateNodePosition_;
    /// Agents max acceleration.
    float maxAccel_;
    /// Agents max Velocity.
    float maxSpeed_;
    /// Agents height.
    float height_;
    /// Agents radius.
    float radius_;
    /// Agents flags.
    unsigned flags_;
    /// Agent's NavigationAvoidanceQuality
    NavigationAvoidanceQuality avoidanceQuality_;
    /// Agent's Navigation Pushiness
    NavigationPushiness pushiness_;
    /// Agent's velocity attribute.
    Vector3 velocity_;
};

}