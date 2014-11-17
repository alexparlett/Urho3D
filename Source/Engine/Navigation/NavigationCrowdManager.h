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

class dtCrowd;
struct dtCrowdAgent;
struct dtCrowdAgentDebugInfo;

namespace Urho3D
{

class NavigationMesh;
class NavigationAgent;

enum NavigationRegionType
{
    RegionType_Ground = 0,
    RegionType_Water,
    RegionType_Road,
    RegionType_Door,
    RegionType_Grass,
    RegionType_Jump
};

enum NavigationPolyFlags
{
    NPF_WALK = 0x01,		// Ability to walk (ground, grass, road)
    NPF_SWIM = 0x02,		// Ability to swim (water).
    NPF_DOOR = 0x04,		// Ability to move through doors.
    NPF_JUMP = 0x08,		// Ability to jump.
    NPF_DISABLED = 0x10,		// Disabled polygon
    NPF_ALL = 0xffff	// All abilities.
};

enum NavigationAvoidanceQuality
{
    NAQ_LOW = 0,
    NAQ_MEDIUM = 1,
    NAQ_HIGH = 2,
    NAQ_EXTRA_HIGH = 3
};

enum NavigationPushiness
{
    NP_LOW,
    NP_MEDIUM,
    NP_HIGH
};

enum NavigationUpdateFlags
{
    NUF_ANTICIPATE_TURNS = 1,
    NUF_OBSTACLE_AVOIDANCE = 2,
    NUF_SEPARATION = 4,
    NUF_OPTIMIZE_VIS = 8,
    NUF_OPTIMIZE_TOPO = 16
};


/// Detour Crowd Simulation Scene Component. Should be added only to the root scene node.
/// Agents radius and height is set through the navigation mesh.
/// \todo support multiple agents radii and heights
class URHO3D_API NavigationCrowdManager : public Component
{
    friend class NavigationAgent;

    OBJECT(NavigationCrowdManager);
              
public:
    /// Construct.
    NavigationCrowdManager(Context* context);
    /// Destruct.
    virtual ~NavigationCrowdManager();
    /// Register object factory.
    static void RegisterObject(Context* context);

    /// Get the Navigation mesh assigned to the crowd.
    NavigationMesh* GetNavigationMesh();
    /// Gets all agents.
    const PODVector<NavigationAgent*>& GetNavigationAgents() const;
    /// Gets the closest walkable position.
    Vector3 GetClosestWalkablePosition(Vector3 pos);

    /// Create detour crowd component for the specified navigation mesh.
    bool CreateCrowd();
    /// Create filter query with the specified NavigationPolyFlags. Index can be between 0 (inclusive) and 16.
    bool CreateQuery(int index, unsigned excludedPolyFlags);
    /// Draw the agents debug data. 
    void DrawDebug(DebugRenderer* debug, bool depthTest);

protected:
    /// Create and adds an detour crowd agent.
    int AddAgent(const Vector3 &pos, float maxaccel, float maxSpeed, float radius, float height, unsigned flags, unsigned avoidanceType = 3);
    /// Removes the detour crowd agent.
    void RemoveAgent(int agent);
    /// Adds the agent scene component to the list. Called from NavigationAgent.
    void AddAgentComponent(NavigationAgent* agent);
    /// Removes the agent scene component from the list. Called from NavigationAgent.
    void RemoveAgentComponent(NavigationAgent* agent);

    /// Update the Navigation Agents Avoidance Quality for the specified agent.
    void UpdateAgentAvoidanceQuality(int agent, NavigationAvoidanceQuality nq);
    /// Update the Navigation Agents Pushiness for the specified agent.
    void UpdateAgentPushiness(int agent, NavigationPushiness pushiness);
    /// Update the Navigation Agents MaxSpeed for the specified agent.
    void UpdateAgentMaxSpeed(int agent, float maxSpeed);
    /// Update the Navigation Agents MaxAcceleration for the specified agent.
    void UpdateAgentMaxAcceleration(int agent, float accel);
    /// Update the Navigation Agents Radius for the specified agent.
    void UpdateAgentRadius(int agent, float radius);
    /// Update the Navigation Agents Height for the specified agent.
    void UpdateAgentHeight(int agent, float height);
    /// Update the Navigation Agents Flags for the specified agent.
    void UpdateAgentFlags(int agent, unsigned flags);
    /// Update the Navigation Agents Query for the specified agent.
    void UpdateAgentQuery(int agent, int query);

    /// Sets the move target for the specified agent.
    bool SetAgentTarget(int agent, Vector3 target);
    /// Sets the move target for the specified agent.
    bool SetAgentTarget(int agent, Vector3 target, unsigned int & targetRef);
    /// Sets the move velocity for the specified agent.
    bool SetMoveVelocity(int agent, const Vector3 & velocity);

    /// Gets the agents position for the specified agent.
    Vector3 GetAgentPosition(int agent);
    /// Gets the agents current velocity for the specified agent.
    Vector3 GetAgentCurrentVelocity(int agent);
    /// Gets the agents desired velocity for the specified agent.
    Vector3 GetAgentDesiredVelocity(int agent);

    /// Gets the detour crowd agent.
    const dtCrowdAgent * GetCrowdAgent(int agent);
    /// Gets the internal detour crowd component.
    dtCrowd * GetCrowd();

    /// Handle node being assigned.
    virtual void OnNodeSet(Node* node);

private:
    /// Handle the scene subsystem update event, step simulation here.
    void HandleSceneSubsystemUpdate(StringHash eventType, VariantMap& eventData);
    /// Update the crowd simulation
    void Update(float delta);

    /// internal crowd component
    dtCrowd* crowd_;
    /// NavigationMesh for which the crowd was created
    WeakPtr<NavigationMesh> navigationMesh_;
    /// max agents for the crowd 
    int maxAgents_;	
    /// Agent Components 
    PODVector<NavigationAgent*> agentComponents_;
    /// internal debug information 
    dtCrowdAgentDebugInfo* agentDebug_;
};

}
