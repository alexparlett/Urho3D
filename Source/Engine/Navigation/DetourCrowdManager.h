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
	PolyFlags_Walk = 0x01,		// Ability to walk (ground, grass, road)
	PolyFlags_Swim = 0x02,		// Ability to swim (water).
	PolyFlags_Door = 0x04,		// Ability to move through doors.
	PolyFlags_Jump = 0x08,		// Ability to jump.
	PolyFlags_Disabled = 0x10,		// Disabled polygon
	PolyFlags_All = 0xffff	// All abilities.
};
enum NavigationAvoidanceQuality
{
	NAVIGATIONQUALITY_LOW = 0,
	NAVIGATIONQUALITY_MED = 1,
	NAVIGATIONQUALITY_HIGH = 2
};
enum NavigationPushiness
{
	PUSHINESS_LOW,
	PUSHINESS_MEDIUM,
	PUSHINESS_HIGH
};

/// Detour Crowd Simulation Scene Component. Should be added only to the root scene node.
/// Agents radius and height is set through the navigation mesh.
/// \todo support multiple agents radii and heights
class URHO3D_API DetourCrowdManager : public Component
{
		OBJECT(DetourCrowdManager);
		friend class NavigationAgent;
			  
public:
    /// Construct.
	DetourCrowdManager(Context* context);
    /// Destruct.
	virtual ~DetourCrowdManager();
    /// Register object factory.
    static void RegisterObject(Context* context);


	/// Assigns the navigation mesh for the crowd.
	void SetNavigationMesh(NavigationMesh *navMesh);
	/// Get the Navigation mesh assigned to the crowd.
	NavigationMesh* GetNavigationMesh();

	/// Create detour crowd component for the specified navigation mesh.
	bool CreateCrowd();

	/// Update the crod simulation
	void Update(float delta);

	/// Create and adds an detour crowd agent.
	/// Agents radius and height is set through the navigation mesh!
	int AddAgent(Vector3 pos,  float maxaccel, float maxSpeed);
	/// Removes the detour crowd agent.
	void RemoveAgent(int agent);

	/// Update the Navigation Agents Avoidance Quality for the specified agent.
	void UpdateAgentNavigationQuality(int agent, NavigationAvoidanceQuality nq);
	/// Update the Navigation Agents Pushiness for the specified agent.
	void UpdateAgentPushiness(int agent, NavigationPushiness pushiness);
	/// Update the Navigation Agents MaxSpeed for the specified agent.
	void UpdateAgentMaxSpeed(int agent, float maxSpeed);
	/// Update the Navigation Agents MaxAcceleration for the specified agent.
	void UpdateAgentMaxAcceleration(int agent, float accel);

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

	/// Gets the closest walkable position.
	Vector3 GetClosestWalkablePosition(Vector3 pos);

	/// Gets all agents.
	const PODVector<NavigationAgent*>& GetNavigationAgents() const;
	/// \todo add flags to specify what to show. path line , Velocity, Paths corridor polys ...
	/// Draw the agents debug data. 
	void DrawDebug(DebugRenderer* debug, bool depthTest);
protected:
	/// Handle node being assigned.
	virtual void OnNodeSet(Node* node);
	/// Gets the detour crowd agent.
	const dtCrowdAgent * GetCrowdAgent(int agent);
	/// Adds the agent scene component to the list. Called from NavigationAgent.
	void AddAgentComponent(NavigationAgent* agent);
	/// Removes the agent scene component from the list. Called from NavigationAgent.
	void RemoveAgentComponent(NavigationAgent* agent);
	/// Gets the internal detour crowd component.
	dtCrowd * GetCrowd();
private:
	/// Handle the scene subsystem update event, step simulation here.
	void HandleSceneSubsystemUpdate(StringHash eventType, VariantMap& eventData);

	/// internal crowd component
	dtCrowd* crowd_;
	/// NavigationMesh for which the crowd was created
	WeakPtr<NavigationMesh> navMesh_;
	/// \todo add an check if max agents reached to addagent /addagentcomponent ... ?
	/// max agents for the crowd 
	int maxAgents_;	
	/// Agent Components 
	PODVector<NavigationAgent*> agentComponents_;
	/// internal debug information 
	dtCrowdAgentDebugInfo* agentDebug_;
};

}
