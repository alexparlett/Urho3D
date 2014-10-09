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

namespace Urho3D
{
class NavigationMesh;
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

/// Component which tags geometry for inclusion in the navigation mesh. Optionally auto-includes geometry from child nodes.
class URHO3D_API DetourCrowdManager : public Component
{
		OBJECT(DetourCrowdManager);

public:


    /// Construct.
	DetourCrowdManager(Context* context);
    /// Destruct.
	virtual ~DetourCrowdManager();
    /// Register object factory.
    static void RegisterObject(Context* context);

	void SetNavigationMesh(NavigationMesh *navMesh);
	NavigationMesh* GetNavigationMesh();

	bool CreateCrowd();

	void Update(float delta);

	int AddAgent(Vector3 pos, float radius, float height, float accel, float maxSpeed);
	void RemoveAgent(int agent);

	void UpdateAgentNavigationQuality(int agent, NavigationAvoidanceQuality nq);
	void UpdateAgentPushiness(int agent, NavigationPushiness pushiness);
	void UpdateAgentMaxSpeed(int agent, float maxSpeed);
	void UpdateAgentMaxAcceleration(int agent, float accel);

	bool SetAgentTarget(int agent, Vector3 target);
	bool SetMoveVelocity(int agent, const Vector3 & velocity);

	Vector3 GetAgentPosition(int agent);
	Vector3 GetAgentCurrentVelocity(int agent);
	Vector3 GetAgentDesiredVelocity(int agent);

	Vector3 GetClosestWalkablePosition(Vector3 pos);

private:
	dtCrowd* crowd_;
	WeakPtr<NavigationMesh> navMesh_;
	int maxAgents_;	
};

}
