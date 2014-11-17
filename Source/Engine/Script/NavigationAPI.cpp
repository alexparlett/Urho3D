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
#ifdef URHO3D_NAVIGATION
#include "APITemplates.h"
#include "Navigable.h"
#include "NavigationMesh.h"
#include "OffMeshConnection.h"
#include "NavigationCrowdManager.h"
#include "NavigationAgent.h"

namespace Urho3D
{

void RegisterNavigable(asIScriptEngine* engine)
{
    RegisterComponent<Navigable>(engine, "Navigable");
    engine->RegisterObjectMethod("Navigable", "void set_recursive(bool)", asMETHOD(Navigable, SetRecursive), asCALL_THISCALL);
    engine->RegisterObjectMethod("Navigable", "bool get_recursive() const", asMETHOD(Navigable, IsRecursive), asCALL_THISCALL);
}

static CScriptArray* NavigationMeshFindPath(const Vector3& start, const Vector3& end, const Vector3& extents, NavigationMesh* ptr)
{
    PODVector<Vector3> dest;
    ptr->FindPath(dest, start, end, extents);
    return VectorToArray<Vector3>(dest, "Array<Vector3>");
}

void RegisterNavigationMesh(asIScriptEngine* engine)
{
    RegisterComponent<NavigationMesh>(engine, "NavigationMesh");
    engine->RegisterObjectMethod("NavigationMesh", "bool Build()", asMETHODPR(NavigationMesh, Build, (void), bool), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationMesh", "bool Build(const BoundingBox&in)", asMETHODPR(NavigationMesh, Build, (const BoundingBox&), bool), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationMesh", "Vector3 FindNearestPoint(const Vector3&in, const Vector3&in extents = Vector3(1.0, 1.0, 1.0))", asMETHOD(NavigationMesh, FindNearestPoint), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationMesh", "Vector3 MoveAlongSurface(const Vector3&in, const Vector3&in, const Vector3&in extents = Vector3(1.0, 1.0, 1.0), uint = 3)", asMETHOD(NavigationMesh, MoveAlongSurface), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationMesh", "Array<Vector3>@ FindPath(const Vector3&in, const Vector3&in, const Vector3&in extents = Vector3(1.0, 1.0, 1.0))", asFUNCTION(NavigationMeshFindPath), asCALL_CDECL_OBJLAST);
    engine->RegisterObjectMethod("NavigationMesh", "Vector3 GetRandomPoint()", asMETHOD(NavigationMesh, GetRandomPoint), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationMesh", "Vector3 GetRandomPointInCircle(const Vector3&in, float, const Vector3&in extents = Vector3(1.0, 1.0, 1.0))", asMETHOD(NavigationMesh, GetRandomPointInCircle), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationMesh", "float GetDistanceToWall(const Vector3&in, float, const Vector3&in extents = Vector3(1.0, 1.0, 1.0))", asMETHOD(NavigationMesh, GetDistanceToWall), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationMesh", "Vector3 Raycast(const Vector3&in, const Vector3&in, const Vector3&in extents = Vector3(1.0, 1.0, 1.0))", asMETHOD(NavigationMesh, Raycast), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationMesh", "void DrawDebugGeometry(bool)", asMETHODPR(NavigationMesh, DrawDebugGeometry, (bool), void), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationMesh", "void set_tileSize(int)", asMETHOD(NavigationMesh, SetTileSize), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationMesh", "int get_tileSize() const", asMETHOD(NavigationMesh, GetTileSize), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationMesh", "void set_cellSize(float)", asMETHOD(NavigationMesh, SetCellSize), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationMesh", "float get_cellSize() const", asMETHOD(NavigationMesh, GetCellSize), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationMesh", "void set_cellHeight(float)", asMETHOD(NavigationMesh, SetCellHeight), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationMesh", "float get_cellHeight() const", asMETHOD(NavigationMesh, GetCellHeight), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationMesh", "void set_agentHeight(float)", asMETHOD(NavigationMesh, SetAgentHeight), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationMesh", "float get_agentHeight() const", asMETHOD(NavigationMesh, GetAgentHeight), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationMesh", "void set_agentRadius(float)", asMETHOD(NavigationMesh, SetAgentRadius), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationMesh", "float get_agentRadius() const", asMETHOD(NavigationMesh, GetAgentRadius), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationMesh", "void set_agentMaxClimb(float)", asMETHOD(NavigationMesh, SetAgentMaxClimb), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationMesh", "float get_agentMaxClimb() const", asMETHOD(NavigationMesh, GetAgentMaxClimb), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationMesh", "void set_agentMaxSlope(float)", asMETHOD(NavigationMesh, SetAgentMaxSlope), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationMesh", "float get_agentMaxSlope() const", asMETHOD(NavigationMesh, GetAgentMaxSlope), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationMesh", "void set_regionMinSize(float)", asMETHOD(NavigationMesh, SetRegionMinSize), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationMesh", "float get_regionMinSize() const", asMETHOD(NavigationMesh, GetRegionMinSize), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationMesh", "void set_regionMergeSize(float)", asMETHOD(NavigationMesh, SetRegionMergeSize), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationMesh", "float get_regionMergeSize() const", asMETHOD(NavigationMesh, GetRegionMergeSize), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationMesh", "void set_edgeMaxLength(float)", asMETHOD(NavigationMesh, SetEdgeMaxLength), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationMesh", "float get_edgeMaxLength() const", asMETHOD(NavigationMesh, GetEdgeMaxLength), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationMesh", "void set_edgeMaxError(float)", asMETHOD(NavigationMesh, SetEdgeMaxError), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationMesh", "float get_edgeMaxError() const", asMETHOD(NavigationMesh, GetEdgeMaxError), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationMesh", "void set_detailSampleDistance(float)", asMETHOD(NavigationMesh, SetDetailSampleDistance), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationMesh", "float get_detailSampleDistance() const", asMETHOD(NavigationMesh, GetDetailSampleDistance), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationMesh", "void set_detailSampleMaxError(float)", asMETHOD(NavigationMesh, SetDetailSampleMaxError), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationMesh", "float get_detailSampleMaxError() const", asMETHOD(NavigationMesh, GetDetailSampleMaxError), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationMesh", "void set_padding(const Vector3&in)", asMETHOD(NavigationMesh, SetPadding), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationMesh", "const Vector3& get_padding() const", asMETHOD(NavigationMesh, GetPadding), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationMesh", "bool get_initialized() const", asMETHOD(NavigationMesh, IsInitialized), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationMesh", "const BoundingBox& get_boundingBox() const", asMETHOD(NavigationMesh, GetBoundingBox), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationMesh", "BoundingBox get_worldBoundingBox() const", asMETHOD(NavigationMesh, GetWorldBoundingBox), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationMesh", "IntVector2 get_numTiles() const", asMETHOD(NavigationMesh, GetNumTiles), asCALL_THISCALL);
}

void RegisterOffMeshConnection(asIScriptEngine* engine)
{
    RegisterComponent<OffMeshConnection>(engine, "OffMeshConnection");
    engine->RegisterObjectMethod("OffMeshConnection", "void set_endPoint(Node@+)", asMETHOD(OffMeshConnection, SetEndPoint), asCALL_THISCALL);
    engine->RegisterObjectMethod("OffMeshConnection", "Node@+ get_endPoint() const", asMETHOD(OffMeshConnection, GetEndPoint), asCALL_THISCALL);
    engine->RegisterObjectMethod("OffMeshConnection", "void set_radius(float)", asMETHOD(OffMeshConnection, SetRadius), asCALL_THISCALL);
    engine->RegisterObjectMethod("OffMeshConnection", "float get_radius() const", asMETHOD(OffMeshConnection, GetRadius), asCALL_THISCALL);
    engine->RegisterObjectMethod("OffMeshConnection", "void set_bidirectional(bool)", asMETHOD(OffMeshConnection, SetBidirectional), asCALL_THISCALL);
    engine->RegisterObjectMethod("OffMeshConnection", "bool get_bidirectional() const", asMETHOD(OffMeshConnection, IsBidirectional), asCALL_THISCALL);
}


void RegisterNavigationCrowdManager(asIScriptEngine *engine)
{
    engine->RegisterEnum("NavigationAvoidanceQuality");
    engine->RegisterEnumValue("NavigationAvoidanceQuality", "NAQ_LOW", NAQ_LOW);
    engine->RegisterEnumValue("NavigationAvoidanceQuality", "NAQ_MEDIUM", NAQ_MEDIUM);
    engine->RegisterEnumValue("NavigationAvoidanceQuality", "NAQ_HIGH", NAQ_HIGH);
    
    engine->RegisterEnum("NavigationPushiness");
    engine->RegisterEnumValue("NavigationPushiness", "NP_LOW", NP_LOW);
    engine->RegisterEnumValue("NavigationPushiness", "NP_MEDIUM", NP_MEDIUM);
    engine->RegisterEnumValue("NavigationPushiness", "NP_HIGH", NP_HIGH);

    engine->RegisterEnum("NavigationPolyFlags");
    engine->RegisterEnumValue("NavigationPolyFlags", "NPF_WALK", NPF_WALK);
    engine->RegisterEnumValue("NavigationPolyFlags", "NPF_SWIM", NPF_SWIM);
    engine->RegisterEnumValue("NavigationPolyFlags", "NPF_JUMP", NPF_JUMP);
    engine->RegisterEnumValue("NavigationPolyFlags", "NPF_DOOR", NPF_DOOR);
    engine->RegisterEnumValue("NavigationPolyFlags", "NPF_DISABLED", NPF_DISABLED);
    engine->RegisterEnumValue("NavigationPolyFlags", "NPF_ALL", NPF_ALL);

    engine->RegisterEnum("NavigationUpdateFlags");
    engine->RegisterEnumValue("NavigationUpdateFlags", "NUF_ANTICIPATE_TURNS", NUF_ANTICIPATE_TURNS);
    engine->RegisterEnumValue("NavigationUpdateFlags", "NUF_OBSTACLE_AVOIDANCE", NUF_OBSTACLE_AVOIDANCE);
    engine->RegisterEnumValue("NavigationUpdateFlags", "NUF_SEPARATION", NUF_SEPARATION);
    engine->RegisterEnumValue("NavigationUpdateFlags", "NUF_OPTIMIZE_VIS", NUF_OPTIMIZE_VIS);
    engine->RegisterEnumValue("NavigationUpdateFlags", "NUF_OPTIMIZE_TOPO", NUF_OPTIMIZE_TOPO);

    RegisterComponent<NavigationCrowdManager>(engine, "NavigationCrowdManager");
    engine->RegisterObjectMethod("NavigationCrowdManager", "NavigationMesh@+ get_navigationMesh()", asMETHOD(NavigationCrowdManager, GetNavigationMesh), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationCrowdManager", "bool CreateCrowd()", asMETHOD(NavigationCrowdManager, CreateCrowd), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationCrowdManager", "void DrawDebug(DebugRenderer@+,bool)", asMETHOD(NavigationCrowdManager, DrawDebug), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationCrowdManager", "Vector3 GetClosestWalkablePosition(Vector3)", asMETHOD(NavigationCrowdManager, GetClosestWalkablePosition), asCALL_THISCALL);
    
}

void RegisterNavigationAgent(asIScriptEngine *engine)
{
    engine->RegisterEnum("NavigationTargetState");
    engine->RegisterEnumValue("NavigationTargetState", "NAV_AGENT_TARGET_NONE", NAV_AGENT_TARGET_NONE);
    engine->RegisterEnumValue("NavigationTargetState", "NAV_AGENT_TARGET_FAILED", NAV_AGENT_TARGET_FAILED);
    engine->RegisterEnumValue("NavigationTargetState", "NAV_AGENT_TARGET_VALID", NAV_AGENT_TARGET_VALID);
    engine->RegisterEnumValue("NavigationTargetState", "NAV_AGENT_TARGET_REQUESTING", NAV_AGENT_TARGET_REQUESTING);
    engine->RegisterEnumValue("NavigationTargetState", "NAV_AGENT_TARGET_WAITINGFORPATH", NAV_AGENT_TARGET_WAITINGFORPATH);
    engine->RegisterEnumValue("NavigationTargetState", "NAV_AGENT_TARGET_WAITINGFORQUEUE", NAV_AGENT_TARGET_WAITINGFORQUEUE);
    engine->RegisterEnumValue("NavigationTargetState", "NAV_AGENT_TARGET_VELOCITY", NAV_AGENT_TARGET_VELOCITY);
    engine->RegisterEnumValue("NavigationTargetState", "NAV_AGENT_TARGET_ARRIVED", NAV_AGENT_TARGET_ARRIVED);
    
    engine->RegisterEnum("NavigationAgentState");
    engine->RegisterEnumValue("NavigationAgentState", "NAV_AGENT_INVALID", NAV_AGENT_INVALID);
    engine->RegisterEnumValue("NavigationAgentState", "NAV_AGENT_READY", NAV_AGENT_READY);
    engine->RegisterEnumValue("NavigationAgentState", "NAV_AGENT_TRAVERSINGLINK", NAV_AGENT_TRAVERSINGLINK);
    
    RegisterComponent<NavigationAgent>(engine, "NavigationAgent");
    engine->RegisterObjectMethod("NavigationAgent", "bool SetMoveTarget(const Vector3&in)", asMETHOD(NavigationAgent, SetMoveTarget), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationAgent", "bool SetMoveVelocity(const Vector3&in)", asMETHOD(NavigationAgent, SetMoveVelocity), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationAgent", "void set_updateNodePosition(bool)", asMETHOD(NavigationAgent, SetUpdateNodePosition), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationAgent", "bool get_updateNodePosition() const", asMETHOD(NavigationAgent, GetUpdateNodePosition), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationAgent", "void set_maxAccel(float)", asMETHOD(NavigationAgent, SetMaxAccel), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationAgent", "float get_maxAccel() const", asMETHOD(NavigationAgent, GetMaxAccel), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationAgent", "void set_maxSpeed(float)", asMETHOD(NavigationAgent, SetMaxSpeed), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationAgent", "float get_maxSpeed() const", asMETHOD(NavigationAgent, GetMaxSpeed), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationAgent", "void set_height(float)", asMETHOD(NavigationAgent, SetHeight), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationAgent", "float get_height() const", asMETHOD(NavigationAgent, GetHeight), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationAgent", "void set_radius(float)", asMETHOD(NavigationAgent, SetRadius), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationAgent", "float get_radius() const", asMETHOD(NavigationAgent, GetRadius), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationAgent", "void set_flags(uint)", asMETHOD(NavigationAgent, SetFlags), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationAgent", "uint get_flags() const", asMETHOD(NavigationAgent, GetFlags), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationAgent", "void set_navigationAvoidanceQuality(NavigationAvoidanceQuality)", asMETHOD(NavigationAgent, SetNavigationAvoidanceQuality), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationAgent", "NavigationAvoidanceAvoidanceQuality get_navigationQuality() const", asMETHOD(NavigationAgent, GetNavigationAvoidanceQuality), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationAgent", "void set_navigationPushiness(NavigationPushiness)", asMETHOD(NavigationAgent, SetNavigationPushiness), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationAgent", "NavigationPushiness get_navigationPushiness() const", asMETHOD(NavigationAgent, GetNavigationPushiness), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationAgent", "Vector3 get_desiredVelocity() const", asMETHOD(NavigationAgent, GetDesiredVelocity), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationAgent", "Vector3 get_actualVelocity() const", asMETHOD(NavigationAgent, GetActualVelocity), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationAgent", "Vector3 get_targetPosition() const", asMETHOD(NavigationAgent, GetTargetPosition), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationAgent", "NavigationAgentState get_agentState() const", asMETHOD(NavigationAgent, GetAgentState), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationAgent", "NavigationTargetState get_targetState() const", asMETHOD(NavigationAgent, GetTargetState), asCALL_THISCALL);
    engine->RegisterObjectMethod("NavigationAgent", "Vector3 get_position() const", asMETHOD(NavigationAgent, GetPosition), asCALL_THISCALL);
}

void RegisterNavigationAPI(asIScriptEngine* engine)
{
    RegisterNavigable(engine);
    RegisterNavigationMesh(engine);
    RegisterOffMeshConnection(engine);
    RegisterNavigationCrowdManager(engine);
    RegisterNavigationAgent(engine);
}

}
#endif
