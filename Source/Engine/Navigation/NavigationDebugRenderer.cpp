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
#include "NavigationDebugRenderer.h"
#include "DebugRenderer.h"

#include "DebugNew.h"
#include "Log.h"


namespace Urho3D
{

extern const char* NAVIGATION_CATEGORY;

NavigationDebugRenderer::NavigationDebugRenderer(Context* context) :
Object(context),
type_(DU_DRAW_POINTS),
depthTest_(false)
{
}

NavigationDebugRenderer::~NavigationDebugRenderer()
{

}

void NavigationDebugRenderer::RegisterObject(Context* context)
{
    context->RegisterFactory<NavigationDebugRenderer>(NAVIGATION_CATEGORY);
}

void NavigationDebugRenderer::depthMask(bool state)
{
    depthTest_ = state;
}

void NavigationDebugRenderer::texture(bool state)
{
    
}

void NavigationDebugRenderer::begin(duDebugDrawPrimitives prim, float size /*= 1.0f*/)
{
    switch (prim)
    {
    case DU_DRAW_POINTS:
        type_ = DU_DRAW_POINTS;
        break;
    case DU_DRAW_LINES:
        type_ = DU_DRAW_LINES;
        break;
    case DU_DRAW_TRIS:
        type_ = DU_DRAW_TRIS;
        break;
    case DU_DRAW_QUADS:
        type_ = DU_DRAW_QUADS;
        break;
    };
}

void NavigationDebugRenderer::vertex(const float* pos, unsigned int color)
{
    vertices_.Push(DebugVertex(Vector3(pos[0], pos[1], pos[2]), color));
}

void NavigationDebugRenderer::vertex(const float x, const float y, const float z, unsigned int color)
{
    vertices_.Push(DebugVertex(Vector3(x, y, z), color));
}

void NavigationDebugRenderer::vertex(const float* pos, unsigned int color, const float* uv)
{
    vertices_.Push(DebugVertex(Vector3(pos[0], pos[1], pos[2]), color));
}

void NavigationDebugRenderer::vertex(const float x, const float y, const float z, unsigned int color, const float u, const float v)
{
    vertices_.Push(DebugVertex(Vector3(x, y, z), color));
}

void NavigationDebugRenderer::end()
{
    if (debugRenderer_.Expired())
        return;

    switch (type_)
    {
    case DU_DRAW_POINTS:
        DrawPoints();
        break;
    case DU_DRAW_LINES:
        DrawLines();
        break;
    case DU_DRAW_TRIS:
        DrawTris();
        break;
    case DU_DRAW_QUADS:
        DrawQuads();
        break;
    };

    // When the amount of debug geometry is reduced, release memory
    unsigned  verticesSize = vertices_.Size();

    vertices_.Clear();

    if (vertices_.Capacity() > verticesSize * 2)
        vertices_.Reserve(verticesSize);

}

void NavigationDebugRenderer::SetDebugRenderer(DebugRenderer* debug)
{
    debugRenderer_ = WeakPtr<DebugRenderer>(debug);
}

void NavigationDebugRenderer::DrawPoints()
{
    
}

void NavigationDebugRenderer::DrawLines()
{
    for (unsigned i = 0; i < vertices_.Size(); i= i+2)
    {
        const DebugVertex& vertex = vertices_[i];

        debugRenderer_->AddLine(vertices_[i].vert_, vertices_[i + 1].vert_, vertices_[i].color_, depthTest_);
    }

    
}

void NavigationDebugRenderer::DrawTris()
{
    for (unsigned i = 0; i < vertices_.Size(); i = i + 3)
    {
        const DebugVertex& vertex = vertices_[i];

        debugRenderer_->AddTriangle(vertices_[i].vert_, vertices_[i + 1].vert_, vertices_[i + 2].vert_, vertices_[i].color_, depthTest_);
    }
}

void NavigationDebugRenderer::DrawQuads()
{
    for (unsigned i = 0; i < vertices_.Size(); i = i + 4)
    {
        const DebugVertex& vertex = vertices_[i];

        debugRenderer_->AddTriangle(vertices_[i].vert_, vertices_[i + 1].vert_, vertices_[i + 3].vert_, vertices_[i].color_, depthTest_);
        debugRenderer_->AddTriangle(vertices_[i + 3].vert_, vertices_[i + 1].vert_, vertices_[i + 2].vert_, vertices_[i].color_, depthTest_);
    }
}

}
