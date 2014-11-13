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
#include "DebugDraw.h"
#include "Object.h"

namespace Urho3D
{
    class DebugRenderer;
/// NavigationDebugRenderer
class URHO3D_API NavigationDebugRenderer : public Object, public duDebugDraw
{
    OBJECT(NavigationDebugRenderer);

public:
    /// Construct.
    NavigationDebugRenderer(Context* context);
    /// Destruct.
    virtual ~NavigationDebugRenderer();
    /// Register object factory.
    static void RegisterObject(Context* context);

    virtual void SetDebugRenderer(DebugRenderer* debug);

    virtual void depthMask(bool state);

    virtual void texture(bool state);
    /// Begin drawing primitives.
    ///  @param prim [in] primitive type to draw, one of rcDebugDrawPrimitives.
    ///  @param size [in] size of a primitive, applies to point size and line width only.
    virtual void begin(duDebugDrawPrimitives prim, float size = 1.0f);
    /// Submit a vertex
    ///  @param pos [in] position of the verts.
    ///  @param color [in] color of the verts.
    virtual void vertex(const float* pos, unsigned int color);
    /// Submit a vertex
    ///  @param x,y,z [in] position of the verts.
    ///  @param color [in] color of the verts.
    virtual void vertex(const float x, const float y, const float z, unsigned int color);
    /// Submit a vertex
    ///  @param pos [in] position of the verts.
    ///  @param color [in] color of the verts.
    virtual void vertex(const float* pos, unsigned int color, const float* uv);
    /// Submit a vertex
    ///  @param x,y,z [in] position of the verts.
    ///  @param color [in] color of the verts.
    virtual void vertex(const float x, const float y, const float z, unsigned int color, const float u, const float v);
    /// End drawing primitives.
    virtual void end();


private:
    void DrawPoints();
    void DrawLines();
    void DrawTris();
    void DrawQuads();
    duDebugDrawPrimitives type_;
    WeakPtr<DebugRenderer> debugRenderer_;
    bool depthTest_;
    /// Debug rendering line.
    struct DebugVertex
    {
        /// Construct undefined.
        DebugVertex()
        {
        }
        /// Construct with start and end positions and color.
        DebugVertex(const Vector3& vert, unsigned color) :
            vert_(vert),
            color_(color)
        {
        }
        Vector3 vert_;
        unsigned color_;
    };
    PODVector<DebugVertex> vertices_;
};

}
