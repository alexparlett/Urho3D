
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
#include "AnnotationBuilder.h"
#include "NavigationCrowdManager.h"
#include "NavigationMesh.h"
#include "Log.h"
#include "Profiler.h"
#include <Recast.h>
#include "NavigationDebugRenderer.h"
/// \todo use urho3d functions 
#include "vecmath.h"

#include "DebugNew.h"
#include <float.h>




namespace Urho3D
{
extern const char* NAVIGATION_CATEGORY;

static float getHeight(const float x, const float* pts, const int npts)
{
    using namespace VecMath;
    if (x <= pts[0])
        return pts[1];
    if (x >= pts[(npts - 1) * 2])
        return pts[(npts - 1) * 2 + 1];

    for (int i = 1; i < npts; ++i)
    {
        const float* q = &pts[i * 2];
        if (x <= q[0])
        {
            const float* p = &pts[(i - 1) * 2];
            const float u = (x - p[0]) / (q[0] - p[0]);
            return lerp(p[1], q[1], u);
        }
    }

    return pts[(npts - 1) * 2 + 1];
}
static void evalArc(float* pt, const float* sp, const float* sq, const float u, const float h)
{
    using namespace VecMath;
    vlerp(pt, sp, sq, u);
    const float u2 = u*u*u; //1 - sqr(1-u);
    pt[1] = sp[1] + (sq[1] - sp[1]) * u2;
}

static void trans2d(float* dst, const float* ax, const float* ay, const float* pt)
{
    dst[0] = ax[0] * pt[0] + ay[0] * pt[1];
    dst[1] = ax[1] * pt[0] + ay[1] * pt[1];
    dst[2] = ax[2] * pt[0] + ay[2] * pt[1];
}

static void trans2dc(float* dst, const float* c, const float* ax, const float* ay, const float* pt)
{
    dst[0] = c[0] + ax[0] * pt[0] + ay[0] * pt[1];
    dst[1] = c[1] + ax[1] * pt[0] + ay[1] * pt[1];
    dst[2] = c[2] + ax[2] * pt[0] + ay[2] * pt[1];
}


AnnotationBuilder::AnnotationBuilder(Context* context) : Object(context),
    selEdge_(0),
    nedges_(-1),
    nlinks_(0),
    clinks_(0),
    edges_(NULL),
    sampler_(NULL),
    links_(NULL)
{

}
AnnotationBuilder::~AnnotationBuilder()
{
    Cleanup();
}

void AnnotationBuilder::RegisterObject(Context* context)
{
    context->RegisterFactory<AnnotationBuilder>(NAVIGATION_CATEGORY);
}
void AnnotationBuilder::ClearLinks()
{
    if (links_)
        delete[] links_;
    links_ = 0;
    nlinks_ = 0;
    clinks_ = 0;
}

bool AnnotationBuilder::Build(const AnnotationBuilderConfig& acfg, NavigationMesh *navMesh)
{
    PROFILE(BuildAnnotationBuilder);
    if (navMesh == NULL)
        return false;
    // AnnotationBuilder needs internal data
    if (!navMesh->GetKeepInterResults())
    {
        LOGERROR("Annotation Builder needs internal debug data from NavMesh !");
        return false;
    }
    navMesh_ = WeakPtr<NavigationMesh>(navMesh);
    acfg_ = acfg;
    Cleanup();

    // Build edges for every tile
    nedges_ = 0;
    HashMap<Pair<int, int>, NavigationBuildData*>::Iterator it;
    for (it = navMesh->builds_.Begin(); it != navMesh->builds_.End(); it++)
    {
        if (!it->second_)
            continue;
        if (it->second_->contourSet_)
        {
            for (int i = 0; i < it->second_->contourSet_->nconts; ++i)
                nedges_ += it->second_->contourSet_->conts[i].nverts;
        }
    }

    if (!nedges_)
    {
        LOGERROR("Annotation Builder: No edges!");
        return false;
    }

    edges_ = new Edge[nedges_];

    nedges_ = 0;
    for (it = navMesh->builds_.Begin(); it != navMesh->builds_.End(); it++)
    {
        if (!it->second_)
            continue;
        if (it->second_->contourSet_)
        {
            const float* orig = it->second_->contourSet_->bmin;
            const float cs = it->second_->contourSet_->cs;
            const float ch = it->second_->contourSet_->ch;

            for (int i = 0; i < it->second_->contourSet_->nconts; ++i)
            {
                const rcContour& c = it->second_->contourSet_->conts[i];
                if (!c.nverts)
                    continue;
                for (int j = 0, k = c.nverts - 1; j < c.nverts; k = j++)
                {
                    k = (j + 1) % c.nverts;
                    const int* vj = &c.verts[j * 4];
                    const int* vk = &c.verts[k * 4];

                    Edge* e = &edges_[nedges_++];

                    e->sp[0] = orig[0] + vk[0] * cs;
                    e->sp[1] = orig[1] + (vk[1] + 2)*ch;
                    e->sp[2] = orig[2] + vk[2] * cs;

                    e->sq[0] = orig[0] + vj[0] * cs;
                    e->sq[1] = orig[1] + (vj[1] + 2)*ch;
                    e->sq[2] = orig[2] + vj[2] * cs;
                }

            }

        }
    }

    return false;
}


void AnnotationBuilder::BuildNearestEdge(int type, const float* pos)
{

}

void AnnotationBuilder::Cleanup()
{
    if (edges_)
        delete[] edges_;
    edges_ = 0;
    nedges_ = 0;

    if (sampler_)
        delete sampler_;
    sampler_ = 0;

    ClearLinks();
}

void AnnotationBuilder::DrawDebug(DebugRenderer* debug, bool depthTest, unsigned int flags)
{
    /// \todo create custom geometry and update only if dirty ... to increase speed ??
    if (debug)
    {
        PROFILE(DrawDebugAnnotationBuilder);
        NavigationDebugRenderer dd(context_);
        dd.SetDebugRenderer(debug);
        dd.depthMask(depthTest);

        if (flags & DRAW_WALKABLE_BORDER)
        {
            if (nedges_)
            {
                dd.begin(DU_DRAW_LINES, 3.0f);
                for (int i = 0; i < nedges_; ++i)
                {
                    unsigned int col = duRGBA(0, 96, 128, 255);
                    if (i == selEdge_)
                        continue;
                    dd.vertex(edges_[i].sp, col);
                    dd.vertex(edges_[i].sq, col);
                }
                dd.end();

                dd.begin(DU_DRAW_POINTS, 8.0f);
                for (int i = 0; i < nedges_; ++i)
                {
                    unsigned int col = duRGBA(0, 96, 128, 255);
                    if (i == selEdge_)
                        continue;
                    dd.vertex(edges_[i].sp, col);
                    dd.vertex(edges_[i].sq, col);
                }
                dd.end();

                if (selEdge_ >= 0 && selEdge_ < nedges_)
                {
                    unsigned int col = duRGBA(48, 16, 16, 255); //duRGBA(255,192,0,255);
                    dd.begin(DU_DRAW_LINES, 3.0f);
                    dd.vertex(edges_[selEdge_].sp, col);
                    dd.vertex(edges_[selEdge_].sq, col);
                    dd.end();
                    dd.begin(DU_DRAW_POINTS, 8.0f);
                    dd.vertex(edges_[selEdge_].sp, col);
                    dd.vertex(edges_[selEdge_].sq, col);
                    dd.end();
                }

                dd.begin(DU_DRAW_POINTS, 4.0f);
                for (int i = 0; i < nedges_; ++i)
                {
                    unsigned int col = duRGBA(190, 190, 190, 255);
                    dd.vertex(edges_[i].sp, col);
                    dd.vertex(edges_[i].sq, col);
                }
                dd.end();
            }
        }

        if (flags & DRAW_ANNOTATIONS)
        {
            if (nlinks_)
            {
                unsigned int col0 = duLerpCol(duRGBA(32, 255, 96, 255), duRGBA(255, 255, 255, 255), 200);
                unsigned int col1 = duRGBA(32, 255, 96, 255);

                dd.begin(DU_DRAW_QUADS);
                for (int i = 0; i < nlinks_; ++i)
                {
                    const JumpLink* link = &links_[i];
                    if (link->flags == 0) continue;
                    for (int j = 0; j < link->nspine - 1; ++j)
                    {
                        int u = (j * 255) / link->nspine;
                        unsigned int col = duTransCol(duLerpCol(col0, col1, u), 128);
                        if (link->flags == 0)
                            col = duRGBA(255, 0, 0, 64);

                        dd.vertex(&link->spine1[j * 3], col);
                        dd.vertex(&link->spine1[(j + 1) * 3], col);
                        dd.vertex(&link->spine0[(j + 1) * 3], col);
                        dd.vertex(&link->spine0[j * 3], col);
                    }
                }
                dd.end();
                dd.begin(DU_DRAW_LINES, 3.0f);
                for (int i = 0; i < nlinks_; ++i)
                {
                    const JumpLink* link = &links_[i];
                    if (link->flags == 0) continue;
                    for (int j = 0; j < link->nspine - 1; ++j)
                    {
                        //					int u = (j*255)/link->nspine;
                        unsigned int col = duTransCol(duDarkenCol(col1)/*duDarkenCol(duLerpCol(col0,col1,u))*/, 128);

                        dd.vertex(&link->spine0[j * 3], col);
                        dd.vertex(&link->spine0[(j + 1) * 3], col);
                        dd.vertex(&link->spine1[j * 3], col);
                        dd.vertex(&link->spine1[(j + 1) * 3], col);
                    }

                    dd.vertex(&link->spine0[0], duDarkenCol(col1));
                    dd.vertex(&link->spine1[0], duDarkenCol(col1));

                    dd.vertex(&link->spine0[(link->nspine - 1) * 3], duDarkenCol(col1));
                    dd.vertex(&link->spine1[(link->nspine - 1) * 3], duDarkenCol(col1));
                }
                dd.end();

                dd.begin(DU_DRAW_POINTS, 8.0f);
                for (int i = 0; i < nlinks_; ++i)
                {
                    const JumpLink* link = &links_[i];
                    if (link->flags == 0) continue;
                    dd.vertex(&link->spine0[0], duDarkenCol(col1));
                    dd.vertex(&link->spine1[0], duDarkenCol(col1));
                    dd.vertex(&link->spine0[(link->nspine - 1) * 3], duDarkenCol(col1));
                    dd.vertex(&link->spine1[(link->nspine - 1) * 3], duDarkenCol(col1));
                }
                dd.end();
                dd.begin(DU_DRAW_POINTS, 4.0f);
                for (int i = 0; i < nlinks_; ++i)
                {
                    const JumpLink* link = &links_[i];
                    if (link->flags == 0) continue;
                    dd.vertex(&link->spine0[0], duRGBA(220, 220, 220, 255));
                    dd.vertex(&link->spine1[0], duRGBA(220, 220, 220, 255));
                    dd.vertex(&link->spine0[(link->nspine - 1) * 3], duRGBA(220, 220, 220, 255));
                    dd.vertex(&link->spine1[(link->nspine - 1) * 3], duRGBA(220, 220, 220, 255));
                }
                dd.end();

            }
        }
    }
}
void AnnotationBuilder::BuildAllEdges(int type)
{
    PROFILE(BuildAllEdgesAnnotationBuilder);
    if (navMesh_.Expired())
        return;

    for (int i = 0; i < nedges_; ++i)
    {
        if (sampler_)
            delete sampler_;
        sampler_ = SampleEdge(type, edges_[i].sp, edges_[i].sq);
        if (sampler_)
            AddEdgeLinks(sampler_);
    }

    //FilterJumpOverLinks();
}

AnnotationBuilder::EdgeSampler* AnnotationBuilder::SampleEdge(int type, const float* sp, const float* sq)
{
    using namespace VecMath;

    EdgeSampler* es = new EdgeSampler;
    memset(es, 0, sizeof(EdgeSampler));

    if (type == EDGE_JUMP_DOWN)
    {
        InitJumpDownRig(es, sp, sq, -0.25f, 2.0f, -3.0f, 0.5f);
    }

    InitTrajectory(&es->trajectory);

    // Init start end segments.
    float offset[3];
    trans2d(offset, es->az, es->ay, &es->trajectory.spine[0]);
    vadd(es->start.p, es->rigp, offset);
    vadd(es->start.q, es->rigq, offset);
    trans2d(offset, es->az, es->ay, &es->trajectory.spine[(es->trajectory.nspine - 1) * 2]);
    vadd(es->end.p, es->rigp, offset);
    vadd(es->end.q, es->rigq, offset);

    // Sample start and end ground segments.
    const float dist = sqrtf(vdistSqr(es->rigp, es->rigq));
    const float cs = navMesh_->GetCellSize();
    const int ngsamples = max(2, (int) ceilf(dist / cs));

    SampleGroundSegment(&es->start, ngsamples, es->groundRange);
    SampleGroundSegment(&es->end, ngsamples, es->groundRange);

    SampleAction(es);

    return es;
}
void AnnotationBuilder::InitJumpDownRig(EdgeSampler* es, const float* sp, const float* sq, const float jumpStartDist, const float jumpEndDist, const float jumpDownDist, const float groundRange)
{
    using namespace VecMath;

    vcopy(es->rigp, sp);
    vcopy(es->rigq, sq);

    vsub(es->ax, sq, sp);
    vnormalize_(es->ax);
    vset(es->az, es->ax[2], 0, -es->ax[0]);
    vnormalize_(es->az);
    vset(es->ay, 0, 1, 0);

    // Build action sampling spine.
    es->trajectory.nspine = MAX_SPINE;
    for (int i = 0; i < MAX_SPINE; ++i)
    {
        float* pt = &es->trajectory.spine[i * 2];
        const float u = (float) i / (float) (MAX_SPINE - 1);
        pt[0] = jumpStartDist + u * (jumpEndDist - jumpStartDist);
        pt[1] = u*u*u * jumpDownDist;
    }

    es->groundRange = groundRange;

}

static void insertSort(unsigned char* a, const int n)
{
    int i, j;
    for (i = 1; i < n; i++)
    {
        const unsigned char value = a[i];
        for (j = i - 1; j >= 0 && a[j] > value; j--)
            a[j + 1] = a[j];
        a[j + 1] = value;
    }
}

void AnnotationBuilder::AddEdgeLinks(EdgeSampler* es)
{
    using namespace VecMath;

    if (es->start.ngsamples != es->end.ngsamples)
        return;

    const int nsamples = es->start.ngsamples;

    // Filter small holes.
    const int RAD = 2;
    unsigned char kernel[RAD * 2 + 1];

    unsigned char* nflags = new unsigned char[nsamples];

    for (int i = 0; i < nsamples; ++i)
    {
        const int a = max(0, i - RAD);
        const int b = min(nsamples - 1, i + RAD);
        int nkernel = 0;
        for (int j = a; j <= b; ++j)
            kernel[nkernel++] = es->start.gsamples[i].flags & 4;
        insertSort(kernel, nkernel);
        nflags[i] = kernel[(nkernel + 1) / 2];
    }

    // Build segments
    int start = -1;
    for (int i = 0; i <= nsamples; ++i)
    {
        bool valid = i < nsamples && nflags[i] != 0;
        if (start == -1)
        {
            if (valid)
                start = i;
        }
        else
        {
            if (!valid)
            {
                if (i - start > 5)
                {
                    const float u0 = (float) start / (float) nsamples;
                    const float u1 = (float) i / (float) nsamples;

                    float sp[3], sq[3], ep[3], eq[3];

                    vlerp(sp, es->start.p, es->start.q, u0);
                    vlerp(sq, es->start.p, es->start.q, u1);
                    vlerp(ep, es->end.p, es->end.q, u0);
                    vlerp(eq, es->end.p, es->end.q, u1);
                    sp[1] = es->start.gsamples[start].height;
                    sq[1] = es->start.gsamples[i - 1].height;
                    ep[1] = es->end.gsamples[start].height;
                    eq[1] = es->end.gsamples[i - 1].height;

                    JumpLink* link = AddLink();

                    link->flags = 1;
                    link->nspine = es->trajectory.nspine;

                    const float startx = es->trajectory.spine[0];
                    const float endx = es->trajectory.spine[(es->trajectory.nspine - 1) * 2];
                    const float deltax = endx - startx;

                    const float starty = es->trajectory.spine[1];
                    const float endy = es->trajectory.spine[(es->trajectory.nspine - 1) * 2 + 1];

                    // Build start spine.
                    for (int j = 0; j < es->trajectory.nspine; ++j)
                    {
                        const float* spt = &es->trajectory.spine[j * 2];
                        const float u = (spt[0] - startx) / deltax;
                        const float dy = spt[1] - lerp(starty, endy, u) + acfg_.agentClimb;
                        float* p = &link->spine0[j * 3];
                        vlerp(p, sp, ep, u);
                        vmad(p, p, es->ay, dy);
                    }

                    for (int j = 0; j < es->trajectory.nspine; ++j)
                    {
                        const float* spt = &es->trajectory.spine[j * 2];
                        const float u = (spt[0] - startx) / deltax;
                        const float dy = spt[1] - lerp(starty, endy, u) + acfg_.agentClimb;
                        float* p = &link->spine1[j * 3];
                        vlerp(p, sq, eq, u);
                        vmad(p, p, es->ay, dy);
                    }
                }

                start = -1;
            }
        }
    }

    delete[] nflags;
}

void AnnotationBuilder::InitTrajectory(Trajectory2D* tra)
{
    using namespace VecMath;

    const float cs = acfg_.cellSize;

    const float* pa = &tra->spine[0];
    const float* pb = &tra->spine[(tra->nspine - 1) * 2];

    const float dx = pb[0] - pa[0];
    tra->nsamples = max(2, (int) ceilf(dx / cs));
    tra->samples = new TrajectorySample[tra->nsamples];

    for (int i = 0; i < tra->nsamples; ++i)
    {
        const float u = (float) i / (float) (tra->nsamples - 1);
        TrajectorySample* s = &tra->samples[i];
        s->x = lerp(pa[0], pb[0], u);

        const float y0 = getHeight(s->x - acfg_.agentRadius, tra->spine, tra->nspine);
        const float y1 = getHeight(s->x + acfg_.agentRadius, tra->spine, tra->nspine);

        const float y = lerp(pa[1], pb[1], u);

        s->ymin = min(y0, y1) + acfg_.agentClimb - y;
        s->ymax = max(y0, y1) + acfg_.agentHeight - y;
    }
}

void AnnotationBuilder::SampleGroundSegment(GroundSegment* seg, const float nsamples, const float groundRange)
{
    using namespace VecMath;
    float delta[3];
    vsub(delta, seg->p, seg->q);

    seg->ngsamples = nsamples;
    seg->gsamples = new GroundSample[seg->ngsamples];
    seg->npass = 0;

    for (int i = 0; i < seg->ngsamples; ++i)
    {
        const float u = (float) i / (float) (seg->ngsamples - 1);
        float pt[3];

        GroundSample* s = &seg->gsamples[i];
        vlerp(pt, seg->p, seg->q, u);
        s->flags = 0;
        if (!GetCompactHeightfieldHeigh(pt, groundRange, &s->height))
            continue;
        s->flags |= 1;
        seg->npass++;
    }
}

bool AnnotationBuilder::GetCompactHeightfieldHeigh(const float* pt, const float hrange, float* height)
{
    using namespace VecMath;
    rcCompactHeightfield* m_chf = NULL;
    bool found = false;
    float bestDist = FLT_MAX;
    float bestHeight = FLT_MAX;
    const float range = navMesh_->GetCellSize();

    HashMap<Pair<int, int>, NavigationBuildData*>::Iterator it;
    for (it = navMesh_->builds_.Begin(); it != navMesh_->builds_.End(); it++)
    {
        if (!it->second_)
            continue;
        m_chf = it->second_->compactHeightField_;
        if (!m_chf)
            continue;


        const int ix0 = clamp((int) floorf((pt[0] - range - m_chf->bmin[0]) / m_chf->cs), 0, m_chf->width - 1);
        const int iz0 = clamp((int) floorf((pt[2] - range - m_chf->bmin[2]) / m_chf->cs), 0, m_chf->height - 1);
        const int ix1 = clamp((int) floorf((pt[0] + range - m_chf->bmin[0]) / m_chf->cs), 0, m_chf->width - 1);
        const int iz1 = clamp((int) floorf((pt[2] + range - m_chf->bmin[2]) / m_chf->cs), 0, m_chf->height - 1);

        for (int z = iz0; z <= iz1; ++z)
        {
            for (int x = ix0; x <= ix1; ++x)
            {
                const rcCompactCell& c = m_chf->cells[x + z*m_chf->width];
                for (int i = (int) c.index, ni = (int) (c.index + c.count); i < ni; ++i)
                {
                    const rcCompactSpan& s = m_chf->spans[i];
                    if (m_chf->areas[i] == RC_NULL_AREA)
                        continue;
                    const float y = m_chf->bmin[1] + s.y * m_chf->ch;
                    const float dist = abs(y - pt[1]);
                    if (dist < hrange && dist < bestDist)
                    {
                        bestDist = dist;
                        bestHeight = y;
                        found = true;
                    }
                }
            }
        }

        if (found)
        {
            *height = bestHeight;
            break;
        }
        else
            *height = pt[1];

    }



    return found;
}

void AnnotationBuilder::SampleAction(EdgeSampler* es)
{
    using namespace VecMath;

    if (es->start.ngsamples != es->end.ngsamples)
        return;

    const int nsamples = es->start.ngsamples;

    for (int i = 0; i < nsamples; ++i)
    {
        GroundSample* ssmp = &es->start.gsamples[i];
        GroundSample* esmp = &es->end.gsamples[i];

        if ((ssmp->flags & 1) == 0 || (esmp->flags & 1) == 0)
            continue;

        const float u = (float) i / (float) (nsamples - 1);
        float spt[3], ept[3];
        vlerp(spt, es->start.p, es->start.q, u);
        vlerp(ept, es->end.p, es->end.q, u);


        spt[1] = ssmp->height;
        ept[1] = esmp->height;

        if (!SampleTrajectory(spt, ept, &es->trajectory))
            continue;
        ssmp->flags |= 4;
    }
}

bool AnnotationBuilder::SampleTrajectory(const float* pa, const float* pb, Trajectory2D* tra)
{
    using namespace VecMath;
    float p[3];

    for (int i = 0; i < tra->nsamples; ++i)
    {
        const TrajectorySample* s = &tra->samples[i];
        const float u = (float) i / (float) (tra->nsamples - 1);
        vlerp(p, pa, pb, u);
        if (CheckHeightfieldCollision(p[0], p[1] + s->ymin, p[1] + s->ymax, p[2]))
            return false;
    }

    return true;
}

inline bool overlapRange(const float amin, const float amax, const float bmin, const float bmax)
{
    return (amin > bmax || amax < bmin) ? false : true;
}

bool AnnotationBuilder::CheckHeightfieldCollision(const float x, const float ymin, const float ymax, const float z)
{
    rcHeightfield* m_solid = NULL;

    HashMap<Pair<int, int>, NavigationBuildData*>::Iterator it;
    for (it = navMesh_->builds_.Begin(); it != navMesh_->builds_.End(); it++)
    {
        if (it->second_ == NULL)
            continue;
        m_solid = it->second_->heightField_;
        if (m_solid == NULL)
            continue;

        const int w = m_solid->width;
        const int h = m_solid->height;
        const float cs = m_solid->cs;
        const float ch = m_solid->ch;
        const float* orig = m_solid->bmin;
        const int ix = (int) floorf((x - orig[0]) / cs);
        const int iz = (int) floorf((z - orig[2]) / cs);

        if (ix < 0 || iz < 0 || ix >= w || iz >= h)
            return false;

        if (m_solid->spans == NULL) return false;
        const rcSpan* s = m_solid->spans[ix + iz*w];
        if (s == NULL) return false;

        while (s)
        {
            const float symin = orig[1] + s->smin*ch;
            const float symax = orig[1] + s->smax*ch;
            if (overlapRange(ymin, ymax, symin, symax))
                return true;
            s = s->next;
        }

    }
    return false;
}

AnnotationBuilder::JumpLink* AnnotationBuilder::AddLink()
{
    if (nlinks_ + 1 > clinks_)
    {
        clinks_ = clinks_ ? clinks_ * 2 : 8;
        JumpLink* n = new JumpLink[clinks_];
        if (nlinks_)
            memcpy(n, links_, sizeof(JumpLink) *nlinks_);
        delete[] links_;
        links_ = n;
    }
    JumpLink* link = &links_[nlinks_++];
    return link;
}

void AnnotationBuilder::FilterJumpOverLinks()
{
    using namespace VecMath;

    // Filter out links which overlap
    const float thr = navMesh_->GetCellSize()*5.0f;

    for (int i = 0; i < nlinks_ - 1; ++i)
    {
        JumpLink* li = &links_[i];
        if (li->flags == 0)
            continue;
        const float* spi = &li->spine0[0];
        const float* sqi = &li->spine1[0];
        const float* epi = &li->spine0[(li->nspine - 1) * 3];
        const float* eqi = &li->spine1[(li->nspine - 1) * 3];

        for (int j = i + 1; j < nlinks_; ++j)
        {
            JumpLink* lj = &links_[j];
            if (lj->flags == 0)
                continue;
            const float* spj = &lj->spine0[0];
            const float* sqj = &lj->spine1[0];
            const float* epj = &lj->spine0[(lj->nspine - 1) * 3];
            const float* eqj = &lj->spine1[(lj->nspine - 1) * 3];

            const float d0 = distSegSegSqr(spi, sqi, epj, eqj);
            const float d1 = distSegSegSqr(epi, eqi, spj, sqj);
            //			const float d0 = min(distSegSegSqr(spi,sqi, spj,sqj), distSegSegSqr(spi,sqi, epj,eqj));
            //			const float d1 = min(distSegSegSqr(epi,eqi, epj,eqj), distSegSegSqr(epi,eqi, spj,sqj));

            //			printf("%d/%d  d0=%f d1=%f (%f)\n", i,j, sqrtf(d0), sqrtf(d1), thr);

            if (d0 < sqr(thr) && d1 < sqr(thr))
            {
                //				printf(" - merge!\n");
                if (vdistSqr(spi, sqi) > vdistSqr(spj, sqj))
                {
                    lj->flags = 0;
                }
                else
                {
                    li->flags = 0;
                    break;
                }
            }
        }
    }
}

}


