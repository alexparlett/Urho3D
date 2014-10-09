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

#include "Object.h"



namespace Urho3D
{

	class NavigationMesh;
	class DebugRenderer;

	enum AnnotationBuilderDrawFlags
	{
		DRAW_WALKABLE_SURFACE = 1 << 0,
		DRAW_WALKABLE_BORDER = 1 << 1,
		DRAW_SELECTED_EDGE = 1 << 2,
		DRAW_ANIM_TRAJECTORY = 1 << 3,
		DRAW_LAND_SAMPLES = 1 << 4,
		DRAW_COLLISION_SLICES = 1 << 5,
		DRAW_ANNOTATIONS = 1 << 6,
	};

	enum SampleType
	{
		EDGE_JUMP_DOWN,
		EDGE_JUMP_OVER,
	};

	struct AnnotationBuilderConfig
	{
		float agentRadius;
		float agentHeight;
		float agentClimb;
		float cellSize;
		float cellHeight;
	};

	/// Annotation Builder to create jump links or coverpoints.
	class URHO3D_API AnnotationBuilder : public Object
	{
		OBJECT(AnnotationBuilder);
	public:
		/// Construct.
		AnnotationBuilder(Context* context);
		/// Destruct.
		virtual ~AnnotationBuilder();
		/// Register object factory.
		static void RegisterObject(Context* context);

		///
		void ClearLinks();
		///
		bool Build(const AnnotationBuilderConfig& acfg, NavigationMesh *navMesh);
		///
		void BuildAllEdges(int type);
		///
		void BuildNearestEdge(int type, const float* pos);
		/// 
		void DrawDebug(DebugRenderer* debug, bool depthTest, unsigned int flags = DRAW_WALKABLE_BORDER);

	private:
		static const int MAX_SPINE = 8;
		struct Edge
		{
			float sp[3], sq[3];
		};
		struct TrajectorySample
		{
			float x, ymin, ymax;
		};
		struct Trajectory2D
		{
			Trajectory2D() : nspine(0), samples(0), nsamples(0) {}
			~Trajectory2D() 
			{ 
				if (samples)
				delete[] samples; 
			}
			float spine[2 * MAX_SPINE];
			int nspine;
			TrajectorySample* samples;
			int nsamples;
		};
		struct GroundSample
		{
			float height;
			unsigned char flags;
		};
		struct GroundSegment
		{
			inline GroundSegment() : gsamples(0), ngsamples(0) {}
			inline ~GroundSegment() 
			{
				if (gsamples)		
				delete[] gsamples; 
			}

			float p[3], q[3];
			GroundSample* gsamples;
			int ngsamples;
			int npass;
		};
		struct EdgeSampler
		{
			GroundSegment start;
			GroundSegment end;

			float groundRange;

			Trajectory2D trajectory;

			float rigp[3], rigq[3];
			float ax[3], ay[3], az[3];
		};
		struct JumpLink
		{
			float spine0[MAX_SPINE * 3];
			float spine1[MAX_SPINE * 3];
			int nspine;
			unsigned char flags;
		};


		void Cleanup();
		EdgeSampler* SampleEdge(int type, const float* sp, const float* sq);
		void AddEdgeLinks(EdgeSampler* es);
		void InitJumpDownRig(EdgeSampler* es, const float* sp, const float* sq,
			const float jumpStartDist, const float jumpEndDist,
			const float jumpDownDist, const float groundRange);
		void InitTrajectory(Trajectory2D* tra);
		void SampleGroundSegment(GroundSegment* seg, const float nsamples, const float groundRange);
		bool GetCompactHeightfieldHeigh(const float* pt, const float hrange, float* height);
		void SampleAction(EdgeSampler* es);
		bool SampleTrajectory(const float* pa, const float* pb, Trajectory2D* tra);
		bool CheckHeightfieldCollision(const float x, const float ymin, const float ymax, const float z);
		JumpLink* AddLink();
		void FilterJumpOverLinks();

		AnnotationBuilderConfig acfg_;
		WeakPtr<NavigationMesh> navMesh_;

		int selEdge_;
		int nedges_;
		/// \todo use PODVector ??
		Edge* edges_;
		EdgeSampler* sampler_;
		JumpLink* links_;
		int nlinks_;
		int clinks_;
	};

}