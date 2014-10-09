#include <math.h>
#include <float.h>
#include "vecmath.h"
#include <stdlib.h>

namespace VecMath
{
	void vnormalize_(float* v)
	{
		float d = 1.0f / sqrtf(sqr(v[0]) + sqr(v[1]) + sqr(v[2]));
		v[0] *= d;
		v[1] *= d;
		v[2] *= d;
	}

	void mmul(float* m, const float* a, const float* b)
	{
		for (int r = 0; r < 4; r++)
		for (int c = 0; c < 4; c++)
			m[r * 4 + c] = a[r * 4 + 0] * b[0 * 4 + c] + a[r * 4 + 1] * b[1 * 4 + c] + a[r * 4 + 2] * b[2 * 4 + c] + a[r * 4 + 3] * b[3 * 4 + c];
	}

	void midentity(float* m)
	{
		for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			m[i * 4 + j] = i == j ? 1 : 0;
	}

	void mtrans(float* m, float x, float y, float z)
	{
		midentity(m);
		m[12] = x;
		m[13] = y;
		m[14] = z;
	}

	void mscale(float* m, float x, float y, float z)
	{
		midentity(m);
		m[0] = x;
		m[5] = y;
		m[10] = z;
	}

	void mrotate(float* m, float a, float x, float y, float z)
	{
		midentity(m);

		const float cs = cosf(a);
		const float sn = sinf(a);

		const float omcs = 1 - cs;
		const float X2 = x * x;
		const float Y2 = y * y;
		const float Z2 = z * z;
		const float XYM = x * y * omcs;
		const float XZM = x * z * omcs;
		const float YZM = y * z * omcs;
		const float xSin = x * sn;
		const float ySin = y * sn;
		const float zSin = z * sn;

		m[0] = X2 * omcs + cs;
		m[1] = XYM - zSin;
		m[2] = XZM + ySin;

		m[4] = XYM + zSin;
		m[5] = Y2 * omcs + cs;
		m[6] = YZM - xSin;

		m[8] = XZM - ySin;
		m[9] = YZM + xSin;
		m[10] = Z2 * omcs + cs;
	}

	void mperspective(float* m, float fov, float aspect, float znear, float zfar)
	{
		float xmin, xmax, ymin, ymax;

		ymax = znear * tanf(fov / 2);
		ymin = -ymax;

		xmin = ymin * aspect;
		xmax = ymax * aspect;

		// Build frustum
		m[0] = (2 * znear) / (xmax - xmin);
		m[1] = 0;
		m[2] = 0;
		m[3] = 0;

		m[4] = 0;
		m[5] = (2 * znear) / (ymax - ymin);
		m[6] = 0;
		m[7] = 0;

		m[8] = (xmax + xmin) / (xmax - xmin);
		m[9] = (ymax + ymin) / (ymax - ymin);
		m[10] = -(zfar + znear) / (zfar - znear);
		m[11] = -1;

		m[12] = 0;
		m[13] = 0;
		m[14] = -(2 * zfar * znear) / (zfar - znear);
		m[15] = 0;
	}

	void mortho(float* m, float left, float right, float bottom, float top, float near, float far)
	{
		float tx = -(right + left) / (right - left);
		float ty = -(top + bottom) / (top - bottom);
		float tz = -(far + near) / (far - near);

		// Build frustum
		m[0] = 2 / (right - left);
		m[1] = 0;
		m[2] = 0;
		m[3] = 0;

		m[4] = 0;
		m[5] = 2 / (top - bottom);
		m[6] = 0;
		m[7] = 0;

		m[8] = 0;
		m[9] = 0;
		m[10] = -2 / (far - near);
		m[11] = 0;

		m[12] = tx;
		m[13] = ty;
		m[14] = tz;
		m[15] = 1;
	}

	void minverse(float* inv, const float* src)
	{
		int i, j, k;
		float t;
		float temp[4][4];

		for (i = 0; i < 4; i++)
		for (j = 0; j < 4; j++)
			temp[i][j] = src[i * 4 + j];
		midentity(inv);

		for (i = 0; i < 4; i++)
		{
			if (temp[i][i] == 0.0f)
			{
				for (j = i + 1; j < 4; j++)
				if (temp[j][i] != 0.0f)
					break;

				if (j != 4)
				{
					for (k = 0; k < 4; k++)
					{
						t = temp[i][k];
						temp[i][k] = temp[j][k];
						temp[j][k] = t;
						t = inv[i * 4 + k];
						inv[i * 4 + k] = inv[j * 4 + k];
						inv[j * 4 + k] = t;
					}
				}
				else
				{
					return;
				}
			}

			t = 1.0f / temp[i][i];
			for (k = 0; k < 4; k++)
			{
				temp[i][k] *= t;
				inv[i * 4 + k] *= t;
			}
			for (j = 0; j < 4; j++)
			{
				if (j != i)
				{
					t = temp[j][i];
					for (k = 0; k < 4; k++)
					{
						temp[j][k] -= temp[i][k] * t;
						inv[j * 4 + k] -= inv[i * 4 + k] * t;
					}
				}
			}
		}
	}

	void mlerp(float* m, const float* m0, const float* m1, const float u)
	{
		midentity(m);
		vlerp(&m[0], &m0[0], &m1[0], u);
		vlerp(&m[4], &m0[4], &m1[4], u);
		vlerp(&m[8], &m0[8], &m1[8], u);
		vlerp(&m[12], &m0[12], &m1[12], u);
		vnormalize_(&m[0]);
		vnormalize_(&m[4]);
		vnormalize_(&m[8]);
	}

	void transbox(float* rmin, float* rmax, const float* m, const float* bmin, const float* bmax)
	{
		vset(rmin, FLT_MAX, FLT_MAX, FLT_MAX);
		vset(rmax, -FLT_MAX, -FLT_MAX, -FLT_MAX);

		for (int i = 0; i < 8; ++i)
		{
			const float* bx = (i & 1) ? bmin : bmax;
			const float* by = (i & 2) ? bmin : bmax;
			const float* bz = (i & 4) ? bmin : bmax;
			float p[3], pt[3];
			vset(p, bx[0], by[1], bz[2]);
			transpoint(pt, m, p);
			vmin(rmin, pt);
			vmax(rmax, pt);
		}
	}

	void project(float x, float y, float z, const float* model, const float* proj,
		int w, int h, float* pos)
	{
		float A[16];
		float in[4], out[4];
		in[0] = x; in[1] = y; in[2] = z; in[3] = 1;
		mmul(A, model, proj);
		transpoint4(out, A, in);
		pos[0] = ((out[0] / out[3]) + 1)*w / 2;
		pos[1] = ((out[1] / out[3]) + 1)*h / 2;
		pos[2] = (out[2] / out[3]);
	}

	void unproject(float wx, float wy, float wz,
		const float* model, const float* proj,
		int w, int h, float* pos)
	{
		float m[16], A[16];
		float in[4], out[4];

		in[0] = (wx / w) * 2 - 1;
		in[1] = (wy / h) * 2 - 1;
		in[2] = wz * 2 - 1;
		in[3] = 1;

		mmul(A, model, proj);
		minverse(m, A);

		transpoint4(out, m, in);

		if (out[3] == 0.0f)
			return;
		pos[0] = out[0] / out[3];
		pos[1] = out[1] / out[3];
		pos[2] = out[2] / out[3];
	}

	int isectSegPlane(const float* sp, const float* sq, const float* p, float &t)
	{
		// Compute the t value for the directed line ab intersecting the plane
		float spq[3];
		vsub(spq, sp, sq);
		float intr = vdot(p, spq);
		float d = vdot(p, sp) + p[3];
		if (intr == 0.0f) return 0;
		t = d / intr;
		// If t in [0..1] compute and return intersection point
		if (t >= 0.0f) // && t <= 1.0f)
			return 1;
		return 0;
	}

	float getClosestPtPtSeg(const float* pt, const float* sp, const float* sq)
	{
		float dir[3], diff[3];
		vsub(dir, sq, sp);
		vsub(diff, pt, sp);
		float t = vdot(dir, diff);
		if (t <= 0.0f) return 0.0f;
		const float d = vdot(dir, dir);
		if (t >= d) return 1.0f;
		return t / d;
	}

	void closestPtSegSeg(const float* ap, const float* aq,
		const float* bp, const float* bq,
		float& s, float& t)
	{
		const float EPSILON = 1e-6f;
		float d1[3], d2[3], r[3];
		vsub(d1, aq, ap);
		vsub(d2, bq, bp);
		vsub(r, ap, bp);
		const float a = vdot(d1, d1); // Squared length of segment S1, always nonnegative
		const float e = vdot(d2, d2); // Squared length of segment S2, always nonnegative
		const float f = vdot(d2, r);

		// Check if either or both segments degenerate into points
		if (a <= EPSILON && e <= EPSILON)
		{
			// Both segments degenerate into points
			s = t = 0.0f;
			return;
		}
		if (a <= EPSILON)
		{
			s = 0.0f;
			t = f / e;
			t = clamp(t, 0.0f, 1.0f);
		}
		else
		{
			const float c = vdot(d1, r);
			if (e <= EPSILON)
			{
				// Second segment degenerates into a point
				t = 0.0f;
				s = clamp(-c / a, 0.0f, 1.0f); // t = 0 => s = (b*t - c) / a = -c / a
			}
			else
			{
				// The general nondegenerate case starts here
				const float b = vdot(d1, d2);
				const float denom = a*e - b*b; // Always nonnegative

				// If segments not parallel, compute closest point on L1 to L2, and
				// clamp to segment S1. Else pick arbitrary s (here 0)
				if (denom != 0.0f)
					s = clamp((b*f - c*e) / denom, 0.0f, 1.0f);
				else
					s = 0.0f;

				// Compute point on L2 closest to S1(s) using
				// t = Dot((p.start+D1*s)-q.start,D2) / Dot(D2,D2) = (b*s + f) / e
				t = (b*s + f) / e;

				// If t in [0,1] done. Else clamp t, recompute s for the new value
				// of t using s = Dot((q.start+D2*t)-p.start,D1) / Dot(D1,D1)= (t*b - c) / a
				// and clamp s to [0, 1]
				if (t < 0.0f)
				{
					t = 0.0f;
					s = clamp(-c / a, 0.0f, 1.0f);
				}
				else if (t > 1.0f)
				{
					t = 1.0f;
					s = clamp((b - c) / a, 0.0f, 1.0f);
				}
			}
		}
	}

	float distSegSegSqr(const float* ap, const float* aq,
		const float* bp, const float* bq)
	{
		float s, t;
		closestPtSegSeg(ap, aq, bp, bq, s, t);
		float anp[3], bnp[3];
		vlerp(anp, ap, aq, s);
		vlerp(bnp, bp, bq, t);
		return vdistSqr(anp, bnp);
	}

	float getNearestDistOnPath(const float* pt, const float* path, int npath)
	{
		if (npath < 2) return 0;
		float nearest = FLT_MAX;
		float totd = 0;
		float ret = 0;
		for (int i = 0; i < npath - 1; ++i)
		{
			const float* sp = &path[i * 3];
			const float* sq = &path[(i + 1) * 3];
			float t = getClosestPtPtSeg(pt, sp, sq);
			float pos[3];
			vlerp(pos, sp, sq, t);
			const float dsqr = vdistSqr(pt, pos);
			const float sd = sqrtf(vdistSqr(sp, sq));
			if (dsqr < nearest)
			{
				nearest = dsqr;
				ret = totd + sd*t;
			}
			totd += sd;
		}
		return ret;
	}

	float getPathLen(const float* path, int npath)
	{
		if (npath < 2) return 0;
		float totd = 0;
		for (int i = 0; i < npath - 1; ++i)
		{
			const float* sp = &path[i * 3];
			const float* sq = &path[(i + 1) * 3];
			totd += sqrtf(vdistSqr(sp, sq));
		}
		return totd;
	}

	void getPointAlongPath(float dist, const float* path, int npath, float* pt, float* dir)
	{
		if (npath == 0) return;
		if (npath == 1)
		{
			if (dir)
				vset(dir, 1, 0, 0);
			vcopy(pt, path);
			return;
		}
		if (dist <= 0)
		{
			if (dir)
				vsub(dir, &path[1 * 3], &path[0 * 3]);
			vcopy(pt, path);
			return;
		}
		float totd = 0;
		for (int i = 0; i < npath - 1; ++i)
		{
			const float* sp = &path[i * 3];
			const float* sq = &path[(i + 1) * 3];
			const float sd = sqrtf(vdistSqr(sp, sq));
			if (dist >= totd && dist <= totd + sd)
			{
				vlerp(pt, sp, sq, (dist - totd) / sd);
				if (dir)
					vsub(dir, sq, sp);
				return;
			}
			totd += sd;
		}
		vcopy(pt, &path[(npath - 1) * 3]);
		if (dir)
			vsub(dir, &path[(npath - 1) * 3], &path[(npath - 2) * 3]);
	}

	bool isectSegTriangle(const float* sp, const float* sq,
		const float* a, const float* b, const float* c,
		float &t)
	{
		float v, w;
		float ab[3], ac[3], qp[3], ap[3], norm[3], e[3];
		vsub(ab, b, a);
		vsub(ac, c, a);
		vsub(qp, sp, sq);

		// Compute triangle normal. Can be precalculated or cached if
		// intersecting multiple segments against the same triangle
		vcross(norm, ab, ac);

		// Compute denominator d. If d <= 0, segment is parallel to or points
		// away from triangle, so exit early
		float d = vdot(qp, norm);
		if (d <= 0.0f) return false;

		// Compute intersection t value of pq with plane of triangle. A ray
		// intersects iff 0 <= t. Segment intersects iff 0 <= t <= 1. Delay
		// dividing by d until intersection has been found to pierce triangle
		vsub(ap, sp, a);
		t = vdot(ap, norm);
		if (t < 0.0f) return false;
		//	if (t > d) return false; // For segment; exclude this code line for a ray test

		// Compute barycentric coordinate components and test if within bounds
		vcross(e, qp, ap);
		v = vdot(ac, e);
		if (v < 0.0f || v > d) return false;
		w = -vdot(ab, e);
		if (w < 0.0f || v + w > d) return false;

		// Segment/ray intersects triangle. Perform delayed division
		t /= d;

		return true;
	}

	// Returns true if 'c' is left of line 'a'-'b'.  
	inline bool left(const float* a, const float* b, const float* c)
	{
		const float u1 = b[0] - a[0];
		const float v1 = b[2] - a[2];
		const float u2 = c[0] - a[0];
		const float v2 = c[2] - a[2];
		return u1 * v2 - v1 * u2 < 0;
	}

	// Returns true if 'a' is more lower-left than 'b'.  
	inline bool cmppt(const float* a, const float* b)
	{
		if (a[0] < b[0]) return true;
		if (a[0] > b[0]) return false;
		if (a[2] < b[2]) return true;
		if (a[2] > b[2]) return false;
		return false;
	}

	// Calculates convex hull on xz-plane of points on 'pts',  
	// stores the indices of the resulting hull in 'out' and  
	// returns number of points on hull. 
	int convexhull(const float* pts, int npts, int* out)
	{
		// Find lower-leftmost point. 
		int hull = 0;
		for (int i = 1; i < npts; ++i)
		if (cmppt(&pts[i * 3], &pts[hull * 3]))
			hull = i;
		// Gift wrap hull.
		int endpt = 0;
		int i = 0;
		do
		{
			out[i++] = hull;
			endpt = 0;
			for (int j = 1; j < npts; ++j)
			if (hull == endpt || left(&pts[hull * 3], &pts[endpt * 3], &pts[j * 3]))
				endpt = j;
			hull = endpt;
		} while (endpt != out[0] && i < npts);
		return i;
	}


	bool isectSegAABB(const float* sp, const float* sq,
		const float* amin, const float* amax,
		float& tmin, float& tmax)
	{
		static const float EPS = 1e-6f;

		float d[3];
		vsub(d, sq, sp);
		tmin = 0;  // set to -FLT_MAX to get first hit on line
		tmax = FLT_MAX;		// set to max distance ray can travel (for segment)

		// For all three slabs
		for (int i = 0; i < 3; i++)
		{
			if (fabsf(d[i]) < EPS)
			{
				// Ray is parallel to slab. No hit if origin not within slab
				if (sp[i] < amin[i] || sp[i] > amax[i])
					return false;
			}
			else
			{
				// Compute intersection t value of ray with near and far plane of slab
				const float ood = 1.0f / d[i];
				float t1 = (amin[i] - sp[i]) * ood;
				float t2 = (amax[i] - sp[i]) * ood;
				// Make t1 be intersection with near plane, t2 with far plane
				if (t1 > t2) swap(t1, t2);
				// Compute the intersection of slab intersections intervals
				if (t1 > tmin) tmin = t1;
				if (t2 < tmax) tmax = t2;
				// Exit with no collision as soon as slab intersection becomes empty
				if (tmin > tmax) return false;
			}
		}

		return true;
	}

}

