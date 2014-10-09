
#ifndef VECMATH_H
#define VECMATH_H

namespace VecMath
{
/// \todo use urho3d functions 

#define M_PI       3.14159265358979323846
	// Common helper functions
	template<class T> inline void swap(T& a, T& b) { T t = a; a = b; b = t; }
	template<class T> inline T min(T a, T b) { return a < b ? a : b; }
	template<class T> inline T max(T a, T b) { return a > b ? a : b; }
	template<class T> inline T abs(T a) { return a < 0 ? -a : a; }
	template<class T> inline T sqr(T a) { return a*a; }
	template<class T> inline T clamp(T v, T mn, T mx) { return v < mn ? mn : (v > mx ? mx : v); }

	inline float lerp(float a, float b, float t) { return a + (b - a)*t; }

	inline float easeOutElastic(float t)
	{
		const float p = 1;//0.6f;
		return 1 + powf(2, -10 * t) * sinf((t - p / 4)*(2 * M_PI) / p);
	}

	inline float easeOutBounce(float t)
	{
		const float p = 1;//0.6f;
		const float v = 1 + powf(2, -10 * t) * sinf((t - p / 4)*(2 * M_PI) / p);
		return v > 1 ? 2 - v : v;
	}

	static const float degToRad = (float)M_PI / 180.0f;

	inline void vcross(float* dest, const float* v1, const float* v2)
	{
		dest[0] = v1[1] * v2[2] - v1[2] * v2[1];
		dest[1] = v1[2] * v2[0] - v1[0] * v2[2];
		dest[2] = v1[0] * v2[1] - v1[1] * v2[0];
	}

	inline float vdot(const float* v1, const float* v2)
	{
		return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
	}

	inline void vsub(float* dest, const float* v1, const float* v2)
	{
		dest[0] = v1[0] - v2[0];
		dest[1] = v1[1] - v2[1];
		dest[2] = v1[2] - v2[2];
	}

	inline void vadd(float* dest, const float* v1, const float* v2)
	{
		dest[0] = v1[0] + v2[0];
		dest[1] = v1[1] + v2[1];
		dest[2] = v1[2] + v2[2];
	}

	inline void vmad(float* dest, const float* v1, const float* v2, float a)
	{
		dest[0] = v1[0] + v2[0] * a;
		dest[1] = v1[1] + v2[1] * a;
		dest[2] = v1[2] + v2[2] * a;
	}

	inline void vscale(float* dest, const float* src, float s)
	{
		dest[0] = src[0] * s;
		dest[1] = src[1] * s;
		dest[2] = src[2] * s;
	}

	inline void vlerp(float* dest, const float* v1, const float* v2, float t)
	{
		dest[0] = v1[0] + (v2[0] - v1[0])*t;
		dest[1] = v1[1] + (v2[1] - v1[1])*t;
		dest[2] = v1[2] + (v2[2] - v1[2])*t;
	}

	inline void vmin(float* mn, const float* v)
	{
		mn[0] = min(mn[0], v[0]);
		mn[1] = min(mn[1], v[1]);
		mn[2] = min(mn[2], v[2]);
	}

	inline void vmax(float* mx, const float* v)
	{
		mx[0] = max(mx[0], v[0]);
		mx[1] = max(mx[1], v[1]);
		mx[2] = max(mx[2], v[2]);
	}

	inline void vcopy(float* dest, const float* v)
	{
		dest[0] = v[0];
		dest[1] = v[1];
		dest[2] = v[2];
	}

	inline void vneg(float* dest, const float* v)
	{
		dest[0] = -v[0];
		dest[1] = -v[1];
		dest[2] = -v[2];
	}

	inline void vzero(float* dest)
	{
		dest[0] = 0;
		dest[1] = 0;
		dest[2] = 0;
	}

	inline float vdistSqr(const float* v1, const float* v2)
	{
		float dx = v2[0] - v1[0];
		float dy = v2[1] - v1[1];
		float dz = v2[2] - v1[2];
		return dx*dx + dy*dy + dz*dz;
	}

	inline float vdist2dSqr(const float* v1, const float* v2)
	{
		float dx = v2[0] - v1[0];
		float dz = v2[2] - v1[2];
		return dx*dx + dz*dz;
	}

	void vnormalize_(float* v);

	inline bool vequal(const float* p0, const float* p1)
	{
		static const float thr = sqr(1.0f / 16384.0f);
		const float d = vdistSqr(p0, p1);
		return d < thr;
	}

	inline void vset(float* v, float x, float y, float z)
	{
		v[0] = x; v[1] = y; v[2] = z;
	}

	inline float vlenSqr(const float* v)
	{
		return vdot(v, v);
	}

	inline bool pointInBox(const float* p, const float* bmin, const float* bmax)
	{
		return	p[0] >= bmin[0] && p[0] <= bmax[0] &&
			p[1] >= bmin[1] && p[1] <= bmax[1] &&
			p[2] >= bmin[2] && p[2] <= bmax[2];
	}

	inline bool overlapBounds(const float* amin, const float* amax, const float* bmin, const float* bmax)
	{
		bool overlap = true;
		overlap = (amin[0] > bmax[0] || amax[0] < bmin[0]) ? false : overlap;
		overlap = (amin[1] > bmax[1] || amax[1] < bmin[1]) ? false : overlap;
		overlap = (amin[2] > bmax[2] || amax[2] < bmin[2]) ? false : overlap;
		return overlap;
	}

	inline bool overlapBoundsXZ(const float* amin, const float* amax, const float* bmin, const float* bmax)
	{
		bool overlap = true;
		overlap = (amin[0] > bmax[0] || amax[0] < bmin[0]) ? false : overlap;
		overlap = (amin[2] > bmax[2] || amax[2] < bmin[2]) ? false : overlap;
		return overlap;
	}


	void mmul(float* rest, const float* a, const float* b);
	void midentity(float* m);
	void mtrans(float* m, float x, float y, float z);
	void mscale(float* m, float x, float y, float z);
	void mrotate(float* m, float a, float x, float y, float z);
	void mperspective(float* m, float fov, float aspect, float znear, float zfar);
	void mortho(float* m, float left, float right, float bottom, float top, float near, float far);
	void minverse(float* m, const float* a);
	void mlerp(float* m, const float* m0, const float* m1, const float u);

	inline void transvec(float* r, const float* m, const float* v)
	{
		r[0] = v[0] * m[0] + v[1] * m[4] + v[2] * m[8];
		r[1] = v[0] * m[1] + v[1] * m[5] + v[2] * m[9];
		r[2] = v[0] * m[2] + v[1] * m[6] + v[2] * m[10];
	}

	inline void transpoint(float* r, const float* m, const float* v)
	{
		r[0] = v[0] * m[0] + v[1] * m[4] + v[2] * m[8] + m[12];
		r[1] = v[0] * m[1] + v[1] * m[5] + v[2] * m[9] + m[13];
		r[2] = v[0] * m[2] + v[1] * m[6] + v[2] * m[10] + m[14];
	}

	inline void transpoint4(float* r, const float* m, const float* v)
	{
		r[0] = v[0] * m[0] + v[1] * m[4] + v[2] * m[8] + v[3] * m[12];
		r[1] = v[0] * m[1] + v[1] * m[5] + v[2] * m[9] + v[3] * m[13];
		r[2] = v[0] * m[2] + v[1] * m[6] + v[2] * m[10] + v[3] * m[14];
		r[3] = v[0] * m[3] + v[1] * m[7] + v[2] * m[11] + v[3] * m[15];
	}

	void transbox(float* rmin, float* rmax, const float* m, const float* bmin, const float* bmax);

	void project(float x, float y, float z, const float* model, const float* proj,
		int w, int h, float* pos);

	void unproject(float wx, float wy, float wz, const float* model, const float* proj,
		int w, int h, float* pos);

	float getClosestPtPtSeg(const float* pt, const float* sp, const float* sq);
	float getNearestDistOnPath(const float* pt, const float* path, int npath);
	void getPointAlongPath(float dist, const float* path, int npath, float* pt, float* dir = 0);
	float getPathLen(const float* path, int npath);

	int isectSegPlane(const float* sp, const float* sq, const float* p, float &t);

	bool isectSegTriangle(const float* sp, const float* sq,
		const float* a, const float* b, const float* c,
		float &t);

	bool isectSegAABB(const float* sp, const float* sq,
		const float* amin, const float* amax,
		float& tmin, float& tmax);

	void closestPtSegSeg(const float* ap, const float* aq,
		const float* bp, const float* bq,
		float& s, float& t);

	float distSegSegSqr(const float* ap, const float* aq,
		const float* bp, const float* bq);

	int convexhull(const float* pts, int npts, int* out);

}
#endif // VECMATH_H