#pragma once

#include <Eigen/Dense>

//=============find barycentric-coordinates===================================
//http://gamedev.stackexchange.com/questions/23743/whats-the-most-efficient-way-to-find-barycentric-coordinates

template <typename T>
T dot(const vector<T> &a, const vector<T> &b) {
	T d = 0;
	for (int i = 0; i < a.size(); i++) d += a[i] * b[i];
	return d;
}
template <typename T>
vector<T> sub(const vector<T> &b, const vector<T> &a) {
	vector<T> s(a.size());
	for (int i = 0; i < a.size(); i++) s[i] = b[i] - a[i];
	return s;
}

template <typename T>
inline bool Get2DBarycentricFacters(const vector<T> &p, const vector<T> &a, const vector<T> &b,
	const vector<T> &c , vector<T> &alpha)
{
	vector<T> v0 = sub(b,a), v1 = sub(c,a), v2 = sub(p,a);
	T d00 = dot(v0,v0);
	T d01 = dot(v0,v1);
	T d11 = dot(v1,v1);
	T d20 = dot(v2,v0);
	T d21 = dot(v2,v1);
	T denom = d00*d11 - d01*d01;  	denom = 1. / denom;

	// compute parametric coordinates
	T v = (d11 * d20 - d01 * d21) * denom;
	T w = (d00 * d21 - d01 * d20) * denom;
	T u = 1.0f - v - w;
	alpha[0] = u; alpha[1] = v; alpha[2] = w;
	return v >= 0. && w >= 0. && v + w <= 1.;
}

template <typename T>
inline bool Get2DBarycentricFacters(const T p[2], const T a[2], const T b[2], const T c[2], vector<T> &alpha)
{
	vector<T> P = { p[0],p[1] }, A = { a[0],a[1] }, B = { b[0],b[1] }, C = { c[0],c[1] };
	return Get2DBarycentricFacters(P, A, B, C, alpha);
}


template <typename T,typename PointType>
inline bool Get2DBarycentricFacters(const PointType p, const PointType a, const PointType b, 
	const PointType c, vector<T> &alpha)
{
	PointType v0 = b - a, v1 = c - a, v2 = p - a;
	T d00 = v0*v0;
	T d01 = v0*v1;
	T d11 = v1*v1;
	T d20 = v2*v0;
	T d21 = v2*v1;
	T denom = d00*d11 - d01*d01;  	denom = 1. / denom;

	// compute parametric coordinates
	T v = (d11 * d20 - d01 * d21) * denom;
	T w = (d00 * d21 - d01 * d20) * denom;
	T u = 1.0f - v - w;
	alpha[0] = u; alpha[1] = v; alpha[2] = w;
	return v >= 0. && w >= 0. && v + w <= 1.;
}

template <typename T>
inline bool Get2DBarycentricFacters(const Matrix< T, 1, Dynamic> p, const Matrix< T, 1, Dynamic> a,
	const Matrix< T, 1, Dynamic> b,	const Matrix< T, 1, Dynamic> c, vector<T> &alpha)
{
	typedef Matrix< T, 1, Dynamic> point_type;

	point_type v0 = b - a, v1 = c - a, v2 = p - a;
	T d00 = v0*v0.adjoint();
	T d01 = v0*v1.adjoint();
	T d11 = v1*v1.adjoint();
	T d20 = v2*v0.adjoint();
	T d21 = v2*v1.adjoint();
	T denom = d00*d11 - d01*d01;  	denom = 1. / denom;

	// compute parametric coordinates
	T v = (d11 * d20 - d01 * d21) * denom;
	T w = (d00 * d21 - d01 * d20) * denom;
	T u = 1.0f - v - w;

	alpha[0] = u; alpha[1] = v; alpha[2] = w;
	return v >= 0. && w >= 0. && v + w <= 1.;
}

template <typename T>
bool GetBarycentricFacters(const Matrix< T, 1, Dynamic> p, const Matrix< T, 1, Dynamic> a,
	const Matrix< T, 1, Dynamic> b,
	const Matrix< T, 1, Dynamic> c, vector<T> &alpha)
{
	const int dim = p.cols();
	if (dim == 2) {
		return Get2DBarycentricFacters(p, a, b, c, alpha);
	}
	else {
		Matrix< T, 1, 3> ab = b - a, ac = c - a;
		Matrix< T, 1, 3> abc = ab.cross(ac);
		Matrix< T, 1, 3> normal = abc.normalized();
		// The area of a triangle is 
		T areaABC = normal*abc.adjoint();  //DOT(normal, abc);// CROSS((b - a), (c - a)));
		Matrix< T, 1, 3> pb = b - p, pc = c - p;
		Matrix< T, 1, 3> pbc = pb.cross(pc);
		Matrix< T, 1, 3> pa = a - p;
		Matrix< T, 1, 3> pca = pc.cross(pa);

		T areaPBC = normal*pbc.adjoint();  //DOT(normal, CROSS((b - P), (c - P)));
		T areaPCA = normal*pca.adjoint();  //DOT(normal, CROSS((c - P), (a - P)));

		alpha[0] = areaPBC / areaABC;
		alpha[1] = areaPCA / areaABC;
		alpha[2] = 1.0f - alpha[0] - alpha[1];
		/*
		bary.x = areaPBC / areaABC; // alpha
		bary.y = areaPCA / areaABC; // beta
		bary.z = 1.0f - bary.x - bary.y; // gamma
		*/
		return alpha[0] >= 0. && alpha[1] >= 0. && alpha[2] >= 0.;
	}
}

template <typename T>
inline void Get2DVectorAtPoint(const T va[2], const T vb[2], const T vc[2],
	const double alpha[3], T v[2])
{
	v[0] = alpha[0] * va[0] + alpha[1] * vb[0] + alpha[2] * vc[0];
	v[1] = alpha[0] * va[1] + alpha[1] * vb[1] + alpha[2] * vc[1];
}
template <typename T>
inline void Get2DVectorAtPoint(const Matrix< T, 1, Dynamic> &va, const Matrix< T, 1, Dynamic> &vb,
	const Matrix< T, 1, Dynamic> &vc, const double alpha[3], Matrix< T, 1, Dynamic> &v)
{
	v = alpha[0] * va + alpha[1] * vb + alpha[2] * vc;
}


//==================detect-where-two-line-segments-intersect=====================
//http://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
/*
meaning of return value
0----Intersection dosn't exists
1----Intersection exists.
2----two line segments are parallel.
3----two line segments are collinear, but not overlap.
4----two line segments are collinear, and share one same end point.
5----two line segments are collinear, and overlap.
*/

template<typename T>
int Intersection2LineSegments(T PointA[2], T PointB[2], T PointC[2], T PointD[2], T t[2])
{
	T delta;
	T t1, t2;
	T a, b, c, d;
	T xba, yba, xdc, ydc, xca, yca;

	xba = PointB[0] - PointA[0];    yba = PointB[1] - PointA[1];
	xdc = PointD[0] - PointC[0];    ydc = PointD[1] - PointC[1];
	xca = PointC[0] - PointA[0];    yca = PointC[1] - PointA[1];

	delta = xba*ydc - yba*xdc;
	t1 = xca*ydc - yca*xdc;
	t2 = xca*yba - yca*xba;

	if (delta != 0)
	{
		t[0] = t1 / delta;   t[1] = t2 / delta;
		//two segments intersect (including intersect at end points)
		//if ( t[0]<=1 && t[0]>=0 && t[1]<=1 && t[1]>=0 ) return 1;
		if (t[0] <= 1 && (t[0] >= 0 || fabs(t[0]) <= 1.e-8)
			&& t[1] <= 1 && (t[1] >= 0 || fabs(t[1]) <= 1.e-8)) //set threshold to allow some numerical errors
			return 1;
		else return 0;
	}

	else
	{
		// AB & CD are parallel. 
		if ((t1 != 0) && (t2 != 0)) return 2;

		// when AB & CD are collinear 

		//if AB isn't a vertical line segment, project to x-axis 
		if (PointA[0] != PointB[0])
		{
			a = std::min(PointA[0], PointB[0]); b = std::max(PointA[0], PointB[0]);
			c = std::min(PointC[0], PointD[0]); d = std::max(PointC[0], PointD[0]);

			if ((d<a) || (c>b)) return  3;
			else if ((d == a) || (c == b)) return 4;
			else return 5;
		}

		else         // if AB is a vertical line segment, project to y-axis 
		{

			a = std::min(PointA[1], PointB[1]); b = std::max(PointA[1], PointB[1]);
			c = std::min(PointC[1], PointD[1]); d = std::max(PointC[1], PointD[1]);

			if ((d<a) || (c>b)) return  3;
			else if ((d == a) || (c == b)) return 4;
			else return 5;
		}
	}
}

template<typename T>
//int GetIntersection2(const Eigen::Matrix<T, 1, -1> &p, const Eigen::Matrix<T, 1, -1> &p2,
int Intersection2LineSegments(const Eigen::Matrix<T, 1, -1> &p, const Eigen::Matrix<T, 1, -1> &p2,
	const Eigen::Matrix<T, 1, -1> &q1, const Eigen::Matrix<T, 1, -1> &q2, T t[2])
{
	T A[2] = { p(0),p(1) }, B[2] = { p2(0),p2(1) }, C[2] = { q1(0),q1(1) }, D[2] = { q2(0),q2(1) };
	return Intersection2LineSegments(A, B, C, D, t);
}