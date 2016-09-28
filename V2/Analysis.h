#pragma once
#include "VectorField.h"
#include "Analysis2D.h"
#include "MathUtil.h"



template<typename T>
int locateSingularityPoint2D(VectorField<T> &VF, const int face_id, const T v1[2], const T v2[2],const T v3[2],
	const T A[2], const T B[2], const T C[2], vector<T>  &p, vector<T> &alpha,vector<T> &equation)
{
	if (isSigularity(v1[0], v1[1], v2[0], v2[1], v3[0], v3[1]) == 0) return 0;
	T x1 = A[0], y1 = A[1], x2 = B[0], y2 = B[1], x3 = C[0], y3 = C[1];
	T abc[3], def[3];
	cal_linear_field(v1[0], v1[1], v2[0], v2[1], v3[0], v3[1], x1, y1, x2, y2, x3, y3, abc, def);

//	a = abc[0]; b = abc[1]; c = abc[2];   d = def[0]; e = def[1]; f = def[2];
	T x, y;
	findIntersection(abc[0], abc[1], abc[2], def[0], def[1], def[2], x, y);

	p[0] = x; p[1] = y;
//	T p[2] = { x, y }, A[2] = { x1,y1 }, B[2] = { x2,y2, }, C[2] = { x3,y3 };
	T pp[2] = { x,y };
	if (!is_inside_tri(pp, A, B, C))return 0;
	int type = get_sing_type(abc[0], abc[1], def[0], def[1]);
	vector<T> p0 = { x, y }, a0 = { x1,y1 }, b0 = { x2,y2, }, c0 = { x3,y3 };
	Get2DBarycentricFacters(pp, A,B,C, alpha);
	equation[0] = abc[0]; equation[1] = abc[1]; equation[2] = abc[2];
	equation[3] = def[0]; equation[4] = def[1]; equation[5] = def[2];
	return type;
}

template<typename T>
int locateSingularityPoint(VectorField<T> &VF, const int face_id, vector<T>  &p, vector<T> &alpha, vector<T> &equation) {
	int i = VF.F(face_id, 0), j = VF.F(face_id, 1), k = VF.F(face_id, 2);
	const int dim = VF.VF.cols();
	if (dim == 2) {
		T v1[2] = { VF.VF(i,0),VF.VF(i,1) }, v2[2] = { VF.VF(j,0),VF.VF(j,1) }, v3[2] = { VF.VF(k,0),VF.VF(k,1) };
		T A[2]  = { VF.V(i,0),VF.V(i,1) },    B[2] = { VF.V(j,0),VF.V(j,1) },    C[2] = { VF.V(k,0),VF.V(k,1) };	
		return locateSingularityPoint2D(VF, face_id, v1, v2, v3, A, B, C, p, alpha, equation);
	}
	return 0;
}

template<typename T>
void VF_Analysis(VectorField<T> &VF, std::vector<Singularity<T>> &singularities)
{
	int type; vector<T> alpha(3), p(2);
	vector<T> equation(6);
	Singularity<T> singular;
	const int dim = VF.VF.cols();
	singular.eigenVectorL.resize(dim); singular.eigenVectorS.resize(dim);
	for (int f = 0; f < VF.F.rows(); f++) {		
		int type = locateSingularityPoint(VF, f, p,alpha, equation);
		if (type > 0) {
			singular.face_id = f; singular.pos = p; singular.alpha = alpha; singular.type = type;
		//	std::cout << Tri.x << ", " << Tri.y << ", " << Tri.type << std::endl;
			if (singular.type == 3) {//sandle
				
				findEigenVector(equation[0], equation[1], equation[3], equation[4], singular.eigenVectorL, singular.eigenVectorS);
				
			}
			singularities.push_back(singular);
		}
	}
}