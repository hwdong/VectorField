#pragma once
//https://eigen.tuxfamily.org/dox/classEigen_1_1EigenSolver.html
//http://stackoverflow.com/questions/14066933/direct-way-of-computing-clockwise-angle-between-2-vectors


#include <Eigen/Eigenvalues>
#include <iostream>
#include <vector>
using namespace std;

// 1.----------check sigular triangle by computing poincare index-----------  
template<typename T>
T angle2vector(const T x1, const T y1, const T x2, const T y2) {
	T dot = x1*x2 + y1*y2;      // dot product
	T det = x1*y2 - y1*x2;      // determinant
	T angle = atan2(det, dot);  // atan2(y, x) or atan2(sin, cos)
	return angle;
}

template<typename T>
int isSigularity(const T x1, const T y1, const T x2, const T y2, const T x3, const T y3) 
{
#if 1
	T angle = angle2vector(x1, y1, x2, y2) +  angle2vector(x2, y2, x3, y3) + angle2vector(x3, y3, x1, y1);
	T threshold = 2 * M_PI-0.1;
	if (angle > threshold)  return 1;
	else if (angle <-threshold)  return -1;
	return 0;
#else
	double ang_sum = 0;
	double vec_ang[3], theta[3];

	vec_ang[0] = atan2(y1, x1);
	vec_ang[1] = atan2(y2, x2);
	vec_ang[2] = atan2(y3, x3);

	for (int i = 0; i < 3; i++)
	{
		theta[i] = vec_ang[(i + 1) % 3] - vec_ang[i];

		if (theta[i] < -M_PI)
			theta[i] += 2 * M_PI;

		if (theta[i] > M_PI)
			theta[i] -= 2 * M_PI;

		ang_sum += theta[i];
	}
	
	T threshold = 2 * M_PI - 0.1;
	if (ang_sum > threshold)  return 1;
	else if (ang_sum <-threshold)  return -1;
	return 0;
#endif
}



//2. -----------compute linear field--------------
//https://en.wikipedia.org/wiki/Barycentric_coordinate_system
template<typename T>
void cal_linear_field(const T v1x, const T v1y, const T v2x, const T v2y, const T v3x, const T v3y, 
	const T x1x, const T x1y, const T x2x, const T x2y, const T x3x, const T x3y,
	T abc[3], T def[3])
{
	//v1 = A*x1,v2 = A*x2.v2 = A*x3 => (v1,v2,v3) = A*(x1,x2,x3)   were v1 = (v1x,v1y)^T   x1 = (x1x,x1y,1)^T
	//  v1x = (a b c )   | x1x |
	//                *  | x1y |
	//  v1y = (d e f)    |  1  |
	//  A =  (v1,v2,v3)*(x1,x2,x3)^-1 
	Eigen::MatrixXd X(3,3),V(2,3);  
	X << x1x, x2x, x3x,
		x1y, x2y, x3y,
		1, 1, 1;

	V << v1x, v2x, v3x,
		v1y, v2y, v3y;

	Eigen::MatrixXd A = V*X.inverse();
	abc[0] = A(0, 0);  abc[1] = A(0, 1);  abc[2] = A(0, 2);
	def[0] = A(1, 0);  def[1] = A(1, 1);  def[2] = A(1, 2);	
}

template<typename T>
int get_sing_type(const T  a, const T  b, const T  d,  const T  e)
{
	T A, B, C, delta;
	T r1, r2, i1, i2;

	A = 1;
	B = -(a + e);
	C = (a * e - b * d);

	delta = B*B - 4 * A*C;

	if (delta >= 0)
	{
		i1 = i2 = 0;
		r1 = (-B + sqrt(delta)) / 2;
		r2 = (-B - sqrt(delta)) / 2;
	}
	else
	{
		r1 = r2 = -B / 2.;
		i1 = sqrt(-delta) / 2;
		i2 = -sqrt(-delta) / 2;
	}


	////////////////////////////////////////////////////////////////
	if (r1 > 0 && r2 > 0 && i1 == 0 && i2 == 0)
		return 1; //it is a source

	else if (r1 < 0 && r2 < 0 && i1 == 0 && i2 == 0)
		return 2; //it is a sink

	else if (r1 * r2 < 0 && i1 == 0 && i2 == 0)
		return 3; //it is a saddle

	else if (r1 == 0 && r2 == 0 && i1 != 0 && i2 != 0)
		//else if( fabs(r1) <= 1e-4 && fabs(r2) <= 1e-4 && i1 != 0 && i2 != 0)
		return 4; //it is a center

	else if (r1 < 0 && r2 < 0 && i1 != 0 && i2 != 0)
		return 6; //it is an attracting focus

	else if (r1 > 0 && r2 > 0 && i1 != 0 && i2 != 0)
		return 7; //it is a repelling focus

	else
		return 0; //Unknow, there must be some error here!
}

//3. --------locate sigular point in the triangle --------------------
//3.1--------compute sigular position of the linear field by intersection of two lines

//http://baike.baidu.com/view/417411.htm
template<typename T>
void findIntersection(const T a, const T b, const T c, 
	                  const T d, const T e, const T f, T& x, T&y ) {
	//  ax+by+c = 0;        
	//  dx+ey+f = 0;
	T det = a*e - d*b;
	x = (b*f - e*c)/det;  y =  (c*d-a*f)/det;
}

// 3.2 ---------is the singularity lying in the triangle?
//Judge whether a given point is inside the specified triangle
//is p1 and p2 lie in the same side of line ab ?
template<typename T>
bool same_side(const T p1[2], const T p2[2], const T a[2], const T b[2])
{
	T vec1[2] = { b[0] - a[0], b[1] - a[1] };
	T vec2[2] = { p1[0] - a[0], p1[1] - a[1] };
	T vec3[2] = { p2[0] - a[0], p2[1] - a[1] };

	//normalize(vec1);
	//normalize(vec2);
	//normalize(vec3);

	double cp1 = vec1[0] * vec2[1] - vec1[1] * vec2[0];
	double cp2 = vec1[0] * vec3[1] - vec1[1] * vec3[0];

	double cp = cp1*cp2;

	//if ((cp1*cp2/*+9.e-9*/) >= 0) return true;
	if ((cp1*cp2+9.e-9) >= 0) return true;
	return false;
}

template<typename T>
bool is_inside_tri(const T p[2], const T a[2], const T b[2] , const T c[2])
{	
	if (same_side(p, a, b, c) && same_side(p, b, a, c) && same_side(p, c, a, b))
		return true;
	return false;
}


template<typename T>
class VF_triangle{
	T v1x, v1y, v2x, v2y, v3x, v3y, x1, y1, x2, y2, x3, y3;
public:
	T a, b, c, d, e, f; //linear vector field in the triangle
	T x, y;  int type; //singularity        

	VF_triangle(const T v1x_, const T v1y_, const T v2x_, const T v2y_, const T v3x_, const T v3y_,
		const T x1_, const T y1_, const T x2_, const T y2_, const T x3_, const T y3_)
		:v1x(v1x_), v1y(v1y_), v2x(v2x_), v2y(v2y_), v3x(v3x_), v3y(v3y_),
		x1(x1_), y1(y1_), x2(x2_), y2(y2_), x3(x3_), y3(y3_)
	{
		type = -1;
		locateSingularityPoint();
	}
	bool locateSingularityPoint(){
		if (isSigularity(v1x, v1y, v2x, v2y, v3x, v3y)==0) return false;
		T abc[3], def[3];
		cal_linear_field(v1x, v1y, v2x, v2y, v3x, v3y, x1, y1, x2, y2, x3, y3,abc,def);
		a = abc[0]; b = abc[1]; c = abc[2];   d = def[0]; e = def[1]; f = def[2];
		findIntersection(abc[0], abc[1], abc[2],def[0],def[1],def[2],x,y);
		T p[2] = { x, y }, A[2] = {x1,y1}, B[2] = {x2,y2,}, C[2] = {x3,y3};
		if (!is_inside_tri(p, A, B, C))return false;
		type = get_sing_type(a, b, d, e);
		return true;
	}
};

template<typename T>
//typedef float T;
void findEigenVector(const T a, const T b, const T d, const T e, std::vector<T> &v1, std::vector<T> &v2){
	//https://eigen.tuxfamily.org/dox/classEigen_1_1EigenSolver.html
	//	typedef Matrix< double, Dynamic, Dynamic > MatrixXd  https://eigen.tuxfamily.org/dox/group__matrixtypedefs.html#ga0750af9a6b82761985a15fe77256de87
	Eigen::Matrix< T, Eigen::Dynamic, Eigen::Dynamic > m(2, 2);

	m(0, 0) = a;	m(1, 0) = b;
	m(0, 1) = d;	m(1, 1) = e;
	Eigen::EigenSolver<Eigen::Matrix< T, Eigen::Dynamic, Eigen::Dynamic >> es(m, true);
	Eigen::Matrix< std::complex< T >, Eigen::Dynamic, Eigen::Dynamic > eigenVectors = es.eigenvectors();
	Eigen::Matrix< std::complex< T >, Eigen::Dynamic, 1 >  V1 = es.eigenvectors().col(0);
	Eigen::Matrix< std::complex< T >, Eigen::Dynamic, 1 >  V2 = es.eigenvectors().col(1);
	//	std::cout << "eifenvector:\n " << es.eigenvectors() << std::endl;
	std::cout << V1 << std::endl;
	std::cout << V2 << std::endl;
	v1[0] = V1(0, 0).real();   v1[1] = V1(1, 0).real();
	v2[0] = V2(0, 0).real();   v2[1] = V2(1, 0).real();
	std::cout << v1[0] << "," << v1[1] << std::endl;
	std::cout << v2[0] << "," << v2[1] << std::endl;
}

//============================
template<typename T>
struct Singularity {
	int face_id;
	vector<T> pos;
	vector<T>  alpha;
	int type;
	vector<T> eigenVectorL, eigenVectorS;
};


#if 0
#include <vector>
template<typename T>
//typedef float T;
void VF_Analysis(const T *vx, const  T *vy, const  int width, const int height, std::vector<Singularity<T>> &singularities)
{
	int H = height - 1, W = width - 1;
	int idx_A, idx_B, idx_C, idx_D; 
	//T A[2].B[2], C[2], D[2];
	for (int y = 0; y < H; y ++) {
		for (int x = 0; x < W; x++) {
			idx_A = y*width + x;        idx_D = y*width + x + 1;
			idx_B = (y+1)*width + x;    idx_C = (y + 1)*width + x + 1;
			
			//int s1 = isSigularity(vx[idx_A], vy[idy_A], vx[idx_B], vy[idy_B], vx[idx_C], vy[idy_C]);
			//int s2 = isSigularity(vx[idx_C], vy[idy_C], vx[idx_D], vy[idy_D], vx[idx_A], vy[idy_A]);
			VF_triangle<T> Tri(vx[idx_A], vy[idx_A], vx[idx_B], vy[idx_B], vx[idx_C], vy[idx_C],
				(T)x, (T)y,   (T)x, (T)(y + 1),    (T)(x + 1), (T)(y + 1) );
		
			if (Tri.type>0){				
				Singularity<T> singular;
				singular.face_id = f; singular.p = p; singular.alpha = alpha; singular.type = type;

				
				if (Tri.type == 3){
					T eV1[2], eV2[2];
					findEigenVector(Tri.a, Tri.b, Tri.d, Tri.e, eV1, eV2);
					singular.eigenVectorL[0] = eV1[0]; singular.eigenVectorL[1] = eV1[1];
					singular.eigenVectorS[0] = eV2[0]; singular.eigenVectorS[1] = eV2[1];
				}
				singularities.push_back(singular);
			}
			Tri = VF_triangle<T>(vx[idx_C], vy[idx_C], vx[idx_D], vy[idx_D], vx[idx_A], vy[idx_A],
				(T)(x + 1), (T)(y + 1), (T)(x + 1), (T)y, (T)x, (T)y);
			if (Tri.type>0){
				Singularity<T> singular(Tri.x, Tri.y, Tri.type);
				std::cout << Tri.x << ", " << Tri.y << ", " << Tri.type << std::endl;
				if (Tri.type == 3){
					T eV1[2], eV2[2];
					findEigenVector(Tri.a, Tri.b, Tri.d, Tri.e, eV1, eV2);
					singular.eigenVectorL[0] = eV1[0]; singular.eigenVectorL[1] = eV1[1];
					singular.eigenVectorS[0] = eV2[0]; singular.eigenVectorS[1] = eV2[1];
				}
				singularities.push_back(singular);
			}
		}
	}
}
#endif


//==========================================
extern void draw_line_rgb_DDA(unsigned char * outimg, const int x1, const int y1, const int x2, const int y2,
	const unsigned char color[], const int width);

template<typename T>
void draw_singularity_rgb(unsigned char * outimg, const Singularity<T> &singular, const int width){
	unsigned char color[3] = { 0, 0,255 };
	float radious = 2;
	int L = singular.pos[0] - radious, R = singular.pos[0] + radious,
		B = singular.pos[1] - radious, T = singular.pos[1] + radious;
	int idx;
	for (int y = B; y <= T; y++){
		for (int x = L; x <= R; x++){
			idx = y*width + x;
			unsigned char *p = outimg + 3 * idx;
			p[0] = color[0];  p[1] = color[1];  p[2] = color[2];
		}
	}
	if (singular.type == 3){
		
		int x1 , y1, x2, y2;
		float scale = 20., sV[2] = { scale*singular.eigenVectorL[0], scale*singular.eigenVectorL[1] };

		x1 = singular.pos[0] - sV[0]; y1 = singular.pos[1] - sV[1];
		x2 = singular.pos[0] + sV[0]; y2 = singular.pos[1] + sV[1];
		draw_line_rgb_DDA(outimg, x1, y1, x2, y2, color, width);

		scale = scale/2;
		sV[0] = scale*singular.eigenVectorS[0]; sV[1] = scale*singular.eigenVectorS[1];	

		x1 = singular.pos[0] - sV[0]; y1 = singular.pos[1] - sV[1];
		x2 = singular.pos[0] + sV[0]; y2 = singular.pos[1] + sV[1];
		draw_line_rgb_DDA(outimg, x1, y1, x2, y2, color, width);
	}
}

inline void test_draw_singularities_rgb(unsigned char * outimg,
	const std::vector<Singularity<float>> singularites,
	const  int width)
{
	unsigned char color[3] = { 0,0,255 };

	for (int i = 0; i < singularites.size(); i++) {
		draw_singularity_rgb(outimg, singularites[i], width);
	}
}

inline void test_VF_Design() {
#if 0
	Eigen::MatrixXd m = Eigen::MatrixXd::Ones(3, 3);
#else
	Eigen::MatrixXd m(2, 2);
	m(0, 0) = 1;	m(1, 0) = 0;
	m(0, 1) = 0;	m(1, 1) = -1;
#endif
	Eigen::EigenSolver<Eigen::MatrixXd> es(m, false);
	cout << "The eigenvalues of the 3x3 matrix of ones are:"
		<< endl << es.eigenvalues() << endl;

	cout << angle2vector(1., 0., 1., 1.) << endl;
}

