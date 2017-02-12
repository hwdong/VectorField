#pragma once

template<typename T>
inline bool  isIntersection(const T p[2], const T A[2], const T B[2]) {
	if (p[1] >= B[1] || p[1] < A[1]) return false;
	T x = (p[1] - A[1]) / (B[1] - A[1])*(B[0] - A[0]) + A[0];
	return x >= p[0];
}

template<typename T>
bool isIn(T p[2], const std::vector<T> xs, const std::vector<T> &ys) {
	int num_intersection = 0;
	int n = xs.size();
	T A[2] = { xs[n - 1], ys[n - 1] };
	for (int i = 0; i < n; i++) {
		T B[2] = { xs[i], ys[i] };
		//if the scanline of p intersect with AB
		bool ret;
		if (B[1] >= A[1])
			ret = isIntersection(p, A, B);
		else
			ret = isIntersection(p, B, A);
		if (ret) num_intersection++;
		A[0] = B[0]; A[1] = B[1];
	}
	return num_intersection % 2 == 1;
}

template<typename T>
void isIn(Eigen::Matrix<T, -1, -1> &V, vector<int> &in_vids,
	const std::vector<T> xs, const std::vector<T> &ys)
{
	T p[2];
	for (int i = 0; i < V.rows(); i++) {
		p[0] = V(i, 0); p[1] = V(i, 1);
		if (isIn(p, xs, ys))
			in_vids.push_back(i);
	}
}


template<typename T>
void isIn(Eigen::Matrix<T, -1, -1> &V, vector<int> &in_vids,
	const std::vector<T> &xs, const std::vector<T> &ys,
	const int width, const int height)
{
	int n = xs.size();
	std::vector<T> x(n), y(n);
	for (int i = 0; i < n; i++) {
		x[i] = xs[i] * width; y[i] = ys[i] * height;
	}
	isIn(V, in_vids, x, y);
}

template<typename T>
T length(const T x1, const T y1, const T x2, const T y2) {
	T x = x2 - x1, y = y2 - y1;
	T d = x*x + y*y;
	return sqrt(d);
}
template<typename T>
T length(const T p[2],const T q[2]) {	
	return length(p[0],p[1],q[0],q[1]);
}
template<typename T>
T cal_length(const vector<T> &x, const vector<T> &y) {
	T len = 0;
	assert(x.size() == y.size());
	T p[2] = { x[0],y[0] },q[2];
	for (int i = 0; i<x.size(); i++){
		int j = (i + 1) % x.size();
		q[0] = x[j]; q[1] = y[j];		
		len += length(p, q);
		p[0] = q[0]; p[1] = q[1];
	}
	return len;
}
template<typename T>
void resample_curve(vector<T> &sampled_x, vector<T> &sampled_y, const vector<T> &x, const vector<T> &y,const T step_length) 
{
	T curLen = 0;
	int nPts = x.size();
	sampled_x.push_back(x[0]); sampled_y.push_back(y[0]);
	T p[2] = { x[0],y[0] }, q[2];
	for (int i = 0; i<nPts ; i++){
		int j = (i + 1) % x.size();
		q[0] = x[j]; q[1] = y[j];
		T lineLen = length(p, q);
		curLen += lineLen;

		while (curLen >= step_length)	{
			//   obtain the position of the sampled point  
			T extraLen = curLen - step_length;
			T alpha = extraLen / lineLen;
			T t_x = alpha*p[0] + (1 - alpha)*q[0];
			T t_y = alpha*p[1] + (1 - alpha)*q[1];

			sampled_x.push_back(t_x); sampled_y.push_back(t_y);			

			//parameter t
	//		T t = 1. / (num - 1)*(sampled_x.size() - 1);

			//   update the current accumulating length   
			curLen -= step_length;
		}
		p[0] = q[0]; p[1] = q[1];
	}
		
	if (nPts > 0){    //  add the last point of the design curve 
		sampled_x.push_back(x.back()); sampled_y.push_back(y.back());
		//parameter t=1.	
	}
}

template<typename T>
void resample_curve(vector<T> &sampled_x, vector<T> &sampled_y, const vector<T> &x, const vector<T> &y, const int num)
{
	T curve_length = cal_length(x, y);
	T step_length = curve_length / (num - 1);
	resample_curve(sampled_x, sampled_y,x,y, step_length);
}

template<typename T>
void resample_curve( vector<T> &x,  vector<T> &y, const int num)
{
	vector<T> sampled_x, sampled_y;
	resample_curve(sampled_x, sampled_y, x, y, num);
	x = sampled_x; y = sampled_y;
}




//=================tracing triangles along the curve================
#include "VectorField.h"
//http://www.informit.com/articles/article.aspx?p=1405557
//Chen:TriangleDetect
template<typename T>
int TriangleDetect(const VectorField<T> &vField, T x, T y)
{
#ifdef USE_QT
	// SETUP
	QSurfaceFormat format;
	// following should be commeted, it may due to old opengl driver?
	//	format.setMajorVersion(3);
	//	format.setMinorVersion(3);

	QWindow window;
	window.setSurfaceType(QWindow::OpenGLSurface);
	window.setFormat(format);
	window.create();

	QOpenGLContext context;
	context.setFormat(format);
	if (!context.create())
		qFatal("Cannot create the requested OpenGL context!");
	context.makeCurrent(&window);
#else

#endif

	const int MaxSize = 512;
	GLuint selectBuffer[MaxSize];

	glSelectBuffer(MaxSize, selectBuffer);
	glRenderMode(GL_SELECT);
	glInitNames();
	glPushName(0);

	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT, viewport);

	GLint	vp[4] = { 0, 0 ,1, 1 };
	glViewport(vp[0], vp[1], vp[2], vp[3]);

//	glMatrixMode(GL_MODELVIEW);
//	glLoadIdentity();

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();

	gluPickMatrix(x, y, 1e-8, 1e-8, vp);
	glOrtho(0, 1, 0, 1, 0, 50);

	float pos_v[4];
	int nfaces = vField.F.rows();
//	glBegin(GL_TRIANGLES);
	for (int f = 0; f<nfaces; f++) {
		//Eigen::Matrix<T, 1, -1> face = vField.F.row(f);
		glLoadName(f);
		int nvertices = vField.F.cols();
			glBegin(GL_POLYGON);
		for (int j = 0; j<nvertices; j++)
		{
			int vert = vField.F(f, j);
			
			glVertex2f(vField.V(vert, 0), vField.V(vert, 1));
		}
		glEnd();
	}
//	glEnd();

	int hits = glRenderMode(GL_RENDER);
	int selectedTriangle = -1;
	if (hits>0) {
		////Because this is 2D plane, we need not worry about the overlap of objects
		////It will have at most one object being selected at one click (it may not be true)
		int objname = selectBuffer[3];
		selectedTriangle = objname;// objname - NAMEOFTRIANGLE;		
	}
	
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
	return selectedTriangle;

}


//return: parameter t of the vertex on line pq, if t<=0, pq not pass a vertex.
template<typename T>
T  is_cross_vertex(const VectorField<T> &VF,const int cur_triangle, vector<T> &p, vector<T> &q,int &which_vertex)
{  
	T A, B, C, d,t;
	A = q[1] - p[1];
	B = q[0] - p[0];
	C = q[0] * p[1] - p[0] * q[1];
	for (int i = 0; i < 3; i++)
	{
		int vid = VF.F(cur_triangle, i);
		Eigen::Matrix<T, 1, -1> vert = VF.V.row(vid);
	//	vert = vP;
		d = A*vert(0) + B*vert(1) + C;
		if (fabs(d) < 1e-8) { //passing the vertex
			t = (vert(0) - p[0]) / B;
			if (t > 1e-8) {
				which_vertex = i;
				return t;
			}
				
		}
	}
	which_vertex = -1;
	return -1.;
}


//suppose p is in the triangle and q is outside the triangle
// and p may lie on a vertex or an edge of the triangle
//solution: p,q lie on the opposite side of an edge?
template<typename T>
int  cross_which_edge(const VectorField<T> &VF, int &face_id, const Eigen::Matrix<T, 1, -1> &p,
	Eigen::Matrix<T, 1, -1> &q,T t[2])
{
	T pp[2] = { p(0),p(1) }, qq[2] = { q(0),q(1) };

	int which_edge=-1;
	const int vn = VF.F.cols();
	if (VF.V.cols() == 2) { //2d
		Eigen::Matrix<T, 1, 2> pq; pq << q[0] - p[0], q[1] - p[1];
		pq.normalize();
		Eigen::Matrix<T, 1, -1> p1 = VF.V.row(VF.F(face_id, 0));
		which_edge = -1;
		T max_dist = 0;
		for (int i = 0; i < vn; i++) {
			int j = (i + 1) % vn;
			Eigen::Matrix<T, 1, -1> p2 = VF.V.row(VF.F(face_id, j));
			Eigen::Matrix<T, 1, 2> normal;
			normal << p2(1) - p1(1), p1(0) - p2(0); // the right normal of vector(x,y) is (y,-x)
			Eigen::Matrix<T, 1, 2> p1q;			
			p1q << q(0) - p1(0), q(1) - p1(1);
			normal.normalize();		
			T d = p1q.dot(normal); //the distance of q to the positive side of p1p2
			if (d > 0) { //check if q   sss 
#if 0
				Eigen::Matrix<T, 1, 2> p1p2; p1p2 << p2[0] - p1[0], p2[1] - p1[1];
				T c1 = p1q(0)*p1p2(1) - p1p2(0)*p1q(1);
				if (c1 >= 0) {
					Eigen::Matrix<T, 1, 2> p2q;
					p2q << q(0) - p2(0), q(1) - p2(1);
					T c2 = p2q(0)*p1p2(1) - p1p2(0)*p2q(1);
					if (c2 < 0) {
						if (d > max_dist) {
							max_dist = d; which_edge = i; //the edge from the i-th vertex
						}
					}
				}
#else
				if (d > max_dist) {
				//	max_dist = d; which_edge = i; //the edge from the i-th vertexz
					T tt[2];
					if (Intersection2LineSegments(p, q, p1, p2, tt) == 1) {
						max_dist = d; which_edge = i;
						t[0] = tt[0]; t[1] = tt[1];
					}
				}
				
#endif
			}			
			p1(0) = p2(0); p1(1) = p2(1);//p1 = p2;
		}
		if (which_edge >= 0) return which_edge;
		/*
		if (which_edge >= 0) {
			Eigen::Matrix<T, 1, -1> p1 = VF.V.row(VF.F(face_id, which_edge));
			Eigen::Matrix<T, 1, -1> p2 = VF.V.row(VF.F(face_id, (which_edge +1)%vn));
			if (Intersection2LineSegments(p, q, p1, p2, t) == 1) {
				return which_edge;
			}
		}
		*/
	}
	return -1;
}

template<typename T>
int  cross_which_edge(const VectorField<T> &VF, int &face_id, const vector<T> &p, const vector<T> &q,
	 T t[2])
{
	Eigen::Matrix<T, 1, -1> P(1,p.size()), Q(1, p.size());

	for (int i = 0; i < p.size(); i++) {
		P(i) = p[i];  Q(i) = q[i];
	}
	return cross_which_edge(VF, face_id, P, Q,t);

}
#include "Tracing.h"

//-----p is in and q is outside of the cur_triangle,we want to get to the next triangle----
//  because p is in the cur_triangle, then pq has only two case:
//  1) q pass through a vertex of the cur_triangle,that is pq colline with an edge of cur_triangle
//     X   1.1.1) p...q...v,  q in the the edge e,then we go to the next point (set p=q).   
//                 this case can't happen because q outside and p is in the  cur_triangle
//         1.1.2) p...v...q,  q outside of an edge e,find a neighbor triangle which pq pass through.
// 2) pq point to an edge of the  cur_triangle,get the opposite_triangle
//    2.1  q is in the cur_triangle
//    2.2  set p = intersection of the edge and pq and  next_triangle will be the opposite triangle
// return: <=-1 failed; ==0 p updated but not reach to q;  ==1 reached q
template<typename T>
int goto_next_triangle(const VectorField<T> &VF,int &cur_triangle, vector<T> &p, vector<T> &q) {
	vector<T> alpha(3);
	int vert_id;
	T s = is_cross_vertex(VF, cur_triangle, p, q, vert_id);
	if (s > 1)  return -2- vert_id;//something wrong : the vertex is on the right of the line segment pq. 
	else if (s > 0) {// the vertex is in the line segment pq.
		Eigen::Matrix<T, 1, Dynamic> pV; 
		pV << q[0] - p[0], q[1] - p[1];
		pV.normalize();		
		int next_triangle = cur_triangle;
		Vertex2NextTriangle(VF, vert_id, next_triangle, pV);
		if (next_triangle == cur_triangle) {
			return -1;
		}
		cur_triangle = next_triangle;

		VF.GetBarycentricFacters(next_triangle, q, alpha);
		if ((alpha[0] >= 0 || fabs(alpha[0]) <= 1e-8) && alpha[0] <= 1
			&& (alpha[1] >= 0 || fabs(alpha[1]) <= 1e-8) && alpha[1] <= 1
			&& (alpha[2] >= 0 || fabs(alpha[2]) <= 1e-8) && alpha[2] <= 1)
		{
			p[0] = q[0]; p[1] = q[1];
			return 1; // has reached q
		}
		else {
			p[0] = VF.V(vert_id, 0); p[1] = VF.V(vert_id, 1); //set p=vert so that trave from the new triangle
			return 0;  //has not reached q,just update p
		}
	}
	// pq will point to an edge of the  cur_triangle
	T t[2];
	int which_edge = cross_which_edge(VF, cur_triangle, p, q, t );
	if (which_edge < 0)
		return -1;
	int next_triangle = VF.TT(cur_triangle, which_edge);

	cur_triangle = next_triangle;
	// if q in the next_triangle
	VF.GetBarycentricFacters(next_triangle, q, alpha);
	if ((alpha[0] >= 0 || fabs(alpha[0]) <= 1e-8) && alpha[0] <= 1
		&& (alpha[1] >= 0 || fabs(alpha[1]) <= 1e-8) && alpha[1] <= 1
		&& (alpha[2] >= 0 || fabs(alpha[2]) <= 1e-8) && alpha[2] <= 1)
	{
		p[0] = q[0]; p[1] = q[1];
		return 1; // has reached q
	}
	else {// p = intersection of the edge and pq
		p[0] = p[0] + t[0] * (q[0] - p[0]);
		p[1] = p[1] + t[0] * (q[1] - p[1]);
		return 0;  // has not reached q,just update p
	}
}



//-------------comp_designTriStrip-------------
#include <fstream>
template<typename T>
bool log_comp_designTriStrip(const vector<T> &x, const vector<T> &y, vector<int>&triangles,int ret,
	const char *file = "comp_designTriStrip.txt")
{
	std::ofstream oF(file);
	if (!oF) return false;

	oF << x.size() << " " << triangles.size() << " " << ret << "\n";
	for (int i = 0; i < x.size(); i++)
		oF << x[i] << " " << y[i] << "\n";
	for (int i = 0; i < triangles.size(); i++)
		oF << triangles[i] << "\n";

	oF.close();
	return true;
}
template<typename T>
bool log_comp_designTriStrip_read( vector<T> &x,  vector<T> &y,
	const char *file = "comp_designTriStrip.txt")
{
	int n,t,ret;
	std::ifstream iF(file);
	if (!iF) return false;

	iF >> n >> t >> ret;
	x.resize(n); y.resize(n);
	for (int i = 0; i < x.size(); i++)
		iF >> x[i] >> y[i] ;
	iF.close();
	return true;
}

template<typename T>
void extract_sketch_triangle_strip(const VectorField<T> &VF,const vector<T> &x, const vector<T> &y,
	vector<int> &triangles, vector<int> &nearest_point_indices)
{
	int cur_triangle = TriangleDetect(VF, x[0], y[0]), pre_triangle;
	assert(cur_triangle >= 0);
	triangles.push_back(cur_triangle); nearest_point_indices.push_back(0);

	int ret;
	vector<T> p = { x[0],y[0] }, q(2);

	vector<T> alpha(3);
	for (int i = 0; i < x.size(); i++) {
		int j = (i + 1) % x.size();
		q[0] = x[j]; q[1] = y[j];
		
		VF.GetBarycentricFacters(cur_triangle, q, alpha);
		if ((alpha[0] >= 0 || fabs(alpha[0]) <= 1e-8) && alpha[0] <= 1
			&& (alpha[1] >= 0 || fabs(alpha[1]) <= 1e-8) && alpha[1] <= 1
			&& (alpha[2] >= 0 || fabs(alpha[2]) <= 1e-8) && alpha[2] <= 1)
		{
			p[0] = q[0];
			p[1] = q[1]; continue;
		}
		pre_triangle = cur_triangle;
		ret = goto_next_triangle(VF, cur_triangle, p, q);

		if (ret <0) {
			break; //failed
		}
		triangles.push_back(cur_triangle); 
		if (ret == 0) {
			nearest_point_indices.push_back(i);
			i--;
		}
		else if (ret == 1) {
			p[0] = q[0];	p[1] = q[1]; 
			nearest_point_indices.push_back(j);
			continue;
		}
	}		
//	log_comp_designTriStrip(x,y, triangles,ret);  //log the designTriStriptp file
}

template<typename T>
void extract_sketch_triangle_strip(const VectorField<T> &VF, const Eigen::Matrix<T, Dynamic, Dynamic> &sketch,
	vector<int> &triangles, vector<int> &nearest_point_indices)
{
	int cur_triangle = TriangleDetect(VF, sketch(0,0), sketch(0, 1)), pre_triangle;
	triangles.push_back(cur_triangle); nearest_point_indices.push_back(0);

	int ret;
	vector<T> p = { sketch(0,0), sketch(0, 1) }, q(2);

	vector<T> alpha(3);
	for (int i = 0; i < sketch.rows(); i++) {
		int j = (i + 1) % sketch.rows();

	//	q[0] = x[j]; q[1] = y[j];
		q[0] = sketch(j, 0);  q[1] = sketch(j, 1);

		VF.GetBarycentricFacters(cur_triangle, q, alpha);
		if ((alpha[0] >= 0 || fabs(alpha[0]) <= 1e-8) && alpha[0] <= 1
			&& (alpha[1] >= 0 || fabs(alpha[1]) <= 1e-8) && alpha[1] <= 1
			&& (alpha[2] >= 0 || fabs(alpha[2]) <= 1e-8) && alpha[2] <= 1)
		{
			p[0] = q[0];
			p[1] = q[1]; continue;
		}
		pre_triangle = cur_triangle;
		ret = goto_next_triangle(VF, cur_triangle, p, q);

		if (ret <0) {
			break; //failed
		}
		triangles.push_back(cur_triangle);
		if (ret == 0) {
			nearest_point_indices.push_back(i);
			i--;
		}
		else if (ret == 1) {
			p[0] = q[0];	p[1] = q[1];
			nearest_point_indices.push_back(j);
			continue;
		}
	}
}

