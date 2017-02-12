#include "VectorField.h"
#include "MathUtil.h"

//===============Routines to solve Runge-Kutta Integration====================
template<typename T>
void rkck(const VectorField<T> &VF, const int face_id, const vector<T> &y, const vector<T> &dydx, 
	const T x,	const T h, vector<T> &yout, vector<T> &yerr,
	//void derivs(const T, const vector<T> &, vector<T> &))
	void derivs(const VectorField<T> & VF, const int face_id, const vector<T> &, vector<T> &))
{
	static const T a2 = 0.2, a3 = 0.3, a4 = 0.6, a5 = 1.0, a6 = 0.875,
		b21 = 0.2, b31 = 3.0 / 40.0, b32 = 9.0 / 40.0, b41 = 0.3, b42 = -0.9,
		b43 = 1.2, b51 = -11.0 / 54.0, b52 = 2.5, b53 = -70.0 / 27.0,
		b54 = 35.0 / 27.0, b61 = 1631.0 / 55296.0, b62 = 175.0 / 512.0,
		b63 = 575.0 / 13824.0, b64 = 44275.0 / 110592.0, b65 = 253.0 / 4096.0,
		c1 = 37.0 / 378.0, c3 = 250.0 / 621.0, c4 = 125.0 / 594.0, c6 = 512.0 / 1771.0,
		dc1 = c1 - 2825.0 / 27648.0, dc3 = c3 - 18575.0 / 48384.0,
		dc4 = c4 - 13525.0 / 55296.0, dc5 = -277.00 / 14336.0, dc6 = c6 - 0.25;
	int i;

	int n = y.size();
	vector<T> ak2(n), ak3(n), ak4(n), ak5(n), ak6(n), ytemp(n);
	for (i = 0; i<n; i++)
		ytemp[i] = y[i] + b21*h*dydx[i];
//	derivs(x + a2*h, ytemp, ak2);
	derivs(VF,face_id, ytemp, ak2);
	for (i = 0; i<n; i++)
		ytemp[i] = y[i] + h*(b31*dydx[i] + b32*ak2[i]);
	//derivs(x + a3*h, ytemp, ak3);
	derivs(VF, face_id, ytemp, ak3);
	for (i = 0; i<n; i++)
		ytemp[i] = y[i] + h*(b41*dydx[i] + b42*ak2[i] + b43*ak3[i]);
	//derivs(x + a4*h, ytemp, ak4);
	derivs(VF, face_id, ytemp, ak4);
	for (i = 0; i<n; i++)
		ytemp[i] = y[i] + h*(b51*dydx[i] + b52*ak2[i] + b53*ak3[i] + b54*ak4[i]);
	//derivs(x + a5*h, ytemp, ak5);
	derivs(VF, face_id, ytemp, ak5);
	for (i = 0; i<n; i++)
		ytemp[i] = y[i] + h*(b61*dydx[i] + b62*ak2[i] + b63*ak3[i] + b64*ak4[i] + b65*ak5[i]);
	//derivs(x + a6*h, ytemp, ak6);
	derivs(VF, face_id, ytemp, ak6);
	for (i = 0; i<n; i++)
		yout[i] = y[i] + h*(c1*dydx[i] + c3*ak3[i] + c4*ak4[i] + c6*ak6[i]);
	for (i = 0; i<n; i++)
		yerr[i] = h*(dc1*dydx[i] + dc3*ak3[i] + dc4*ak4[i] + dc5*ak5[i] + dc6*ak6[i]);
}


template<typename T>
void rkqs(const VectorField<T> & VF, const int face_id, vector<T> &y, vector<T> &dydx, T &x,
	const T htry, const T eps, const vector<T> &yscal, T &hdid, T &hnext,
	//void derivs(const T, const vector<T> &, vector<T> &))	
	void derivs(const VectorField<T> & VF, const int  face_id, const vector<T> &, vector<T> &))
{
	const T SAFETY = 0.9, PGROW = -0.2, PSHRNK = -0.25, ERRCON = 1.89e-4;
	int i;
	T errmax, h, htemp, xnew;

	int n = y.size();
	h = htry;
	vector<T> yerr(n), ytemp(n);
	for (;;) {
		rkck(VF,face_id,y, dydx, x, h, ytemp, yerr, derivs);
		errmax = 0.0;
		for (i = 0; i < n; i++) errmax = std::max(errmax, fabs(yerr[i] / yscal[i]));//MAX(errmax, fabs(yerr[i] / yscal[i]));
		errmax /= eps;
		if (errmax <= 1.0) break;
		htemp = SAFETY*h*pow(errmax, PSHRNK);
		h = (h >= 0.0 ? std::max(htemp, 0.1*h) : std::min(htemp, 0.1*h));
		xnew = x + h;
		if (xnew == x) {
			printf("error:stepsize underflow in rkqs\n");
			return;
		}
	}
	if (errmax > ERRCON) hnext = SAFETY*h*pow(errmax, PGROW);
	else hnext = 5.0*h;
	x += (hdid = h);
	for (i = 0; i<n; i++) y[i] = ytemp[i];
}

//---------------Runge Kutta integrator driver for local tracing---------------
template<typename T>
void localderive(const VectorField<T> & VF, const int  face_id, const vector<T> &y, vector<T> &dydx)
//const DP t, Vec_I_DP &c, Vec_O_DP &dydx)
{
	//double alpha[3];
	vector<T> alpha(3);
	VF.GetBarycentricFacters(face_id, y, alpha);
	//	Get2DBarycentricFacters(globalface, y[0], y[1], alpha);

	////first, we need to get the change of next points
	Eigen::Matrix<T, 1, Dynamic> pV = VF.GetVectorAtPoints(face_id, alpha);
	//icVector2 v = GetVectorAtPoints(globalface, alpha);

	/*normalize it before RK  07/25/07*/
	//normalize(v);
	//v = 100*DistanceThreshold*v;

	dydx.resize(pV.cols());
	for (int j = 0; j <dydx.size(); j++)
		dydx[j] = pV(j);
	//	dydx[0] = v.entry[0];
	//	dydx[1] = v.entry[1];

}

template<typename T>
void localinverse_derive(const VectorField<T> & VF, const int  face_id, 
	const vector<T> &y, vector<T> &dydx)
{
	localderive(VF, face_id, y, dydx);
	for (int j = 0; j < dydx.size(); j++)
		dydx[j] = -dydx[j];
}

//--------------Runge-Kutta tracing from a point to the next point-----------------------
//http://www.aip.de/groups/soe/local/numres/bookcpdf/c16-2.pdf
template<typename T>
bool ToNextPointRK4(const VectorField<T> & VF, const int face_id,
	const Eigen::Matrix<T, 1, -1> &p, Eigen::Matrix<T, 1, -1> &p_next,
	const vector<T> &alpha, T &htry, bool forward_backward=true)
{
	Eigen::Matrix<T, 1, Dynamic> pV = VF.GetVectorAtPoints(face_id, alpha);
//	std::cout << "pV.norm():=" << pV.norm() << "\n";
//	if (pV.norm() < 1e-20)
	if (pV.norm() < 1e-13) 
		return false; //for saddle-saddle connection example 
										 ////calling Runge Kutta to get the next point
	const int N = p.cols();  // N = 2
	int i, j;
	T eps, hdid, hnext, t = 1.0;
//	T htry = 0.02;// 1.;

	vector<T> by(N), dydx(N), dysav(N), ysav(N), yscal(N);

	//ysav[0] = first[0];	ysav[1] = first[1];
	for (int j = 0; j < N; j++)
		ysav[j] = p[j];
	
	if (forward_backward)
		localderive(VF,face_id, ysav, dysav);
	else
		localinverse_derive(VF, face_id, ysav, dysav);

	for (i = 0; i < N; i++) yscal[i] = 1;

	//set the htry to be the smallest edge length  07/30/07=
	//-------------------------------------------------------------------
	////calling adaptive stepsize runge-kutta method here to get next step
	for (i = 0; i < 15; i++) {
		eps = exp(-T(i + 1));
	//	t = 1.0;
		t = htry;
		for (j = 0; j < N; j++) {
			by[j] = ysav[j];
			dydx[j] = dysav[j];
		}
		if (forward_backward)
			rkqs(VF,face_id,by, dydx, t, htry, eps, yscal, hdid, hnext, localderive);
		else
			rkqs(VF,face_id,by, dydx, t, htry, eps, yscal, hdid, hnext, localinverse_derive);
	}

	////Set the new stepsize, note that this stepsize will affect the limit cycle detection
	htry = hnext;
	
//	if (hnext >= 2.)		htry = 2.;
	if (hnext >= 0.1)		htry = 0.1;

	/////store some important information
	//second[0] = by[0];	second[1] = by[1];
	for (int j = 0; j < N; j++)
		p_next[j] = by[j];
	/*
	//-------------------------------------------------------------------

	icVector2 line_v;
	line_v.entry[0] = second[0] - first[0];
	line_v.entry[1] = second[1] - first[1];
	normalize(line_v);
	double proj_len = dot(line_v, VecAtPoint); //here we use the dot product in the local frame
											   //for temporal tau
	g_cur_vec = VecAtPoint;

	g_vec_mag = fabs(proj_len);
	//g_dt += hdid/g_vec_mag;

	*/
	return true;
}

//Implement the fourth order Runge-Kutta integration without using adaptive step
//https://math.okstate.edu/people/yqwang/teaching/math4513_fall11/Notes/rungekutta.pdf

template<typename T>
bool ToNextPointRK4_(const VectorField<T> & VF, const int face_id,
	const Eigen::Matrix<T, 1, -1> &p, Eigen::Matrix<T, 1, -1> &p_next,
	const vector<T> &alpha_, T &htry, bool forward_backward = true)
{
	const T h = htry*0.5;

	vector<T> alpha = alpha_;
	//compute K1
	Eigen::Matrix<T, 1, Dynamic> pV1 = VF.GetVectorAtPoints(face_id, alpha);
	std::cout << "pV1.norm():=" << pV1.norm() << "\n";
	//	if (pV.norm() < 1e-20)
	if (pV1.norm() < 1e-13)
		return false; //for saddle-saddle connection example 
	if (!forward_backward) {
		pV1 = -pV1;
	}

//	Eigen::Matrix<T, 1, Dynamic> k1 = h* pV1.normalized();
	Eigen::Matrix<T, 1, Dynamic> k1 = h* pV1;


	//compute K2
	Eigen::Matrix<T, 1, -1> tP = p + k1 / 2; //p2
	VF.GetBarycentricFacters(face_id, tP, alpha);
	Eigen::Matrix<T, 1, Dynamic> pV2 = VF.GetVectorAtPoints(face_id, alpha);
	if (!forward_backward) 
		pV2 = -pV2;	

//	Eigen::Matrix<T, 1, Dynamic> k2 = h* pV2.normalized();
	Eigen::Matrix<T, 1, Dynamic> k2 = h* pV2;


	//compute K3
	tP = p + k2 / 2;  //p3
	VF.GetBarycentricFacters(face_id, tP, alpha);
	Eigen::Matrix<T, 1, Dynamic> pV3 = VF.GetVectorAtPoints(face_id, alpha);
	if (!forward_backward)
		pV3 = -pV3;
//	Eigen::Matrix<T, 1, Dynamic> k3 = h* pV3.normalized();
	Eigen::Matrix<T, 1, Dynamic> k3 = h* pV3;

	//compute K4
	tP = p + k3;  //p4
	VF.GetBarycentricFacters(face_id, tP, alpha);
	Eigen::Matrix<T, 1, Dynamic> pV4 = VF.GetVectorAtPoints(face_id, alpha);
	if (!forward_backward)
		pV4 = -pV4;
//	Eigen::Matrix<T, 1, Dynamic> k4 = h* pV4.normalized();
	Eigen::Matrix<T, 1, Dynamic> k4 = h* pV4;

	p_next= p + 1. / 6.*(k1 + 2 * k2 + 2 * k3 + k4);

	return true;
}

//-------------Eular to next point----------------
template<typename T>
bool ToNextPointEuler(const VectorField<T> & VF, const int face_id,
	const Eigen::Matrix<T, 1, -1> &p, Eigen::Matrix<T, 1, -1> &p_next,
	const vector<T> &alpha, T &htry, bool forward_backward = true)
{
	Eigen::Matrix<T, 1, Dynamic> pV = VF.GetVectorAtPoints(face_id, alpha);
	std::cout << "pV.norm():=" << pV.norm() << "\n";
	//	if (pV.norm() < 1e-20)
	if (pV.norm() < 1e-13)
		return false; //for saddle-saddle connection example 
	if (!forward_backward) {
		pV = -pV;
	}
	p_next = p + htry*pV;
	return true;
}

template<typename T>
bool ToNextPoint(const VectorField<T> & VF, const int face_id,
	const Eigen::Matrix<T, 1, -1> &p, Eigen::Matrix<T, 1, -1> &p_next,
	const vector<T> &alpha, T &htry, bool forward_backward = true)
{
//	return ToNextPointEuler(VF, face_id, p, p_next, alpha, htry, forward_backward);
//	return ToNextPointRK4(VF, face_id, p, p_next, alpha, htry, forward_backward);
	return ToNextPointRK4_(VF, face_id, p, p_next, alpha, htry, forward_backward);
}
//==============find the next triangle in tracing path==================
//------------------------------------
// start from a vertex (vert_id) on the triangle(face_id),
// we want to go to the the next triangle along the forwardbackward direction.
// The old face_id will be updated if successful
template<typename T>
//void TriangleThroughVertex(const VectorField<T> &VF, const int vert_id, int &face_id,
void VertextoNextTriangle(const VectorField<T> &VF,  const int vert_id, int &face_id,
	const bool forwardbackward)
{
	Eigen::Matrix<T, 1, Dynamic> pV = VF.VF.row(vert_id); //the direction of walking	
	if (!forwardbackward) pV = -pV;

	std::vector<int> triangles = VF.VT[vert_id];
	int vn = VF.F.cols();
	if(pV.cols() == 2){ //2D		
		for (int i = 0; i < triangles.size(); i++) {
			int f = triangles[i];
			int v = 0;
			for (; v < vn; v++) {
				if (VF.F(f, v) == vert_id)break;
			}
			int a = VF.F(f, v), b = VF.F(f, (v + 1) % vn), c = VF.F(f, (v + 2) % vn);
			Eigen::Matrix<T, 1, Dynamic> A = VF.V.row(a), B = VF.V.row(b), C = VF.V.row(c);
			//check if pV lie between AB and AC
			Eigen::Matrix<T, 1, Dynamic> AB = B - A, AC = C - A;
			AB.normalize(); AC.normalize();
			T pBV = AB*pV.adjoint(),pVC = pV*AC.adjoint();
			if (pBV >= 0 && pVC >= 0) {
				face_id = f;
				break;
			}
		}		
	}
	else {
		for (int i = 0; i < triangles.size(); i++) {
			int f = triangles[f];
			int v = 0;
			for (; v < vn; v++) {
				if (VF.F(f, v) == vert_id)break;
			}
			int a = VF.F(f, v), b = VF.F(f, (v + 1) % vn), c = VF.F(f, (v + 2) % vn);
			Eigen::Matrix<T, 1, Dynamic> A = VF.V.row(a), B = VF.V.row(b), C = VF.V.row(c);

			Eigen::Matrix<T, 1, 3> AB = B - A, AC = C - A;		
			AB.normalize(); AC.normalize();
			//project the pV to plane ABC
			Eigen::Matrix<T, 1, 3> face_normal = AB.cross(AC);
			face_normal.normalize();
			Eigen::Matrix<T, 1, 3> ppV = pV - (pV*face_normal.adjoint())*face_normal;
			//check if pV lie between AB and AC
			T pBV = AB.cross(ppV) *face_normal.adjoint(), pVC = ppV.cross(AC) *face_normal.adjoint();
			if (pBV >= 0 && pVC>= 0) {
				face_id = f;
				break;
			}
		}
	}
}

//New routine for crossing vertex testing 4/30/06
// return : which vertex is passing through 
template<typename T>
int  CrossVertex2(const VectorField<T> &VF, int &face_id, const Eigen::Matrix<T, 1, -1> &cur_p,
	const Eigen::Matrix<T, 1, -1> &pre_p, bool forwardbackward)
{	
	Eigen::Matrix<T, 1, -1> vert = pre_p;
	int newtriangleid = 0;
	int crossVert;

	// determin the equation of line through pre_p and  cur_p
	T A, B, C, pending;
	A = pre_p(1) - cur_p(1);
	B = cur_p(0) - pre_p(0);
	C = (pre_p(0) * cur_p(1) - cur_p(0) * pre_p(1));

	for (int i = 0; i < 3; i++)
	{
		int vid = VF.F(face_id, i);
		Eigen::Matrix<T, 1, -1> vP = VF.V.row(vid);
		vert = vP;
	
		pending = A*vert(0) + B*vert(1) + C;
		////We also need to make sure that the vertex is between 'pre' and 'cur' points
		if (fabs(pending) == 0.00) ////passing the vertex
		{
			////Test whether the vertex is between 'pre' and 'cur' points
			T t;
			if (pre_p(0) != cur_p(0))
			{
				t = (vert(0) - pre_p(0)) / (cur_p(0) - pre_p(0));

			}
			else {
				t = (vert(1) - pre_p(1)) / (cur_p(1) - pre_p(1));
			}

			if (t < 0 || t > 1)
			{				
				continue;
			}

			crossVert = vid; // face->verts[i];
			int old_face_id = face_id;
			VertextoNextTriangle(VF,crossVert, face_id, forwardbackward);		
			if (old_face_id != face_id) { //has found the next triangle

				return vid;// i + 1;
			}
		}
	}
	return 0;
}

template<typename T>
void  CrossBoundary3(const VectorField<T> &VF, int &face_id, const Eigen::Matrix<T, 1, -1> &p,
	Eigen::Matrix<T, 1, -1> &p_next,vector<T> alpha, int &which_edge, T t[2])
{
	const int vn = VF.F.cols();
	vector<int> v(3);
	v[0] = VF.F(face_id, 0), v[1] = VF.F(face_id, 1), v[2] = VF.F(face_id, 2);
	vector<Eigen::Matrix<T, 1, -1>> P(3);
	P[0] = VF.V.row(v[0]); P[1] = VF.V.row(v[1]); P[2] = VF.V.row(v[2]);


	vector<int> which_edges;
	if (alpha[0] < 0 && alpha[1] < 0) {//p_next lie out side of v1v2 and v2v0
		
		which_edges.push_back(1); which_edges.push_back(2);
	}
	else if (alpha[0] < 0 && alpha[2] < 0) {//p_next lie out side of v1v2 and v0v1
		which_edges.push_back(1); which_edges.push_back(0);
	}
	else if (alpha[1] < 0 && alpha[2] < 0) {//p_next lie out side of v2v0 and v0v1
		which_edges.push_back(2); which_edges.push_back(0);
	}
	else if (alpha[0] < 0) {//p_next lie out side of v1v2
		which_edges.push_back(1);
	}
	else if (alpha[1] < 0) {//p_next lie out side of v2v0
		which_edges.push_back(2);
	}
	else if (alpha[2] < 0) {//p_next lie out side of v0v1
		which_edges.push_back(0);
	}

	for (int i = 0; i < which_edges.size(); i++) {
		int ei = which_edges[i], ej = (ei + 1) % vn;
		int v1 = v[ei], v2 = v[ej];
		Eigen::Matrix<T, 1, -1> P1 = P[ei], P2 = P[ej];
		if (Intersection2LineSegments(p, p_next, P1, P2, t) == 1) {
			which_edge = ei;  //warning:  differnt from chen!
			//p_next(0) = P1[0] + t[1] * (P2[0] - P1[0]);
			//p_next(1) = P1[1] + t[1] * (P2[1] - P1[1]);
			p_next = P1 + t[1]*(P2 - P1);
			return;
		}
	}
}

// If we have already judge which edge the curve will cross
// We can use the edge information to get next triangle
template<typename T>
int PassEdge(const VectorField<T> &VF, int &face_id, int which_edge)
{
	////Using edge information to get next triangle
	int opposite_face_id = VF.TT(face_id, which_edge); // TT(f,i) 对应的triangle
	if (opposite_face_id >= 0) {
		face_id = opposite_face_id;
		return opposite_face_id;
	}
	return -1;
}


// return -1 表示失败，比如到达 边界了
template<typename T>
int  GetNextTriangle(const VectorField<T> &VF, int &face_id, const Eigen::Matrix<T, 1, -1> &p, 
	Eigen::Matrix<T, 1, -1> &p_next,
	T param_t[2], const bool forwardbackward,
	const vector<T> alpha,bool &passVertex)
{
	
	//We should put pass vertex testing here before testing crossing edge
	int PassWhichVert = CrossVertex2(VF, face_id, p_next, p, forwardbackward);
	if (PassWhichVert > 0) {
		p_next = VF.V.row(PassWhichVert); //update the p_next
		passVertex = true;
		return PassWhichVert; //pass vertex PassWhichVert-1
	}
	passVertex = false;

	//pass which edge
	int which_edge = -1;
	int prev_face_id = face_id;
	CrossBoundary3(VF, face_id, p, p_next,  alpha, which_edge, param_t);
	if (param_t[0] == -1 && param_t[1] == -1){
		face_id = prev_face_id;   ////something wrong here
		return -1;
	}

	////if not passing a vertex, judge which triangle it will enter later
	//PassEdge(VF,face_id, which_edge);
	if (which_edge < 0) {
		//something wrong!!!
		return -1;
	}
	int opposite_face_id = VF.TT(face_id, which_edge); // TT(f,i) 对应的triangle
	if (opposite_face_id >= 0) {
		face_id = opposite_face_id;
		return opposite_face_id;
	}		
	return -1;
}

template<typename T>
int TraceInATriangle(const VectorField<T> &VF,int face_id, const Eigen::Matrix<T, 1, -1> &p0,
	vector<Eigen::Matrix<T, 1, -1>> &trajectory, bool forward_backward)
{
	RowVector3i tri = VF.F.row(face_id);
	Eigen::Matrix<T, 1, Dynamic> ABC[3], Vector[3];
	for (int i = 0; i < 3; i++) {
		ABC[i] = VF.V.row(tri(i));
		Vector[i] = VF.VF.row(tri(i));
	}

	const int TRACESTEPS = 700;
	T htry = 0.02;
	Eigen::Matrix<T, 1, -1> p = p0, next_p=p0;
	vector<T> alpha(3);
	
	trajectory.push_back(p);
//	while(1)
	for (int i = 0; i < TRACESTEPS; i++) 
	{
		VF.GetBarycentricFacters(face_id, p, alpha);
		if (!ToNextPoint(VF, face_id, p, next_p, alpha, htry,forward_backward)) {
			return -3;//the curve reach a singularity
		}
		std::cout << next_p(0) << " " << next_p(1) << "\n";

		if (VF.GetBarycentricFacters(face_id, next_p, alpha)) {//if current point is inside the current triangle
			trajectory.push_back(next_p);
			p = next_p;		
		}
		else {// if current point is out of the current triangle
			bool PassVertornot = false;
			double t[2] = { 0. };
			int vid = GetNextTriangle(VF,face_id, p, next_p, t, forward_backward, alpha, PassVertornot);
			if (vid >=0) { 
				if (PassVertornot) {// pass vertex
					// go to a little to the inner of the triangle?
				}
				else {  //pass edge through next_p
					// go to a little to the inner of the triangle?
				}
				trajectory.push_back(next_p);
				p = next_p;				
			}
			else {// reach the boundary
				break;
			}
	//		std::cout << "face_id:=" << face_id << "\n";
		}		
	}	
	return 0;
}

//=========test tracing =====================

template <typename T>
void copyVectorField(Eigen::Matrix<T, -1, -1> &VF, const int width, const int height, const T*vx, const T *vy, const T *vz) {
	assert(vx);  assert(vy);
	int size = width*height;
	int dim = 2;  if (vz) dim = 3;
	VF.resize(size, dim);
	int idx;
	if (dim == 2) {
		for (int j = 0; j < height; j++) {
			for (int i = 0; i < width; i++) {
				idx = j*width + i;
				VF(idx, 0) = vx[idx]; VF(idx, 1) = vy[idx];
			}
		}
	}
	else {
		for (int j = 0; j < height; j++) {
			for (int i = 0; i < width; i++) {
				idx = j*width + i;
				VF(idx, 0) = vx[idx]; VF(idx, 1) = vy[idx]; VF(idx, 2) = vz[idx];
			}
		}
	}
}

//-------------------barycentric coordinates ----------------------------
// http://www.farinhansford.com/dianne/teaching/cse470/materials/BarycentricCoords.pdf
// Compute barycentric coordinates (u, v, w) for point p with respect to triangle (a, b, c)

void test_GetBarycentricFacters() {
	Matrix< double, 1, Dynamic> p, a, b, c;
	p.resize(1, 3); a.resize(1, 3); b.resize(1, 3); c.resize(1, 3);
	a << 1, 0, 0; 	b << 0, 1, 0;	c << 0, 0, 1;
	p << 1. / 3, 1. / 3, 1. / 3;

	vector<double> alpha(3);
	GetBarycentricFacters(p, a, b, c, alpha);
	std::cout << alpha[0] << ", " << alpha[1] << ", " << alpha[2] << "\n";
}

//-------------------------trace tragitory-------------------
template<typename T>
void generate_circle_field(T *dx, T *dy, int width, int height)
{
	T cx = width*0.5;
	T cy = height*0.5;;

	int x, y;
	T EPSILON = 1e-9;

	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			T deltaX = cx - x;
			T deltaY = cy - y;
			T dist = sqrt(deltaX * deltaX + deltaY * deltaY) + EPSILON;

			int idx = y * width + x;

			dx[idx] = deltaY / dist;
			dy[idx] = -deltaX / dist;

	//		std::cout << dx[idx] << " " << dy[idx] << "\n";
		}
	//	std::cout << "\n";
	}
}

#include <fstream>
void test_TraceInATriangle() {
	Eigen::MatrixXd V;	Eigen::MatrixXi F;
#if 0
	int width = 5, height = 4;
	Eigen::Matrix < double, 1, -1 > p; p.resize(1, 2);  //tracking start and its triangle
	int fid = 15; p(0) = 3.5; p(1) = 1.2;
#else
	int width = 21, height = 18;
	Eigen::Matrix < double, 1, -1 > p; p.resize(1, 2);  //tracking start and its triangle
	int fid = 40*8+11*2-1; p(0) = 10.5; p(1) = 8.2;
#endif

	grid2mesh(width, height, V, F);
	double *vx = 0, *vy = 0, *vz = 0;
	int size = width*width; 	vx = new double[size]; vy = new double[size];
	std::cout << "generate circle field!\n";
	generate_circle_field(vx, vy, width, height);
	Eigen::Matrix<double, -1, -1> VF;
	copyVectorField(VF, width, height, vx, vy, vz);
	IOFormat HeavyFmt(FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");
//	std::cout << VF.format(HeavyFmt) << "\n";


	vector<Eigen::Matrix<double, 1, -1>>  trajectory;
#if 0
	TraceInATriangle(fid, p, V, F, VF, trajectory, true);
#else
	VectorField<double> vField(V, F, VF);
    TraceInATriangle(vField,fid, p, trajectory, true);
	
	std::ofstream oF("D:\\python_ex\\points.txt");
	for (int i = 0; i < V.rows(); i++) {
		oF << V(i,0) << " " << V(i, 1) << " "<< VF(i, 0) << " " << VF(i, 1) << "\n";
	}
	oF.close();

	std::ofstream oFt("D:\\python_ex\\triangles.txt");
	for (int i = 0; i < F.rows(); i++) {
		oFt << F(i, 0) << " " << F(i, 1) << " " << F(i, 2)  << "\n";
	}
	oFt.close();


	std::ofstream oF2("D:\\python_ex\\trajectory.txt");
	for (int i = 0; i < trajectory.size(); i++) {
	//	std::cout << trajectory[i](0) << " " << trajectory[i](1) << "\n";
		oF2 << trajectory[i](0) << " " <<  trajectory[i](1) << "\n";
	}
	oF2.close();
#endif
}


