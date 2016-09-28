#ifndef MESH_H_2019_09_16_HWDONG
#define MESH_H_2019_09_16_HWDONG
#include <Eigen/Dense>
//#include <Eigen/Sparse>
#include <vector>
//#include <memory>
using namespace std;

//=================build adjacent info=======================
template <typename Index, typename IndexVector>
void vertex_vertex_adjacency(const Eigen::PlainObjectBase<Index>  & F,
	std::vector<std::vector<IndexVector> >& VV) {
	VV.clear();
	VV.resize(F.maxCoeff() + 1);

	// Loop over faces
	for (int i = 0; i<F.rows(); i++) {
		// Loop over this face
		for (int j = 0; j<F.cols(); j++) {
			// Get indices of edge: s --> d
			int s = F(i, j);
			int d = F(i, (j + 1) % F.cols());
			VV.at(s).push_back(d);
			VV.at(d).push_back(s);
		}
	}
	// Remove duplicates
	for (int i = 0; i<(int)VV.size(); ++i) {
		std::sort(VV[i].begin(), VV[i].end());
		VV[i].erase(std::unique(VV[i].begin(), VV[i].end()), VV[i].end());
	}

}

template <typename DerivedF, typename VFType>
void vertex_triangle_adjacency(
	const Eigen::PlainObjectBase<DerivedF>& F,
	std::vector<std::vector<VFType> >& VF)
{
	int n = F.maxCoeff() + 1;
	VF.clear();	VF.resize(n);

	typedef typename DerivedF::Index Index;
	for (Index fi = 0; fi<F.rows(); ++fi) {
		for (Index i = 0; i < F.cols(); ++i) {
			VF[F(fi, i)].push_back(fi);
		}
	}
}


// Compute triangle-triangle adjacency with indices
template <typename DerivedF, typename DerivedTT, typename DerivedTTi>
void triangle_triangle_adjacency(
	const Eigen::PlainObjectBase<DerivedF>& F,
	Eigen::PlainObjectBase<DerivedTT>& TT,
	Eigen::PlainObjectBase<DerivedTTi>& TTi)
{
	std::vector<std::vector<int> > TTT;
	//	triangle_triangle_adjacency_preprocess(F, TTT);
	for (int f = 0; f<F.rows(); ++f)
		for (int i = 0; i<F.cols(); ++i)
		{
			// v1 v2 f ei
			int v1 = F(f, i);
			int v2 = F(f, (i + 1) % F.cols());
			if (v1 > v2) std::swap(v1, v2);
			std::vector<int> r(4);
			r[0] = v1; r[1] = v2;
			r[2] = f;  r[3] = i;
			TTT.push_back(r);
		}
	std::sort(TTT.begin(), TTT.end());

	//	triangle_triangle_adjacency_extractTT(F, TTT, TT);
	TT.setConstant((int)(F.rows()), F.cols(), -1);

	for (int i = 1; i<(int)TTT.size(); ++i)
	{
		std::vector<int>& r1 = TTT[i - 1];
		std::vector<int>& r2 = TTT[i];
		if ((r1[0] == r2[0]) && (r1[1] == r2[1]))
		{
			TT(r1[2], r1[3]) = r2[2];
			TT(r2[2], r2[3]) = r1[2];
		}
	}

#if 0
	//	triangle_triangle_adjacency_extractTTi(F, TTT, TTi);
	TTi.setConstant((int)(F.rows()), F.cols(), -1);
	for (int i = 1; i<(int)TTT.size(); ++i)
	{
		std::vector<int>& r1 = TTT[i - 1];
		std::vector<int>& r2 = TTT[i];
		if ((r1[0] == r2[0]) && (r1[1] == r2[1]))
		{
			TTi(r1[2], r1[3]) = r2[3];
			TTi(r2[2], r2[3]) = r1[3];
		}
	}
#endif
	
}

// combine the above  functions:vertex_triangle_adjacency and triangle_triangle_adjacency
template <typename DerivedF, typename VFType, typename DerivedTT, typename DerivedTTi>
bool buildConnectivity(const Eigen::PlainObjectBase<DerivedF>& F,
	std::vector<std::vector<VFType> >& VT,
	Eigen::PlainObjectBase<DerivedTT>& TT,
	Eigen::PlainObjectBase<DerivedTTi>& TTi)
{
	if (F.rows() == 0) return false;//if (V.rows() == 0 || F.rows() == 0) return false;
									//V_border = igl::is_border_vertex(V, F);
									//vertex_vertex_adjacency(F, VV);
	vertex_triangle_adjacency(F, VT);
	std::cout << "has build the vertex_triangle connectivity!\n";
	triangle_triangle_adjacency(F, TT, TTi);
	std::cout << "has build the triangle_triangle connectivity!\n";
	// igl::edge_topology(V, F, E, F2E, E2F);

	return true;
}


//=======================grid2mesh=========================================
template <typename T>
void grid2mesh(const int width, const int height, Eigen::Matrix<T, -1, -1> &V, Eigen::MatrixXi &F)    //Eigen::MatrixXd V	Eigen::MatrixXi F;
{
	const int dim = 2;
	int size = width*height;
	V.resize(size, 2);
	F.resize(2 * (width - 1)*(height - 1), 3);
	T x, y;
	int idx = 0, fid = 0;
	for (int j = 0; j < height; j++) {
		for (int i = 0; i < width; i++) {
			idx = j*width + i;
			V(idx, 0) = i; V(idx, 1) = j; // V(idx, 2) = 0;
		}
	}
	int H = height - 1, W = width - 1;

	for (int j = 0; j<H; j++)
		for (int i = 0; i < W; i++) {
			idx = j*width + i;
		//	fid = j*8+2*i;
			F(fid, 0) = idx;
			F(fid, 1) = idx + width + 1;
			F(fid, 2) = idx + width ;
	//		std::cout << F.coeff(fid, 0) << ", " << F.coeff(fid, 1) << ", " << F.coeff(fid, 2) << "\n";

			fid++;
			F(fid, 0) = idx;   F(fid, 1) = idx + 1; F(fid, 2) = idx + width + 1;
	//		std::cout << F.coeff(fid, 0) << ", " << F.coeff(fid, 1) << ", " << F.coeff(fid, 2) << "\n";
			fid++;
		}
}

#if 1
#include <iostream>
inline void test_grid2mesh() {
	Eigen::MatrixXd V;	Eigen::MatrixXi F;
	int width = 5, height = 4;
	grid2mesh(width, height, V, F);

	int idx = 0;
	for (int j = 0; j < height; j++) {
		for (int i = 0; i < width; i++) {
			idx = j*width + i;
			std::cout << idx << "  ";
		}
		std::cout << "\n";
	}
	std::cout << "\n";

	for (int r = 0; r < V.rows(); r++) {
		if (r%width == 0) std::cout << "\n";
		for (int c = 0; c < V.cols(); c++)
			std::cout << V.coeff(r, c) << ", ";
		std::cout << "     ";
	}
	std::cout << "\n";


	std::cout << "\n";
	for (int r = 0; r < F.rows(); r++) {
		std::cout << F.coeff(r, 0) << ", " << F.coeff(r, 1) << ", " << F.coeff(r, 2) << "\n";
	}
}
#endif

#endif