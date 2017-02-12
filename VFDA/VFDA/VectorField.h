#pragma once

//https://en.wikipedia.org/wiki/Barycentric_coordinate_system
//http://gamedev.stackexchange.com/questions/23743/whats-the-most-efficient-way-to-find-barycentric-coordinates
//http://stackoverflow.com/questions/25385361/point-within-a-triangle-barycentric-co-ordinates
//http://www.cc.gatech.edu/classes/AY2015/cs4496_spring/Eigen.html

#include "Mesh.h"
#include <iostream>
using namespace std;
using namespace Eigen;

#include <vector>
#include <fstream>
inline bool saveConnectivity(const char *filename, const  std::vector<std::vector<int> > &VT,
	const Eigen::MatrixXi &TT)
{
	std::ofstream oF(filename);
	if (!oF) return false;
	oF << VT.size() << " " << TT.rows() << "\n";
	for (int i = 0; i < VT.size(); i++) {
		int n = VT[i].size();
		oF << n << " ";
		for (int j = 0; j<n; j++)
			oF << VT[i][j] << " ";
		oF << "\n";
	}

	for (int i = 0; i < TT.rows(); i++) {
		oF << TT(i, 0) << " " << TT(i, 1) << " " << TT(i, 2) << "\n";
	}
	oF.close();
	return true;
}

inline bool readConnectivity(const char *filename, std::vector<std::vector<int> > &VT,
	Eigen::MatrixXi &TT)
{
	std::ifstream iF(filename);
	if (!iF) return false;
	int vn, tn;
	iF >> vn >> tn;
	VT.resize(vn);
	TT.resize(tn, 3);
	for (int i = 0; i < vn; i++) {
		int n;
		iF >> n;
		VT[i].resize(n);
		for (int j = 0; j<n; j++)
			iF >> VT[i][j];
	}

	for (int i = 0; i < tn; i++) {
		iF >> TT(i, 0) >> TT(i, 1) >> TT(i, 2);
	}
	iF.close();
	return true;
}


template <typename T>
class VectorField {
public:
	Eigen::Matrix<T, -1, -1> V;
	Eigen::MatrixXi F;
	Eigen::Matrix<T, -1, -1> VF;
	std::vector<std::vector<int> > VT,VV;
	Eigen::MatrixXi TT , TTi;

	VectorField(Eigen::Matrix<T, -1, -1> &V_, Eigen::MatrixXi &F_, Eigen::Matrix<T, -1, -1> &VF_,
		const char *filename = 0) {
		V = V_; F = F_; 
		if (VF_.rows() < V_.rows()) {
			VF.resize(V_.rows(), V_.cols());
			VF.setZero(V_.rows(), V_.cols());
		}
		else VF = VF_;

		if (!filename) {
			buildConnectivity();
			saveConnectivity("VTT.txt", VT, TT);
		}
		else 
			readConnectivity("VTT.txt", VT, TT);
		vertex_vertex_adjacency(F,VV);
	}
	
	bool buildConnectivity() {
		return ::buildConnectivity(F, VT, TT, TTi);
	}

	bool GetBarycentricFacters(const int face_id, const Matrix< T, 1, Dynamic>& p,
		vector<T> &alpha) const
	{
		RowVector3i tri = F.row(face_id);
		return ::GetBarycentricFacters<T>(p, V.row(tri(0)), V.row(tri(1)), V.row(tri(2)), alpha);
	}
	bool GetBarycentricFacters(const int face_id, const vector<T>& p,
		vector<T> &alpha) const
	{
		Matrix< T, 1, Dynamic> p0(1, p.size());
		
		for (int j = 0; j < p.size(); j++)
			p0[j] = p[j];
		return GetBarycentricFacters(face_id, p0, alpha);
	}
	//
	Eigen::Matrix<T, 1, Dynamic> GetVectorAtPoints(const int face_id, const vector<T> &alpha)const
	{
		RowVector3i tri = F.row(face_id);
		Eigen::Matrix<T, 1, Dynamic> pV = VF.row(tri(0)) * alpha[0]
			+ VF.row(tri(1)) * alpha[1] + VF.row(tri(2)) * alpha[2];
		return pV;
	}
};

template <typename T>
void VectorField2grid(const VectorField<T> &VF,vector<T>& vx, vector<T>& vy,
	int &width,int &height) 
{
	int vn = VF.V.rows();
	vx.resize(vn); vy.resize(vn);
	width = 0; height = 0;
	for (int r = 0; r < vn; r++) {
		if (VF.V(r, 0) > width) width = VF.V(r, 0);
		if (VF.V(r, 1) > height) height = VF.V(r, 1);
		vx[r] = VF.VF(r, 0); vy[r] = VF.VF(r, 1);
	}
	width++, height++;
}


#include <fstream>
template<typename T>
void save_vectorfield_for_test(VectorField<T> *&vField) {
	std::ofstream oF("D:\\python_ex\\points.txt");
	for (int i = 0; i < vField->V.rows(); i++) {
		oF << vField->V(i, 0) << " " << vField->V(i, 1) << " "
			<< vField->VF(i, 0) << " " << vField->VF(i, 1) << "\n";
	}
	oF.close();

	std::ofstream oFt("D:\\python_ex\\triangles.txt");
	for (int i = 0; i < vField->F.rows(); i++) {
		oFt << vField->F(i, 0) << " " << vField->F(i, 1) << " " << vField->F(i, 2) << "\n";
	}
	oFt.close();
/*
	std::ofstream oF2("D:\\python_ex\\trajectory.txt");
	for (int i = 0; i < singularities.size(); i++) {
		//	std::cout << trajectory[i](0) << " " << trajectory[i](1) << "\n";
		oF2 << vField->singularities[i].pos[0] << " " << vField->singularities[i].pos[1] << "\n";
	}
	oF2.close();
	*/

}