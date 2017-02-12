#include "VectorField.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "Smooth.h"
inline int build_variable_indices(std::vector<int> &v_indices, std::vector<int> &vids, bool vids_is_free = true)
{
	//1. build  the indices for variables of linear system
	const int vn = v_indices.size();
	int j = 0;
	if (vids_is_free) {
		v_indices.resize(vn, -1);
		for (const auto vid : vids) {
			v_indices[vid] = j; j++;
		}
	}
	else { //vids is fixed vertices
		v_indices.resize(vn, 0);
		for (const auto vid : vids)
			v_indices[vid] = -1;

		for (int i = 0; i < v_indices.size(); i++) {
			if (v_indices[i] == 0) {
				v_indices[i] = j; j++;
			}
		}
	}
	return j;
}

int build_variable_indices(std::vector<int> &v_indices, std::set<int> &vids, bool vids_is_free)
{
	//1. build  the indices for variables of linear system
	const int vn = v_indices.size();
	int j = 0;
	if (vids_is_free) {
		v_indices.resize(vn, -1);
		for (const auto vid : vids) {
			v_indices[vid] = j; j++;
		}
	}
	else { //vids is fixed vertices
		v_indices.resize(vn, 0);
		for (const auto vid : vids)
			v_indices[vid] = -1;

		for (int i = 0; i < v_indices.size(); i++) {
			if (v_indices[i] == 0) {
				v_indices[i] = j; j++;
			}
		}
	}
	return j;
}


void smooth(VectorField<float> &VF, const vector<int> &v_indices, const int n_unknowns )
{
	typedef float T;
	int nUnknowns = n_unknowns;
	if (nUnknowns == 0)
		for (const auto flag : v_indices)
			if (flag < 0) nUnknowns++;

	//2. build linear system	

	const int channels = VF.VF.cols();
	std::vector< Eigen::Triplet<float> > lhsTriplets;
	lhsTriplets.reserve(nUnknowns * 5);

	Eigen::Matrix<T, -1, -1> rhs(nUnknowns, channels); //Eigen::MatrixXf rhs(nUnknowns, channels);
	rhs.setZero();

	for (int i = 0; i < v_indices.size(); i++) {
		const int varible_id = v_indices[i];
		if (varible_id == -1)
			continue;
		vector<int> neightbors = VF.VV[i];
#if 0
		vector<int> weights = Vf.VVW[i];
		T weight = 0;
		for (int j = 0; j < weights.size(); j++)
			weight += weights[j];
#else  //uniform weights
		vector<T> weights(neightbors.size(), -1);
		T weight = neightbors.size();
#endif

		for (int j = 0; j < neightbors.size(); j++) {
			int neighbor_v = neightbors[j];
			if (v_indices[neighbor_v] == -1) { //boundary
				rhs.row(varible_id) -= weights[j] * VF.VF.row(neighbor_v);
			}
			else {
				lhsTriplets.push_back(Eigen::Triplet<float>(
					varible_id, v_indices[neighbor_v], weights[j]));
			}
		}
		lhsTriplets.push_back(Eigen::Triplet<float>(
			varible_id, varible_id, weight));
		// Add f to rhs.
		//rhs.row(pid) += VF.VF.row();
	}

	//3.  Solve the sparse linear system of equations

	Eigen::SparseMatrix<T> A(nUnknowns, nUnknowns);
	A.setFromTriplets(lhsTriplets.begin(), lhsTriplets.end());

	Eigen::SparseLU< Eigen::SparseMatrix<T> > solver;
	solver.analyzePattern(A);
	solver.factorize(A);

	Eigen::Matrix<T, -1, -1>result(nUnknowns, channels);
	//	Eigen::MatrixXf result(nUnknowns, channels);
	for (int c = 0; c < channels; ++c)
		result.col(c) = solver.solve(rhs.col(c));

	//4. Copy results back
	for (int i = 0; i < v_indices.size(); i++) {
		const int varible_id = v_indices[i];
		if (varible_id == -1)
			continue;
		VF.VF.row(i) = result.row(varible_id);
	}
}