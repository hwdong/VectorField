#pragma once
#include "VectorField.h"
#include <set>

int build_variable_indices(std::vector<int> &v_indices, std::set<int> &vids, bool vids_is_free = true);
void smooth(VectorField<float> &VF,const  std::vector<int> &v_indices,const int n_unknowns=0);
