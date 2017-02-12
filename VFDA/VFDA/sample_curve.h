#pragma once
#include "IsInClosedRegion.h"

template<typename T>
void resample_curve(Eigen::Matrix<T, Dynamic, Dynamic> &out_curve,
	const Eigen::Matrix<T, Dynamic, Dynamic> &curve, const int num)
{

	int n = curve.rows();
	assert(num <= n);
	vector<T> sampled_x, sampled_y, x(n), y(n);
	for (int i = 0; i < n; i++) {
		x[i] = curve(i, 0); y[i] = curve(i, 1);
	}
	resample_curve(sampled_x, sampled_y, x, y, num);
	out_curve.resize(num, curve.cols());
	for (int i = 0; i < num; i++) {
		out_curve(i, 0) = sampled_x[i];
		out_curve(i, 1) = sampled_y[i];
	}
}

template<typename T>
void interpolate_a_curve(Eigen::Matrix<T, Dynamic, Dynamic> &curve,
	const Eigen::Matrix<T, Dynamic, Dynamic> &start_curve,
	const Eigen::Matrix<T, Dynamic, Dynamic> &end_curve, const T t)
{
	assert(t > 0 && t < 1);
	assert(start_curve.size() == end_curve.size());
	curve.resize(start_curve.rows(), start_curve.cols());
	curve.setZero(start_curve.rows(), start_curve.cols());
	T t1 = 1 - t;
	for (int i = 0; i < curve.rows(); i++)
		for(int j = 0 ; j<curve.cols();j++)
			curve(i, j) = t1*start_curve(i, j) + t*end_curve(i, j);
}

template<typename T>
void interpolate_curves(std::vector<Eigen::Matrix<T, Dynamic, Dynamic>> &curves,
	const Eigen::Matrix<T, Dynamic, Dynamic> &start_curve,
	const Eigen::Matrix<T, Dynamic, Dynamic> &end_curve, const int num)
{
	int n = num + 1;
	T step = 1. / n, t = step;
	for (int i = 1; i < n; i++, t += step) {
		Eigen::Matrix<T, Dynamic, Dynamic> curve;
		interpolate_a_curve(curve, start_curve, end_curve, t);
		curves.push_back(curve);
	}
}

template<typename T>
void interpolate_curves(std::vector<Eigen::Matrix<T, Dynamic, Dynamic>> &curves,
	const Eigen::Matrix<T, Dynamic, Dynamic> &start_curve,
	const Eigen::Matrix<T, Dynamic, Dynamic> &end_curve, const int num_t, const int num_pts = 0)
{
	int num_pts_ = num_pts;
	if (num_pts_ == 0)
		num_pts_ = start_curve.size() < end_curve.size() ? start_curve.size() : end_curve.size();
	resample_curve(start_curve_, start_curve, num_pts_);
	resample_curve(end_curve_, end_curve, num_pts_);
	interpolate_curves(curves, start_curve_, end_curve_, num_t);
}