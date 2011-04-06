/*
 * patch.hpp
 *
 *  Created on: Apr 4, 2011
 *      Author: yasir
 */

#ifndef PATCH_HPP_
#define PATCH_HPP_

#include <ZGZ.hpp>

class patchSupport{
public:
	unsigned int KF;
	double u;
	double v;
	patchSupport(unsigned int kf, double U, double V):
		KF(kf),
		u(U),
		v(V)
	{}
};

class Patch{
public:
	cv::Point3d uvd;
	cv::Point3d xyz;
	cv::Mat texture;
	unsigned int sourceKF;
	std::vector<patchSupport> supportList;
	//!@TODO : Implement the affine transformation for patches.
	//cv::Mat transformAffine();
};

struct match
{
	int pointidx;
	float u;
	float v;
};

#define matchesList std::vector<match>
#endif /* PATCH_HPP_ */
