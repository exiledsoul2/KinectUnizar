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
	unsigned int KF;	//<! Keyframe ID
	double u;			//<! U where the point was observed
	double v;			//<! V where the point was observed

	patchSupport(unsigned int kf, double U, double V):
		KF(kf),
		u(U),
		v(V)
	{}
};

//texture.corners
// [TF TR BL BR ]

class Patch{
public:
	cv::Point3d uvd;					//<! (U,V,D) The U,V,D where the image was first observed
	cv::Point3d xyz;					//<! (X,Y,Z) with reference to the first Keyframe
	cv::Mat texture;					//<! Texture Patch extracted around (U,V)
	cv::Mat depth;						//<! Similar patch in depth but not used atm.
	unsigned int sourceKF;				//<! ID of the key frame in which the point was observed

	std::vector<cv::Point3d> cornersXYZ; //[TF TR BF BR]

	std::vector<patchSupport> supportList;	//<! reobservation in successive keyframes add support!
	Patch(){}

	Patch(cv::Point3d u, cv::Point3d x, cv::Mat tex, cv::Mat dep, unsigned int kf)
	{
		uvd = u;
		xyz = x;
		texture =  tex;
		depth = dep;
		sourceKF = kf;
	}

};


struct match
{
	int pointidx;
	float u;
	float v;
};

/**
 * Class that performs fucntion on the patch such as
 * 1. Affine Transformation
 */

class PatchProcessor{
	/// @ Todo : Not implemented yet
public:

	static cv::Mat getAffineTransformed(Patch &P, Eigen::Vector2f& centerR,Eigen::Vector2f& TLR, Eigen::Vector2f& TRR)
	{
		cv::Mat out;
		int halfApatch = PATCH_HEIGHT/2;
		cv::Point2f center(0,0);
		cv::Point2f TL (-halfApatch,-halfApatch);
		cv::Point2f TR (halfApatch,-halfApatch);

		cv::Point2f src[3]; src[0] = center; src[1]=center+TL; src[2] = center+TR;

		cv::Point2f c(centerR(0),centerR(1));
		cv::Point2f l(TLR(0),TLR(1));
		cv::Point2f r(TRR(0),TRR(1));

		cv::Point2f des[3]; des[0] = c-c; des[1]=l-c; des[2] = r-c;

		cv::Mat M =getAffineTransform(src,des);

		warpAffine(P.texture,out,M,cv::Size(PATCH_WIDTH,PATCH_WIDTH),cv::INTER_LINEAR|cv::WARP_INVERSE_MAP,cv::BORDER_REPLICATE);
		return out;

	}

	static cv::Mat normalizePatch(cv::Mat& P)
	{
		cv::Mat out = P - cv::mean(P);
		return out ;
	}
};

#define PatchList std::vector<Patch>
#define matchesList std::vector<match>
#endif /* PATCH_HPP_ */
