/*
 * patch.hpp
 *
 *  Created on: Apr 4, 2011
 *      Author: yasir
 */

#ifndef PATCH_HPP_
#define PATCH_HPP_

#include <ZGZ.hpp>
using namespace Eigen;
using namespace cv;

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

//texture.corners
// [TF TR BL BR ]

class Patch{
public:
	cv::Point3d uvd;
	cv::Point3d xyz;
	cv::Mat texture;
	cv::Mat depth;
	unsigned int sourceKF;
	std::vector<patchSupport> supportList;
	//!@TODO : Implement the affine transformation for patches.

	cv::Mat transformedTexture(Matrix3f K, Matrix3f R, Vector3f t)
	{
		Mat T = getTransform(K,R,t);
		Mat TransformedTexture;
		cv::warpAffine(texture,TransformedTexture,T,Size(texture.rows,texture.cols),INTER_LINEAR,BORDER_DEFAULT);
		return TransformedTexture;
	}

	cv::Mat getTransform(Matrix3f K, Matrix3f R, Vector3f t)
	{
		Matrix<float,3,4> p;
		p.col(0) << depth.at<unsigned short>(0,depth.cols-1);
		p.col(1) << depth.at<unsigned short>(0,0); // N
		p.col(2) << depth.at<unsigned short>(depth.rows-1,depth.cols-1);


		Matrix<float,3,4> p2;
		p2= K*R*(p+t.replicate(1,4));
		p2 = p2.array() / p2.array().row(2).replicate(4,1);
		//How we can use OpenCV  to calculate the affine transform between the current patch and the new patch
		cv::Point2f textureCorners[3], depthCorners[3];

		gettextureCorners(textureCorners);

		depthCorners[0].x = p2(1,1);
		depthCorners[0].y = p2(1,0);

		depthCorners[1].x = p2(1,1);
		depthCorners[1].y = p2(1,0);

		depthCorners[2].x = p2(1,1);
		depthCorners[2].y = p2(1,0);

		cv::Mat T = cv::getAffineTransform(textureCorners,depthCorners);
		return T;
	}

	void gettextureCorners(cv::Point2f* corners)
	{
		float x = uvd.x;
		float y = uvd.y;
		float r= texture.rows;
		float c = texture.cols;


	 corners[0].x = x -r/2;
	 corners[0].y = y - c/2;
	 corners[1].x = x - r/2;
	 corners[1].x = y + c/2;
	 corners[2].x =	x + r/2;
	 corners[2].y = y - c/2;
			// x +r/2, y+c/2;

	}
};

struct match
{
	int pointidx;
	float u;
	float v;
};

#define matchesList std::vector<match>
#endif /* PATCH_HPP_ */
