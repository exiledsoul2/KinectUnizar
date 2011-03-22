/*
 * ZGZ.hpp
 *
 *  Created on: Feb 27, 2011
 *      Author: yasir
 */

#ifndef ZGZ_HPP_
#define ZGZ_HPP_

#include <Eigen/Core>
#include <Eigen/Dense>

//#include <opencv2/core/core.hpp>
//#include <opencv2/features2d/features2d.hpp>
//#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/opencv.hpp>



namespace ZGZ
{
	namespace zEigen = Eigen;
	namespace zcv = cv;

	typedef enum {
		ZGZ_ERROR=-1,
		ZGZ_OK = 0,
	}ZGZ_RETURN_STATUS;

#define PATCH_WIDTH 15
#define PATCH_HEIGHT 15
#define IMAGE_COLS 640
#define IMAGE_ROWS 480
#define FOVH 1.0144686707507438
#define FOVV 0.78980943449644714
#define MATCHING_METHOD CV_TM_CCORR_NORMED
#define MATCHING_THRESHOLD 0.8

	typedef struct Patch{
		cv::Point3d uvd;
		cv::Point3d xyz;
		cv::Mat texture;
	}Patch;

#define IMG_ROWS 	480
#define IMG_COLS	640

}

#endif /* ZGZ_HPP_ */
