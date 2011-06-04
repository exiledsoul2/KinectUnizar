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

#include <opencv2/opencv.hpp>
#include <utils/shortcuts.hpp>


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

//#define FOVH 1.0144686707507438
//#define FOVV 0.78980943449644714

#define MATCHING_METHOD CV_TM_CCORR_NORMED

#define MATCHING_THRESHOLD .8
#define DETECTOR_THRESHOLD 100

#define MIN_MATCHES_THRESHOLD 50

#define IMG_ROWS 	480
#define IMG_COLS	640

#define filterDataType float	//!< Defines the primary datatype of the filter
#define TS (1.0/30.0)
#define W_EPS 1e-15

};
#endif /* ZGZ_HPP_ */
