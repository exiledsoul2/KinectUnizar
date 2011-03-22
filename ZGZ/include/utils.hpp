/*
 * utils.h
 *
 *  Created on: Feb 27, 2011
 *      Author: yasir
 */

#ifndef _UTILS_H_
#define _UTILS_H_

using namespace Eigen;
using namespace cv;

namespace ZGZ
{
 namespace utils{
	int cv2Eigen(Mat& matIn,MatrixXf & matOut)
	{
		Map<MatrixXf> matOut((float*)matIn.data,matIn.rows,matIn.cols);
	}
 }
}

#endif /* UTILS_H_ */
