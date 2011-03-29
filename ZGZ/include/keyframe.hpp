/*
 * keyframe.hpp
 *
 *  Created on: Mar 27, 2011
 *      Author: yasir
 */

#ifndef KEYFRAME_HPP_
#define KEYFRAME_HPP_

#include <ZGZ.hpp>
#include <state.hpp>
using namespace cv;
using namespace Eigen;

template <typename stateVector_t, typename covariance_t>
class KeyFrame{
	public:
		Mat image;
		Mat depth;
		std::vector<KeyPoint> keyPoints;
		State<stateVector_t,covariance_t> state;
		KeyFrame(Mat& im,
				Mat& dep,
				std::vector<KeyPoint>& kpnts,
				State<stateVector_t,covariance_t>& st
				){
			image = im.clone();
			depth = dep.clone();
			keyPoints = kpnts;
			state.x = st.x;
			state.P = st.P;
		}
};

#define TrackerKeyFrame KeyFrame< Matrix<float,13,1>,Matrix<float,13,13> >
#define KeyFrameVector std::vector<TrackerKeyFrame>
#endif /* KEYFRAME_HPP_ */
