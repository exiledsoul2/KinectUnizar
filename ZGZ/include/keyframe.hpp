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

#define TrackerState State< Matrix<float,13,1>,Matrix<float,13,13> >

class KeyFrame{
	public:
		unsigned int id;
		Mat image;
		Mat depth;
		TrackerState state;
		KeyFrame(Mat& im,
				Mat& dep,
				TrackerState& st
				){
			image = im.clone();
			depth = dep.clone();
			state.x = st.x;
			state.P = st.P;
		}
};

class KeyFrameManager{
private:
	std::vector< KeyFrame > keyFrames;
	unsigned int count;
public:
	KeyFrameManager(): count(0) {}

	void addKeyFrame(
			cv::Mat& im,
			cv::Mat& dep,
			TrackerState& st
			)
	{
		keyFrames.insert(keyFrames.end(),KeyFrame(im,dep,st));
	}
	int numberOfkeyFrames()
	{
		return keyFrames.size();
	}
	int currentKeyFrameId(){
		return numberOfkeyFrames()-1;
	}

};


#endif /* KEYFRAME_HPP_ */
