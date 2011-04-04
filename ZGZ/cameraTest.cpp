/*
 * cameraTest.cpp
 *
 *  Created on: Feb 27, 2011
 *      Author: yasir
 */
#include <camera.hpp>
#include <imagesource.hpp>
#include <ZGZ.hpp>
#include <KalmanTracker.hpp>
#include <keyframe.hpp>
#include <patch.hpp>

using namespace ZGZ::zcv;

#define DATASET_PATH "/home/yasir/CODE/KinectUnizar/KinectCapture/Debug/translation/"

void initTracker(Tracker& tracker)
{
	//Initalizing the state vector and covariance
	tracker.Xk().x << 0,0,0,0,0,0,1,0,0,0,W_EPS,W_EPS,W_EPS;
	tracker.Xk().P.setZero();
	//tracker.Xk().P.setIdentity();
	tracker.Xk().P *= W_EPS;
	//tracker.Xk().P.block(3,3,3,3) = I3*W_EPS;
	tracker.Xk().P.bottomRightCorner(3,3)=I3*W_EPS;

	// Model Noise covariance
	tracker.Q().setZero();
	tracker.Q().block<3,3>(vx,vx) = I3*9;
	tracker.Q().block<3,3>(wx,wx) = I3*9;
	// The measurement Noise covariance depends on the measurement. init later
}

int main(int argc, char**argv)
{
	camera K;
	Tracker tracker;
	KeyFrameManager keyFrameManager;
	matchesList matches;

	initTracker(tracker);

	int start;
	if(argc<2){
		std::cerr<<"Starting Image Miising"<<std::endl;
		return -1;
	}

	else start = atoi(argv[1]);

	ImageSource imsrc(DATASET_PATH,	"",	"ppm",start,1100,4);
	ImageSource depth(DATASET_PATH,	"",	"dep",start,1100,4);

	KeyPointsVector kpnts1,kpnts2;
	std::vector<Patch> patchList1,patchList2;
	K.fastFeatureDetector(DETECTOR_THRESHOLD,1);
	if(K.readCalibration("./calibration2.yml","rgb_intrinsics","rgb_distortion")==ZGZ_ERROR){
		std::cerr<<"Unable to read calibration params"<<std::endl;
		return -1;
	}

	int nMatches;
	Mat dummy;
	Mat CameraK = K.K();

	imsrc.getNextImage(K.currentImage());
	depth.getNextImage(K.currentDepth());

	//Extract Patches and the points in  XYZ are assumed to be the
	// the ground truth points. (uv may change, but XYZ will not)

	K.detectPoints(kpnts1);
	K.extractPatches(kpnts1,patchList1,tracker.Xk().Rot(),tracker.Xk().t(),keyFrameManager.currentKeyFrameId());

	keyFrameManager.addKeyFrame(K.currentImage(),K.currentDepth(),tracker.Xk());

	while(!imsrc.done()){

	std::cerr<<"Current number of patches : "<<patchList1.size()<<std::endl;

	tracker.predict();

	imsrc.getNextImage(K.currentImage());
	depth.getNextImage(K.currentDepth());

	matches.clear();
	tracker.findMatches(K.currentImage(),CameraK,patchList1,matches,nMatches,true,dummy);
	tracker.update();

	std::cout<<tracker.Xk().x<<std::endl;
	std::cout<<tracker.Xk().P<<std::endl;

	std::cout<<"Found "<<nMatches<<" matches"<<std::endl;
	std::cout<<"Keypoints retained "<<kpnts2.size()<< " used for next Image"<<std::endl;
	imshow("Predictions&Matches",dummy);

	if(nMatches<MIN_MATCHES_THRESHOLD)
	{
		K.addSupport(patchList1,matches,keyFrameManager.currentKeyFrameId());
		kpnts2.clear();
		K.detectPoints(kpnts2);
		keyFrameManager.addKeyFrame(K.currentImage(),K.currentDepth(),tracker.Xk());
		K.extractPatches(kpnts2,patchList1,tracker.Xk().Rot(),tracker.Xk().t(),keyFrameManager.currentKeyFrameId());
		K.showPatchesOnImage(kpnts2,"points in new KeyFrame");

		//waitKey();
	}
	else
	{
		//K.extractPatches(kpnts2,patchList2,tracker.Xk().Rot(),tracker.Xk().t());
		//patchList1 = patchList2;
		//patchList2.clear();

	}

	K.showMatches(patchList1,matches,"NEWpatches");
	cvWaitKey(10);
	}
	cvWaitKey();

}



