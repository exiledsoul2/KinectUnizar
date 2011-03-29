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

using namespace ZGZ::zcv;

#define DATASET_PATH "/home/yasir/CODE/KinectUnizar/KinectCapture/Debug/desk/"

void initTracker(Tracker& tracker)
{
	//Initalizing the state vector and covariance
	tracker.Xk().x << 0,0,0,0,0,0,1,0,0,0,W_EPS,W_EPS,W_EPS;
	tracker.Xk().P.setZero();
	//tracker.Xk().P.setIdentity();
	//tracker.Xk().P *= W_EPS;
	tracker.Xk().P.block(3,3,3,3) = I3*W_EPS;
	tracker.Xk().P.bottomRightCorner(3,3)=I3*W_EPS;

	// Model Noise covariance
	tracker.Q().setZero();
	tracker.Q().block<3,3>(3,3) = I3;
	tracker.Q().block<3,3>(10,10) = I3;
	// The measurement Noise covariance depends on the measurement. init later
}

int main()
{
	camera K;
	Tracker tracker;
	KeyFrameVector keyframes;

	initTracker(tracker);

	ImageSource imsrc(DATASET_PATH,	"",	"ppm",100,1100,4);
	ImageSource depth(DATASET_PATH,	"",	"dep",100,1100,4);

	KeyPointsVector kpnts1,kpnts2;
	std::vector<Patch> patchList1,patchList2;
	K.fastFeatureDetector(DETECTOR_THRESHOLD,1);
	if(K.readCalibration("./calibration_rgb.yaml","camera_matrix","distortion_coefficients")==ZGZ_ERROR){
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
	K.extractPatches(kpnts1,patchList1,tracker.Xk().Rot(),tracker.Xk().t());

	std::cout<<tracker.Xk().Rot()<<std::endl<<tracker.Xk().t()<<std::endl;

	keyframes.insert(keyframes.end(),TrackerKeyFrame(K.currentImage(),K.currentDepth(),kpnts1,tracker.Xk()));

	while(!imsrc.done()){

	std::cerr<<"Current number of patches : "<<patchList1.size()<<std::endl;
	//K.showPatchesOnImage(kpnts1,"Patches1");
	//K.showKeyPoints(kpnts1,"KeyPoints1");

	tracker.predict();

	imsrc.getNextImage(K.currentImage());
	depth.getNextImage(K.currentDepth());

	kpnts2.clear();
	tracker.findMatches(K.currentImage(),CameraK,patchList1,kpnts2,nMatches,true,dummy);
	tracker.update();

	std::cout<<tracker.Xk().x<<std::endl;
	std::cout<<tracker.Xk().P.block<3,3>(0,0)<<std::endl;

	std::cout<<"Found "<<nMatches<<" matches"<<std::endl;
	std::cout<<"Keypoints retained "<<kpnts2.size()<< " used for next Image"<<std::endl;
	imshow("Predictions&Matches",dummy);
	//K.extractPatches(kpnts2,patchList2);
	if(nMatches<MIN_MATCHES_THRESHOLD)
	{
		//kpnts1.clear();
		//patchList1.clear();
		K.detectPoints(kpnts1);
		K.extractPatches(kpnts1,patchList1,tracker.Xk().Rot(),tracker.Xk().t());
		keyframes.insert(keyframes.end(),TrackerKeyFrame(K.currentImage(),K.currentDepth(),kpnts1,tracker.Xk()));
		std::cout<<"Added new key frame "<<std::endl;
		std::cin.get();
	}
	K.showPatchesOnImage(kpnts2,"NEWpatches");
	cvWaitKey(10);
	}
	cvWaitKey();

}



