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
#include <settings.hpp>
#include <kdtree.hpp>

#include <osgdisplay.hpp>

//using namespace ZGZ::zcv;

//#define DATASET_PATH "/home/yasir/CODE/KinectUnizar/KinectCapture/Debug/datasets/shelf/"
#define DATASET_PATH "/home/yasir/CODE/KinectUnizar/KinectCapture/Debug/walls/"
//#define DATASET_PATH "/home/yasir/CODE/KinectUnizar/KinectCapture/Debug/checkerboard/"
//initSettings(options);

Settings options;



int main(int argc, char**argv)
{
	pKDTree kDtree;

	camera K;
	Tracker tracker;
	KeyFrameManager keyFrameManager;
	matchesList matches;

	osgDisplay disp3d;

	initSettings(options);
	tracker.init();


	int start, end;
	if(argc<3){
		std::cerr<<"Starting and ending Image Miising"<<std::endl;
		return -1;
	}

	else {
		start = atoi(argv[1]);
		end = atoi(argv[2]);
	}


	ImageSource imsrc(DATASET_PATH,	"",	"ppm",start,end,1,4);
	ImageSource depth(DATASET_PATH,	"",	"dep",start,end,1,4);

	KeyPointsVector kpnts1,kpnts2;

	PatchList gPatchList,temp;


	K.fastFeatureDetector(DETECTOR_THRESHOLD,1);

	if(K.readCalibration("./calibration_rgb2.yaml","camera_matrix","distortion_coefficients")==ZGZ_ERROR){
		std::cerr<<"Unable to read calibration params"<<std::endl;
		return -1;
	}

	int nMatches;
	Mat dummy;
	Mat CameraK = K.K();
	//Mat detectorMask;

	imsrc.getNextImage(K.currentImage());
	depth.getNextImage(K.currentDepth());

	//Extract Patches and the points in  XYZ are assumed to be the
	// the ground truth points. (uv may change, but XYZ will not)

	K.detectPoints(kpnts1);
	K.extractPatches(kpnts1,gPatchList,tracker.Xk().Rot(),tracker.Xk().t(),keyFrameManager.currentKeyFrameId(),kDtree);

	std::cerr<<"First frame : Points  "<<gPatchList.size()<<std::endl;

	keyFrameManager.addKeyFrame(K.currentImage(),K.currentDepth(),tracker.Xk());

	while(!imsrc.done()){

	std::cerr<<"Current number of patches : "<<gPatchList.size()<<std::endl;

	tracker.predict();

	imsrc.getNextImage(K.currentImage());
	depth.getNextImage(K.currentDepth());

	matches.clear();

	tracker.findMatches(K.currentImage(),K.currentDepth(),CameraK,gPatchList,matches,nMatches,true,dummy);//,detectorMask);

	//imshow("detectorMask",detectorMask);

	//if(matches.size()>= MIN_MATCHES_THRESHOLD)
	//	tracker.update();
	//	continue;



	std::cout<<tracker.Xk().x<<std::endl;
	std::cout<<tracker.Xk().P<<std::endl;

	std::cout<<"Found "<<nMatches<<" matches"<<std::endl;

	imshow("Predictions&Matches",dummy);

	if(nMatches<MIN_MATCHES_THRESHOLD)
	{
		K.addSupport(gPatchList,matches,keyFrameManager.currentKeyFrameId());
		kpnts2.clear();
		//gPatchList.clear();
		K.detectPoints(kpnts2);
		keyFrameManager.addKeyFrame(K.currentImage(),K.currentDepth(),tracker.Xk());
		K.extractPatches(kpnts2,gPatchList,tracker.Xk().Rot(),tracker.Xk().t(),keyFrameManager.currentKeyFrameId(),kDtree);
		K.showPatchesOnImage(kpnts2,"points in new KeyFrame");
	}
	else
	{
	}

	K.showMatches(gPatchList,matches,"matches found");
    disp3d.display(gPatchList, tracker.Xk().x);
	cvWaitKey();
	}
	disp3d.loop();
	cvWaitKey();

}



