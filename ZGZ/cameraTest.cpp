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



using namespace ZGZ::zcv;

#define DATASET_PATH "/home/yasir/CODE/KinectUnizar/KinectCapture/Debug/translation_aligned/"

int main()
{
	camera K;
	Tracker tracker;

	//Initalizing the state vector and covariance
	tracker.Xk().x << 0,0,0,0,0,0,1,0,0,0,W_EPS,W_EPS,W_EPS;
	tracker.Xk().P.setZero();
	//tracker.Xk().P.block(3,3,3,3) = I3*W_EPS;
	tracker.Xk().P.bottomRightCorner(3,3)=I3*W_EPS;

	// Model Noise covariance
	tracker.Q().setZero();
	tracker.Q().block<3,3>(3,3) = I3;
	tracker.Q().block<3,3>(10,10) = I3;
	// The measurement Noise covariance depends on the measurement. init later

	ImageSource imsrc(DATASET_PATH,	"",	"ppm",200,900,4);
	ImageSource depth(DATASET_PATH,	"",	"dep",200,900,4);

	KeyPointsVector kpnts1,kpnts2;
	std::vector<Patch> patchList1,patchList2;
	K.fastFeatureDetector(70,1);

	int nMatches;
	Mat dummy;

	double fu = 5.9421434211923247e+02;
	double fv = 5.9104053696870778e+02;

	imsrc.getNextImage(K.currentImage());
	depth.getNextImage(K.currentDepth());
	K.detectAndExtractPatches(kpnts1,patchList1);

	while(!imsrc.done()){

	//K.showPatchesOnImage(kpnts1,"Patches1");
	//K.showKeyPoints(kpnts1,"KeyPoints1");

	tracker.predict();
	tracker.constructH(patchList1,tracker.Xkp1().x.block<3,1>(0,0),fu,fv);

	imsrc.getNextImage(K.currentImage());
	depth.getNextImage(K.currentDepth());

	tracker.findMatches(K.currentImage(),patchList1,kpnts2,nMatches,0,dummy);
	tracker.update();

	//imshow("Predictions&Matches",dummy);
	patchList1.clear();
	K.extractPatches(kpnts2,patchList1);
	std::cout<<"size :"<<patchList1.size()<<std::endl;

	kpnts1.clear();
	kpnts1 = kpnts2;

	//K.showPatchesOnImage(kpnts1,"Patches2");
	//K.showKeyPoints(kpnts1,"KeyPoints2");

	std::cout<<nMatches<<std::endl;
	std::cout<<tracker.Xk().P<<std::endl;
	cvWaitKey(10);
	}
	cvWaitKey();

}


