/*
 * camera.h
 *
 *  Created on: Feb 27, 2011
 *      Author: yasir
 */

#ifndef CAMERA_H_
#define CAMERA_H_

#include <iostream>
#include <ZGZ.hpp>
#include <patch.hpp>

using namespace std;
using namespace ZGZ::zEigen;
using namespace ZGZ::zcv;

#define KeyPointsVector std::vector<KeyPoint>
#define PatchesVector std::vector<Patch>

using namespace ZGZ;


class camera
{
	private:
	Mat 	_K;
	Mat 	_k1;
	Mat 	_currentImage;
	Mat 	_currentDepth;
	bool 	_isCalibrated;

	string 	_model;
	FeatureDetector * _ffd;


	public:

		camera():_model("pinhole"){
			_K = Mat::eye(3,3,CV_32F);
			_k1 = Mat::zeros(1,4,CV_32F);

			_isCalibrated = false;
			_ffd = NULL;
		}
		/**
		 *
		 * @param modelname	projective model used for the camera
		 * @param k Intrinsic calibration matrix
		 * @param distcoeff vector of distortion coeffecients
		 * @return
		 */
		camera(std::string modelname,Mat k,Mat distcoeff){
			_model 	= modelname;
			_K		= k;
			_k1 = distcoeff;
			_isCalibrated = true;
			_ffd = NULL;
		}
		/**
		 *
		 * @param filename name of YML calibration file
		 * @param nameInFileK actual name of K in file
		 * @param nameInFilek1 actual name of k1 in file
		 * @return Error status
		 */
		ZGZ::ZGZ_RETURN_STATUS readCalibration(const string filename, const string nameInFileK, const string nameInFilek1);
		/**
		 *
		 * @param pointList 2xN list of points to be undistorted with (K,distCoeffs)
		 * @return 2xN list of undistorted points
		 */
		MatrixXf rectifyPoints(MatrixXf pointList);		/**
		 *
		 * @param image image to be undistorted
		 * @return	undistorted image
		 */
		MatrixXf rectifyImage(MatrixXf image);
		/*
		 * @return true when the camera has calibration
		 */
		bool isCalibrated();

		/*
		 *@param threshold
		 *@param nonMaximalSuppression
		 */
		void fastFeatureDetector(int threshold, int nonMaximalSuppression);
		void detectPoints(KeyPointsVector&);
		void extractPatches(KeyPointsVector, PatchesVector&, Matrix3f, Vector3f,unsigned int);
		void undistortPoints(KeyPointsVector& k);
		void showKeyPoints(const KeyPointsVector& k,std::string windowname);
		Mat& currentImage();
		Mat& currentDepth();
		void showPatchesOnImage(KeyPointsVector,const char*);
		void detectAndExtractPatches(KeyPointsVector&,PatchesVector&);

		/**
		 * Matching patches between previous and current Image
		 * @param prevImage	patches in the previous image
		 * @param searchRegion the region around the key point in the currentImage
		 * @param currentImage patches in the current image
		 * @param result
		 */
		//void matchKeyPoints(
		//		const PatchesVector prevImage,
		//		const cv::Size2i searchRegion,
		//		const KeyPointsVector currentImage,
		//		KeyPointsVector& result
		//		);
		Vector3f toWorldXYZ(Point3d uvd);
		Mat& K();
		void showMatches(const PatchesVector& p,matchesList& M, const char* windowName);
		void addSupport(PatchesVector& p, matchesList& matches, unsigned int keyFrameID);

};

#endif /* CAMERA_H_ */
