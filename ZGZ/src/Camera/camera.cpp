/*
 * camera.cpp
 *
 *  Created on: Feb 27, 2011
 *      Author: yasir
 */

#include <ZGZ.hpp>
#include <camera.hpp>
#include <math.h>
#include <settings.hpp>
using namespace ZGZ;

ZGZ::ZGZ_RETURN_STATUS camera::readCalibration(std::string filename, const string nameK, const string namek1)
{
	FileStorage fs;
	fs.open(filename,FileStorage::READ);
	Mat matK;
	Mat matK1;

	if(!fs.isOpened())
		return ZGZ::ZGZ_ERROR;

	fs[nameK]>>_K;
	fs[namek1]>>_k1;

	_isCalibrated = true;
	return ZGZ::ZGZ_OK;
}

bool camera::isCalibrated(){
	return _isCalibrated;
}

void camera::fastFeatureDetector(int threshold, int nonMaximalSuppression){
	if(_ffd==NULL)
		if(options.detector.gridAdapted){
			_ffd = new GridAdaptedFeatureDetector(
					//new GoodFeaturesToTrackDetector(1,0.5,1,7),
					new SurfFeatureDetector(200,3,4),
					//new FastFeatureDetector(30,1),
					options.detector.maxNumFeatures,
					options.detector.gridRows,
					options.detector.gridCols
					);
		}
		else
		{
			_ffd = new GoodFeaturesToTrackDetector(options.detector.maxNumFeatures,0.10,5,9);
		}

		//_ffd = new FastFeatureDetector(threshold,nonMaximalSuppression);
	else{
		delete _ffd;
		_ffd = NULL;
		FastFeatureDetector(threshold,nonMaximalSuppression);
	}

}

void camera::detectPoints(KeyPointsVector& k, const Mat& mask){
	if(_ffd)
		_ffd->detect(_currentImage,k,mask);
	else
		std::cerr<<"ERROR : Fast Feature Detector is uninitialized";

}

void camera::extractPatches(
		KeyPointsVector K,
		PatchList& patchList,
		Matrix3f R,
		Vector3f t,
		unsigned int keyFrameCount,
		pKDTree& kdtree
		){
	std::vector<KeyPoint>::iterator iter;

	int patchSize = PATCH_HEIGHT;
	for(iter = K.begin();iter != K.end(); iter++)
	{
		int x = iter->pt.x;
		int y = iter->pt.y;
		int xOrigin = x - (patchSize-1)/2;
		int yOrigin = y - (patchSize -1)/2;

		// Check if a valid patch can be extracted
		if(xOrigin < 0 || yOrigin < 0) continue;
		if((xOrigin+patchSize)>IMAGE_COLS ||(yOrigin+patchSize)>IMAGE_ROWS) continue;

		// Check if we have valid depth for the current point
		float depth = _currentDepth.at<unsigned short int>(y,x);
		if( depth < 1 ) continue;
		//std::cout<<"["<<iter->pt.y<<","<<iter->pt.x<<"]"<<std::endl;
		//if all went well . create the patch and use it.


		/*********************************************************************************/
		/*							PATCH RELATED STUFF									**/
		/*********************************************************************************/

		Patch patch;
		patch.uvd = Point3d(x,y,depth/1000);
		Vector3f xyz = R*toWorldXYZ(patch.uvd)+t;



		Vector3f TL = R*toWorldXYZ(Point3d(x-patchSize/2,y-patchSize/2,depth/1000))+t;
		Vector3f TR = R*toWorldXYZ(Point3d(x+patchSize/2,y-patchSize/2,depth/1000))+t;
		Vector3f BL = R*toWorldXYZ(Point3d(x-patchSize/2,y+patchSize/2,depth/1000))+t;
		Vector3f BR = R*toWorldXYZ(Point3d(x+patchSize/2,y+patchSize/2,depth/1000))+t;

		patch.xyz = Point3d(xyz(0),xyz(1),xyz(2));
		Point3d NN;

		if(options.detector.kdtreeCheck)
		{
			if(kdtree.insert(patch.xyz,options.detector.kdtreeThresh,NN))
				continue;
		}

		patch.cornersXYZ.push_back(Point3d(TL(0),TL(1),TL(2)));
		patch.cornersXYZ.push_back(Point3d(TR(0),TR(1),TR(2)));
		patch.cornersXYZ.push_back(Point3d(BL(0),BL(1),BL(2)));
		patch.cornersXYZ.push_back(Point3d(BR(0),BR(1),BR(2)));

		patch.texture = Mat(_currentImage, Rect(xOrigin,yOrigin,patchSize,patchSize));
		//patch.depth = Mat(_currentImage, Rect(yOrigin,xOrigin,PATCH_WIDTH,PATCH_HEIGHT));
		patch.depth = patch.depth; // > The image is for some reason transposed
		patch.sourceKF = keyFrameCount;
		patch.supportList.insert(patch.supportList.end(),patchSupport(keyFrameCount,x,y));
		patchList.push_back(patch);

		/*********************************************************************************/
	}
}
void camera::undistortPoints(KeyPointsVector& k){
	//cv::undistortPoints()
}

void camera::showKeyPoints(const KeyPointsVector& k,std::string windowname){
	Mat outImage;
	drawKeypoints(_currentImage,k,outImage);
	imshow(windowname,outImage);
}


Mat& camera::currentImage(){
	return _currentImage;
}
Mat& camera::currentDepth(){
	return _currentDepth;
}

void camera::showPatchesOnImage(KeyPointsVector K,const char* windowName){
	Mat img = _currentImage.clone();
	KeyPointsVector::iterator iter;
	for(iter=K.begin(); iter!=K.end(); iter++)
	{
		int x = iter->pt.x;
		int y = iter->pt.y;
		int xOrigin = x - (PATCH_HEIGHT-1)/2;
		int yOrigin = y - (PATCH_WIDTH -1)/2;

		if(_currentDepth.at<unsigned short int>(iter->pt.y,iter->pt.x)>1 )
			rectangle(img,Rect(xOrigin,yOrigin,PATCH_WIDTH,PATCH_HEIGHT),255);
		else
			rectangle(img,Rect(xOrigin,yOrigin,PATCH_WIDTH,PATCH_HEIGHT),0);
	}

	imshow(windowName,img);
}
void camera :: detectAndExtractPatches(KeyPointsVector& kpts,PatchesVector& patchesVector){
	//detectPoints(kpts);
	//extractPatches(kpts,patchesVector);
}

/*
void camera :: matchKeyPoints(
				const PatchesVector prevImage,

				)
{
	MatrixXf Z(2*prevImage.size(),1);
	// Caculate the Rectangle around the search point and form a ROI for the image
	// then calculate the SSD based correlation and find the maximum
	PatchesVector::iterator iter;
	Vector3f t;
	int i;
	for(iter = prevImage.begin(),i=0; iter!=prevImage.end(); iter++,i++){
		t << iter->xyz.x,iter->xyz.y,iter->xyz.z;
	}
	int xOrigin = currentImage[0].pt.x - (searchRegionSize.height-1)/2;
	int yOrigin = currentImage[0].pt.y - (searchRegionSize.width -1)/2;

	Mat res;
	Rect searchRegion(xOrigin,yOrigin,searchRegionSize.width,searchRegionSize.height);
	matchTemplate(Mat(_currentImage,searchRegion),prevImage[0].texture,res,MATCHING_METHOD);
	Point minLoc;
	Point maxLoc;
	double minVal;
	double maxVal;
	minMaxLoc(res,&minVal,&maxVal,&minLoc,&maxLoc);

	// some  theresholding to determine if it raelly is match

}
*/


Vector3f camera::toWorldXYZ(Point3d uvd){
	static const double fx_d = 1.0 / _K.at<double>(0,0);
	static const double fy_d = 1.0 / _K.at<double>(1,1);
	static const double cx_d = _K.at<double>(0,2);
	static const double cy_d = _K.at<double>(1,2);

	double X = uvd.x;
	double Y = uvd.y;
	double depth = uvd.z;
	float x = float((X - cx_d) *fx_d);
	float y = float((Y - cy_d) *fy_d);
	float z = 1;

	Vector3f v(x,y,z);
	//v.normalize();
	return v*depth;
}

Mat& camera::K(){
	return _K;
}

void camera::showMatches(const PatchesVector& p,matchesList& M, const char* windowName)
{
	Mat image = _currentImage.clone();
	matchesList::iterator match;
	KeyPointsVector v;
	foralliter(match,M)
	{
		v.insert(v.end(),cv::KeyPoint(Point2f(match->u,match->v),1.,1.,1.,1,1));
	}
	showPatchesOnImage(v,windowName);
}

void camera::addSupport(PatchesVector& p, matchesList& matches, unsigned int keyFrameID){

	matchesList::iterator mIter;
	foralliter(mIter,matches)
	{
		p[(mIter->pointidx)].supportList.push_back(patchSupport(keyFrameID,mIter->u,mIter->v));
	}
}
